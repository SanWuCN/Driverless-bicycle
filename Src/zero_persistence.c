#include "zero_persistence.h"

#include "stm32f4xx_hal.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/* STM32F427II: reserve the final two 128 KiB sectors for an append-only log. */
#define ZERO_STORE_SECTOR_A_START       0x081C0000u
#define ZERO_STORE_SECTOR_B_START       0x081E0000u
#define ZERO_STORE_SECTOR_SIZE          0x00020000u
#define ZERO_STORE_SECTOR_A             FLASH_SECTOR_22
#define ZERO_STORE_SECTOR_B             FLASH_SECTOR_23

#define ZERO_RECORD_MAGIC               0x5A45524Fu /* "ZERO" */
#define ZERO_RECORD_VERSION             1u
#define ZERO_RECORD_COMMIT              0x434F4D54u /* "COMT" */
#define ZERO_SAVE_MIN_INTERVAL_MS       15000u
#define ZERO_SAVE_MIN_CHANGE_DEG        0.002f

typedef struct
{
    uint32_t magic;
    uint32_t version;
    uint32_t sequence;
    uint32_t zero_bits;
    uint32_t crc32;
    uint32_t sequence_inverse;
    uint32_t zero_bits_inverse;
    uint32_t commit;
} ZeroRecord;

typedef struct
{
    uint32_t start;
    uint32_t sector;
    uint32_t next_address;
    bool full;
    bool has_valid;
    ZeroRecord latest;
} SectorScan;

static const uint32_t sector_starts[2] = {
    ZERO_STORE_SECTOR_A_START,
    ZERO_STORE_SECTOR_B_START
};

static const uint32_t sector_numbers[2] = {
    ZERO_STORE_SECTOR_A,
    ZERO_STORE_SECTOR_B
};

static float factory_zero;
static float allowed_range;
static volatile float stored_zero;
static volatile uint32_t stored_sequence;
static volatile uint32_t persistence_status;
static uint32_t active_sector_index;
static uint32_t next_record_address;
static uint32_t last_save_attempt_ms;
static bool save_attempted;
static bool initialized;

static uint32_t crc32_words(const uint32_t *words, size_t word_count)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t word_index = 0u; word_index < word_count; word_index++)
    {
        uint32_t value = words[word_index];
        for (uint32_t byte_index = 0u; byte_index < 4u; byte_index++)
        {
            crc ^= value & 0xFFu;
            value >>= 8u;
            for (uint32_t bit_index = 0u; bit_index < 8u; bit_index++)
            {
                const uint32_t mask = 0u - (crc & 1u);
                crc = (crc >> 1u) ^ (0xEDB88320u & mask);
            }
        }
    }
    return ~crc;
}

static float bits_to_float(uint32_t bits)
{
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

static uint32_t float_to_bits(float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return bits;
}

static bool zero_is_allowed(float value)
{
    return isfinite(value) &&
           (value >= factory_zero - allowed_range) &&
           (value <= factory_zero + allowed_range);
}

static bool record_is_erased(const ZeroRecord *record)
{
    const uint32_t *words = (const uint32_t *)record;
    for (size_t index = 0u; index < (sizeof(*record) / sizeof(uint32_t)); index++)
    {
        if (words[index] != 0xFFFFFFFFu)
        {
            return false;
        }
    }
    return true;
}

static bool record_is_valid(const ZeroRecord *record)
{
    const float zero = bits_to_float(record->zero_bits);
    return (record->magic == ZERO_RECORD_MAGIC) &&
           (record->version == ZERO_RECORD_VERSION) &&
           (record->commit == ZERO_RECORD_COMMIT) &&
           (record->sequence_inverse == ~record->sequence) &&
           (record->zero_bits_inverse == ~record->zero_bits) &&
           (record->crc32 == crc32_words(&record->magic, 4u)) &&
           zero_is_allowed(zero);
}

static bool sequence_is_newer(uint32_t candidate, uint32_t reference)
{
    return (int32_t)(candidate - reference) > 0;
}

static SectorScan scan_sector(uint32_t index)
{
    SectorScan scan = {
        .start = sector_starts[index],
        .sector = sector_numbers[index],
        .next_address = sector_starts[index],
        .full = true,
        .has_valid = false
    };
    const uint32_t end = scan.start + ZERO_STORE_SECTOR_SIZE;

    for (uint32_t address = scan.start;
         address + sizeof(ZeroRecord) <= end;
         address += sizeof(ZeroRecord))
    {
        const ZeroRecord *record = (const ZeroRecord *)address;
        if (record_is_erased(record))
        {
            if (scan.full)
            {
                scan.next_address = address;
                scan.full = false;
            }
            continue;
        }

        if (record_is_valid(record) &&
            (!scan.has_valid || sequence_is_newer(record->sequence, scan.latest.sequence)))
        {
            memcpy(&scan.latest, record, sizeof(scan.latest));
            scan.has_valid = true;
        }
    }
    return scan;
}

static void reset_flash_data_cache(void)
{
    const bool data_cache_enabled = (FLASH->ACR & FLASH_ACR_DCEN) != 0u;
    if (data_cache_enabled)
    {
        __HAL_FLASH_DATA_CACHE_DISABLE();
    }
    __HAL_FLASH_DATA_CACHE_RESET();
    if (data_cache_enabled)
    {
        __HAL_FLASH_DATA_CACHE_ENABLE();
    }
}

static bool erase_sector(uint32_t sector)
{
    FLASH_EraseInitTypeDef erase = {
        .TypeErase = FLASH_TYPEERASE_SECTORS,
        .Banks = FLASH_BANK_2,
        .Sector = sector,
        .NbSectors = 1u,
        .VoltageRange = FLASH_VOLTAGE_RANGE_3
    };
    uint32_t sector_error = 0xFFFFFFFFu;

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR |
                           FLASH_FLAG_RDERR);

    if (HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK)
    {
        return false;
    }
    reset_flash_data_cache();
    return sector_error == 0xFFFFFFFFu;
}

static bool program_record(uint32_t address, const ZeroRecord *record)
{
    const uint32_t *words = (const uint32_t *)record;
    const size_t commit_index = offsetof(ZeroRecord, commit) / sizeof(uint32_t);

    for (size_t index = 0u; index < commit_index; index++)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                              address + (uint32_t)(index * sizeof(uint32_t)),
                              words[index]) != HAL_OK)
        {
            return false;
        }
    }

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                          address + (uint32_t)offsetof(ZeroRecord, commit),
                          record->commit) != HAL_OK)
    {
        return false;
    }
    return record_is_valid((const ZeroRecord *)address);
}

static bool append_zero(float zero_deg)
{
    ZeroRecord record;
    uint32_t write_address = next_record_address;
    uint32_t target_sector_index = active_sector_index;
    const uint32_t active_end = sector_starts[active_sector_index] + ZERO_STORE_SECTOR_SIZE;

    if (write_address + sizeof(record) > active_end)
    {
        target_sector_index = 1u - active_sector_index;
        if (HAL_FLASH_Unlock() != HAL_OK)
        {
            return false;
        }
        const bool erased = erase_sector(sector_numbers[target_sector_index]);
        HAL_FLASH_Lock();
        if (!erased)
        {
            return false;
        }
        write_address = sector_starts[target_sector_index];
    }

    record.magic = ZERO_RECORD_MAGIC;
    record.version = ZERO_RECORD_VERSION;
    record.sequence = stored_sequence + 1u;
    record.zero_bits = float_to_bits(zero_deg);
    record.crc32 = crc32_words(&record.magic, 4u);
    record.sequence_inverse = ~record.sequence;
    record.zero_bits_inverse = ~record.zero_bits;
    record.commit = ZERO_RECORD_COMMIT;

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return false;
    }
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR |
                           FLASH_FLAG_RDERR);
    const bool programmed = program_record(write_address, &record);
    HAL_FLASH_Lock();
    if (!programmed)
    {
        return false;
    }

    active_sector_index = target_sector_index;
    next_record_address = write_address + sizeof(record);
    stored_zero = zero_deg;
    stored_sequence = record.sequence;
    persistence_status |= ZERO_PERSISTENCE_VALID |
                          ZERO_PERSISTENCE_SAVED_THIS_BOOT;
    persistence_status &= ~ZERO_PERSISTENCE_ERROR;
    return true;
}

void zero_persistence_init(float factory_zero_deg,
                           float allowed_range_deg,
                           float *startup_zero_deg)
{
    factory_zero = factory_zero_deg;
    allowed_range = fabsf(allowed_range_deg);
    stored_zero = factory_zero;
    stored_sequence = 0u;
    persistence_status = 0u;
    last_save_attempt_ms = 0u;
    save_attempted = false;

    const SectorScan sector_a = scan_sector(0u);
    const SectorScan sector_b = scan_sector(1u);
    const SectorScan *active = &sector_a;
    uint32_t active_index = 0u;

    if (sector_b.has_valid &&
        (!sector_a.has_valid ||
         sequence_is_newer(sector_b.latest.sequence, sector_a.latest.sequence)))
    {
        active = &sector_b;
        active_index = 1u;
    }
    else if (!sector_a.has_valid && !sector_b.has_valid && sector_a.full && !sector_b.full)
    {
        active = &sector_b;
        active_index = 1u;
    }

    active_sector_index = active_index;
    next_record_address = active->full
                              ? (active->start + ZERO_STORE_SECTOR_SIZE)
                              : active->next_address;

    if (active->has_valid)
    {
        stored_zero = bits_to_float(active->latest.zero_bits);
        stored_sequence = active->latest.sequence;
        persistence_status |= ZERO_PERSISTENCE_VALID;
    }

    if (startup_zero_deg != NULL)
    {
        *startup_zero_deg = stored_zero;
    }
    initialized = true;
}

void zero_persistence_service(float candidate_zero_deg, bool learning_active)
{
    if (!initialized || !learning_active || !zero_is_allowed(candidate_zero_deg))
    {
        return;
    }

    const uint32_t now_ms = HAL_GetTick();
    if (fabsf(candidate_zero_deg - stored_zero) < ZERO_SAVE_MIN_CHANGE_DEG)
    {
        return;
    }
    if (save_attempted &&
        ((uint32_t)(now_ms - last_save_attempt_ms) < ZERO_SAVE_MIN_INTERVAL_MS))
    {
        return;
    }

    last_save_attempt_ms = now_ms;
    save_attempted = true;
    if (!append_zero(candidate_zero_deg))
    {
        persistence_status |= ZERO_PERSISTENCE_ERROR;
    }
}

uint32_t zero_persistence_status(void)
{
    return persistence_status;
}

float zero_persistence_stored_zero(void)
{
    return stored_zero;
}

uint32_t zero_persistence_sequence(void)
{
    return stored_sequence;
}
