#include "uart7_bridge.h"

#include "main.h"
#include "task.h"
#include "imu.h"
#include "steering.h"
#include "drive_control.h"
#include "balance_monitor.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define UART7_RX_BUFFER_SIZE 256u
#define UART7_RX_BUFFER_MASK (UART7_RX_BUFFER_SIZE - 1u)
#define UART7_COMMAND_LINE_SIZE 128u
#define UART7_TX_SPIN_LIMIT 1000000u

volatile uint8_t uart7_last_rx;
volatile uint32_t uart7_rx_count;
volatile uint32_t uart7_rx_overflow_count;

static volatile uint8_t uart7_rx_buffer[UART7_RX_BUFFER_SIZE];
static volatile uint16_t uart7_rx_head;
static volatile uint16_t uart7_rx_tail;

static void uart7_store_received(uint8_t received)
{
    const uint16_t next_head =
        (uint16_t)((uart7_rx_head + 1u) & UART7_RX_BUFFER_MASK);
    uart7_last_rx = received;
    uart7_rx_count++;
    if (next_head != uart7_rx_tail)
    {
        uart7_rx_buffer[uart7_rx_head] = received;
        uart7_rx_head = next_head;
    }
    else
    {
        uart7_rx_overflow_count++;
    }
}

/*
 * RX normally arrives through UART7_IRQHandler. Polling the data register here
 * is a low-cost fallback for a missed/disabled NVIC interrupt and also makes a
 * one-way UART wiring fault distinguishable from an interrupt configuration
 * fault. Re-check RXNE after masking the IRQ so a byte cannot be consumed twice.
 */
static void uart7_poll_receive(void)
{
    if (!LL_USART_IsActiveFlag_RXNE(UART7) &&
        !LL_USART_IsActiveFlag_ORE(UART7))
    {
        return;
    }

    NVIC_DisableIRQ(UART7_IRQn);
    __DSB();
    if (LL_USART_IsActiveFlag_RXNE(UART7))
    {
        uart7_store_received(LL_USART_ReceiveData8(UART7));
    }
    if (LL_USART_IsActiveFlag_ORE(UART7))
    {
        LL_USART_ClearFlag_ORE(UART7);
    }
    NVIC_EnableIRQ(UART7_IRQn);
}

static uint16_t crc16_ccitt(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFu;
    for (size_t index = 0u; index < length; index++)
    {
        crc ^= (uint16_t)data[index] << 8u;
        for (uint8_t bit = 0u; bit < 8u; bit++)
        {
            crc = (crc & 0x8000u) != 0u
                      ? (uint16_t)((crc << 1u) ^ 0x1021u)
                      : (uint16_t)(crc << 1u);
        }
    }
    return crc;
}

static bool uart7_write_bounded(const char *text, size_t length)
{
    for (size_t index = 0u; index < length; index++)
    {
        uint32_t spins = 0u;
        while (!LL_USART_IsActiveFlag_TXE(UART7))
        {
            if (++spins >= UART7_TX_SPIN_LIMIT)
            {
                return false;
            }
        }
        LL_USART_TransmitData8(UART7, (uint8_t)text[index]);
    }
    return true;
}

static void uart7_reply(uint32_t sequence, const char *status, const char *detail)
{
    char response[384];
    const int length = snprintf(response,
                                sizeof(response),
                                "#P,%lu,%s,%s\r\n",
                                (unsigned long)sequence,
                                status,
                                detail);
    if (length > 0)
    {
        const size_t output_length =
            ((size_t)length < sizeof(response)) ? (size_t)length
                                                : (sizeof(response) - 1u);
        (void)uart7_write_bounded(response, output_length);
    }
}

static const char *tuning_error_text(BalanceTuningResult result)
{
    switch (result)
    {
        case BALANCE_TUNING_UNKNOWN_PARAMETER:
            return "UNKNOWN_PARAMETER";
        case BALANCE_TUNING_OUT_OF_RANGE:
            return "OUT_OF_RANGE";
        case BALANCE_TUNING_UNSAFE_STATE:
            return "REAR_WHEEL_RUNNING";
        default:
            return "INTERNAL";
    }
}

static bool parse_u32(const char *text, uint32_t *value)
{
    char *end = NULL;
    const unsigned long parsed = strtoul(text, &end, 10);
    if (text == end || end == NULL || *end != '\0')
    {
        return false;
    }
    *value = (uint32_t)parsed;
    return true;
}

static bool parse_finite_float(const char *text, float *value)
{
    char *end = NULL;
    const float parsed = strtof(text, &end);
    if (text == end || end == NULL || *end != '\0' || !isfinite(parsed))
    {
        return false;
    }
    *value = parsed;
    return true;
}

static void uart7_process_command(char *line)
{
    uint32_t sequence = 0u;
    char *checksum_separator = strrchr(line, ',');
    if (line[0] != '@' || checksum_separator == NULL)
    {
        uart7_reply(0u, "ERR", "FORMAT");
        return;
    }

    char *checksum_end = NULL;
    const unsigned long received_checksum =
        strtoul(checksum_separator + 1, &checksum_end, 16);
    if (checksum_end == checksum_separator + 1 ||
        checksum_end == NULL ||
        *checksum_end != '\0' ||
        received_checksum > 0xFFFFu)
    {
        uart7_reply(0u, "ERR", "CRC_FORMAT");
        return;
    }

    *checksum_separator = '\0';
    const char *payload = line + 1;
    const uint16_t calculated_checksum =
        crc16_ccitt((const uint8_t *)payload, strlen(payload));
    if (calculated_checksum != (uint16_t)received_checksum)
    {
        uart7_reply(0u, "ERR", "CRC_MISMATCH");
        return;
    }

    char *protocol = strtok(line + 1, ",");
    char *sequence_text = strtok(NULL, ",");
    char *action = strtok(NULL, ",");
    if (protocol == NULL || strcmp(protocol, "P") != 0 ||
        sequence_text == NULL || !parse_u32(sequence_text, &sequence) ||
        action == NULL)
    {
        uart7_reply(sequence, "ERR", "FORMAT");
        return;
    }

    if (strcmp(action, "GET") == 0)
    {
        if (strtok(NULL, ",") != NULL)
        {
            uart7_reply(sequence, "ERR", "FORMAT");
            return;
        }
        BalanceTuningParameters parameters;
        char detail[320];
        balance_tuning_get(&parameters);
        (void)snprintf(detail,
                       sizeof(detail),
                       "PARAMS,RATE_KP=%.6f,RATE_KI=%.6f,RATE_KD=%.6f,"
                       "ANGLE_KP=%.6f,ANGLE_KI=%.6f,ANGLE_KD=%.6f,"
                       "WHEEL_KP=%.6f,WHEEL_KI=%.6f,"
                       "STEER_KP=%.6f,STEER_KI=%.6f,STEER_KD=%.6f",
                       (double)parameters.rate_kp,
                       (double)parameters.rate_ki,
                       (double)parameters.rate_kd,
                       (double)parameters.angle_kp,
                       (double)parameters.angle_ki,
                       (double)parameters.angle_kd,
                       (double)parameters.wheel_kp,
                       (double)parameters.wheel_ki,
                       (double)parameters.steer_kp,
                       (double)parameters.steer_ki,
                       (double)parameters.steer_kd);
        uart7_reply(sequence, "OK", detail);
        return;
    }

    if (strcmp(action, "SET") == 0)
    {
        char *name = strtok(NULL, ",");
        char *value_text = strtok(NULL, ",");
        if (name == NULL || value_text == NULL || strtok(NULL, ",") != NULL)
        {
            uart7_reply(sequence, "ERR", "FORMAT");
            return;
        }
        char *value_end = NULL;
        const float requested_value = strtof(value_text, &value_end);
        if (value_text == value_end || value_end == NULL || *value_end != '\0' ||
            !isfinite(requested_value))
        {
            uart7_reply(sequence, "ERR", "VALUE_FORMAT");
            return;
        }
        float applied_value = 0.0f;
        const BalanceTuningResult result =
            balance_tuning_set(name, requested_value, &applied_value);
        if (result != BALANCE_TUNING_OK)
        {
            uart7_reply(sequence, "ERR", tuning_error_text(result));
            return;
        }
        char detail[80];
        (void)snprintf(detail,
                       sizeof(detail),
                       "SET,%s,%.6f,RAM_ONLY",
                       name,
                       (double)applied_value);
        uart7_reply(sequence, "OK", detail);
        return;
    }

    if (strcmp(action, "REVERT") == 0)
    {
        if (strtok(NULL, ",") != NULL)
        {
            uart7_reply(sequence, "ERR", "FORMAT");
            return;
        }
        const BalanceTuningResult result = balance_tuning_revert();
        if (result != BALANCE_TUNING_OK)
        {
            uart7_reply(sequence, "ERR", tuning_error_text(result));
            return;
        }
        uart7_reply(sequence, "OK", "REVERTED,RAM_ONLY");
        return;
    }

    if (strcmp(action, "TELEM") == 0)
    {
        char *mode = strtok(NULL, ",");
        if (mode == NULL || strtok(NULL, ",") != NULL)
        {
            uart7_reply(sequence, "ERR", "FORMAT");
            return;
        }
        if (strcmp(mode, "COMPACT") == 0)
        {
            balance_telemetry_set_mode(BALANCE_TELEMETRY_COMPACT);
            uart7_reply(sequence, "OK", "TELEM,COMPACT");
            return;
        }
        if (strcmp(mode, "LIVE") == 0)
        {
            balance_telemetry_set_mode(BALANCE_TELEMETRY_COMPACT);
            uart7_reply(sequence, "OK", "TELEM,LIVE");
            return;
        }
        if (strcmp(mode, "BASIC") == 0)
        {
            balance_telemetry_set_mode(BALANCE_TELEMETRY_BASIC);
            uart7_reply(sequence, "OK", "TELEM,BASIC");
            return;
        }
        if (strcmp(mode, "FULL") == 0)
        {
            balance_telemetry_set_mode(BALANCE_TELEMETRY_FULL_TEXT);
            uart7_reply(sequence, "OK", "TELEM,FULL");
            return;
        }
        uart7_reply(sequence, "ERR", "UNKNOWN_TELEMETRY_MODE");
        return;
    }

    if (strcmp(action, "STEER") == 0)
    {
        char *subcommand = strtok(NULL, ",");
        if (subcommand == NULL)
        {
            uart7_reply(sequence, "ERR", "FORMAT");
            return;
        }
        if (strcmp(subcommand, "DISABLE") == 0)
        {
            if (strtok(NULL, ",") != NULL)
            {
                uart7_reply(sequence, "ERR", "FORMAT");
                return;
            }
            steering_disable_command();
            uart7_reply(sequence, "OK", "STEER,DISABLED,CENTERING");
            return;
        }
        if (strcmp(subcommand, "KEEP") == 0)
        {
            if (strtok(NULL, ",") == NULL)
            {
                (void)steering_keepalive_command();
            }
            /* Deliberately no reply: keepalives must not congest telemetry. */
            return;
        }
        if (strcmp(subcommand, "CAPTURE") == 0)
        {
            if (strtok(NULL, ",") != NULL)
            {
                uart7_reply(sequence, "ERR", "FORMAT");
                return;
            }
            (void)steering_capture_heading_command(imu.yaw);
            uart7_reply(sequence, "OK", "STEER,HEADING_CAPTURED");
            return;
        }
        if (strcmp(subcommand, "CAL") == 0 ||
            strcmp(subcommand, "ANGLE") == 0 ||
            strcmp(subcommand, "HOLD") == 0)
        {
            char *value_text = strtok(NULL, ",");
            float value = 0.0f;
            if (value_text == NULL || strtok(NULL, ",") != NULL ||
                !parse_finite_float(value_text, &value))
            {
                uart7_reply(sequence, "ERR", "VALUE_FORMAT");
                return;
            }
            SteeringCommandResult result;
            if (strcmp(subcommand, "CAL") == 0)
            {
                result = steering_calibration_command(value);
            }
            else if (strcmp(subcommand, "ANGLE") == 0)
            {
                result = steering_angle_command(value);
            }
            else
            {
                result = steering_heading_command(value);
            }
            if (result == STEERING_COMMAND_BAD_VALUE)
            {
                uart7_reply(sequence, "ERR", "STEER_VALUE_RANGE");
                return;
            }
            if (result == STEERING_COMMAND_UNSAFE_STATE)
            {
                uart7_reply(sequence, "ERR", "STEER_UNSAFE_STATE");
                return;
            }
            char detail[96];
            (void)snprintf(detail,
                           sizeof(detail),
                           "STEER,%s,%.3f,REFRESH_WITHIN_3000MS",
                           subcommand,
                           (double)value);
            uart7_reply(sequence, "OK", detail);
            return;
        }
        uart7_reply(sequence, "ERR", "UNKNOWN_STEER_COMMAND");
        return;
    }

    if (strcmp(action, "ODRIVE") == 0)
    {
        char *subcommand = strtok(NULL, ",");
        if (subcommand == NULL || strtok(NULL, ",") != NULL)
        {
            uart7_reply(sequence, "ERR", "FORMAT");
            return;
        }
        if (strcmp(subcommand, "RECOVER") == 0)
        {
            const BalanceRecoveryResult result =
                balance_odrive_recovery_command();
            if (result == BALANCE_RECOVERY_ALREADY_ARMED)
            {
                uart7_reply(sequence, "ERR", "BALANCE_ALREADY_ARMED");
                return;
            }
            if (result == BALANCE_RECOVERY_REAR_WHEEL_RUNNING)
            {
                uart7_reply(sequence, "ERR", "REAR_WHEEL_RUNNING");
                return;
            }
            uart7_reply(sequence,
                        "OK",
                        "ODRIVE,RECOVERY_PENDING,UPRIGHT_REQUIRED");
            return;
        }
        uart7_reply(sequence, "ERR", "UNKNOWN_ODRIVE_COMMAND");
        return;
    }

    if (strcmp(action, "DRIVE") == 0)
    {
        char *subcommand = strtok(NULL, ",");
        if (subcommand == NULL)
        {
            uart7_reply(sequence, "ERR", "FORMAT");
            return;
        }
        if (strcmp(subcommand, "KEEP") == 0)
        {
            if (strtok(NULL, ",") == NULL)
            {
                (void)drive_keepalive_command();
            }
            /* Deliberately no reply: see STEER,KEEP above. */
            return;
        }
        if (strcmp(subcommand, "STOP") == 0)
        {
            if (strtok(NULL, ",") != NULL)
            {
                uart7_reply(sequence, "ERR", "FORMAT");
                return;
            }
            drive_stop_command();
            uart7_reply(sequence, "OK", "DRIVE,STOPPING");
            return;
        }
        if (strcmp(subcommand, "MANUAL") == 0 ||
            strcmp(subcommand, "DEMO") == 0)
        {
            char *value_text = strtok(NULL, ",");
            float speed_mps = 0.0f;
            if (value_text == NULL || strtok(NULL, ",") != NULL ||
                !parse_finite_float(value_text, &speed_mps))
            {
                uart7_reply(sequence, "ERR", "VALUE_FORMAT");
                return;
            }
            const DriveCommandResult result =
                strcmp(subcommand, "MANUAL") == 0
                    ? drive_manual_command(speed_mps)
                    : drive_demo_command(speed_mps);
            if (result == DRIVE_COMMAND_BAD_VALUE)
            {
                uart7_reply(sequence, "ERR", "DRIVE_SPEED_RANGE");
                return;
            }
            if (result == DRIVE_COMMAND_BALANCE_NOT_ARMED)
            {
                uart7_reply(sequence, "ERR", "DRIVE_BALANCE_NOT_ARMED");
                return;
            }
            if (result == DRIVE_COMMAND_REAR_FEEDBACK_NOT_READY)
            {
                uart7_reply(sequence, "ERR", "DRIVE_AXIS1_NOT_READY");
                return;
            }
            if (result == DRIVE_COMMAND_FALL_DISARMED)
            {
                uart7_reply(sequence, "ERR", "DRIVE_FALL_RECOVERY_PENDING");
                return;
            }
            if (result == DRIVE_COMMAND_ODOMETRY_NOT_READY)
            {
                uart7_reply(sequence, "ERR", "DRIVE_ENCODER_FIRST_FRAME_PENDING");
                return;
            }
            if (result == DRIVE_COMMAND_ALREADY_ACTIVE)
            {
                uart7_reply(sequence, "ERR", "DRIVE_ALREADY_ACTIVE");
                return;
            }
            if (result == DRIVE_COMMAND_REAR_WHEEL_MOVING)
            {
                uart7_reply(sequence, "ERR", "DRIVE_REAR_WHEEL_MOVING");
                return;
            }
            char detail[96];
            (void)snprintf(detail,
                           sizeof(detail),
                           "DRIVE,%s,%.3f,REFRESH_WITHIN_3000MS",
                           subcommand,
                           (double)speed_mps);
            uart7_reply(sequence, "OK", detail);
            return;
        }
        uart7_reply(sequence, "ERR", "UNKNOWN_DRIVE_COMMAND");
        return;
    }

    uart7_reply(sequence, "ERR", "UNKNOWN_ACTION");
}

void uart7_tuning_service(void)
{
    static char line[UART7_COMMAND_LINE_SIZE];
    static size_t line_length;
    static bool receiving_command;
    static bool discard_line;

    uart7_poll_receive();

    while (uart7_rx_tail != uart7_rx_head)
    {
        const uint8_t byte = uart7_rx_buffer[uart7_rx_tail];
        uart7_rx_tail = (uint16_t)((uart7_rx_tail + 1u) & UART7_RX_BUFFER_MASK);

        if (!receiving_command)
        {
            if (byte == '@')
            {
                receiving_command = true;
                discard_line = false;
                line_length = 0u;
                line[line_length++] = (char)byte;
            }
            continue;
        }

        if (byte == '\r')
        {
            continue;
        }
        if (byte == '\n')
        {
            if (!discard_line && line_length > 0u)
            {
                line[line_length] = '\0';
                uart7_process_command(line);
            }
            receiving_command = false;
            discard_line = false;
            line_length = 0u;
            continue;
        }
        if (line_length + 1u >= sizeof(line))
        {
            discard_line = true;
            continue;
        }
        if (!discard_line)
        {
            line[line_length++] = (char)byte;
        }
    }
}

void UART7_IRQHandler(void)
{
    if (LL_USART_IsActiveFlag_RXNE(UART7) &&
        LL_USART_IsEnabledIT_RXNE(UART7))
    {
        const uint8_t received = LL_USART_ReceiveData8(UART7);
        uart7_store_received(received);

#ifdef BIKE_UART7_ECHO
        uint32_t spins = 0u;
        while (!LL_USART_IsActiveFlag_TXE(UART7) &&
               (++spins < UART7_TX_SPIN_LIMIT))
        {
        }
        if (LL_USART_IsActiveFlag_TXE(UART7))
        {
            LL_USART_TransmitData8(UART7, received);
        }
#endif
    }

    if (LL_USART_IsActiveFlag_ORE(UART7))
    {
        LL_USART_ClearFlag_ORE(UART7);
    }
}
