#include <string.h>
#include <stdio.h>

#include "packet.h"
#include "imu_data_decode.h"

static packet_t RxPkt; /* used for data receive */

uint8_t bitmap;
volatile uint32_t acceleration_frame_count;

__align(4)  id0x91_t id0x91; /* HI226 HI229 CH100 CH110 HI221 protocol packet*/
__align(4)  id0x62_t id0x62; /* HI221 Dongle protocol packet*/

static int stream2int16(int *dest,uint8_t *src)
{
	dest[0] = (int16_t)(src[0] | src[1] << 8);
	dest[1] = (int16_t)(src[2] | src[3] << 8);
	dest[2] = (int16_t)(src[4] | src[5] << 8);
	return 0;
}


static bool item_fits(const packet_t *pkt, int offset, size_t item_size)
{
    return (offset >= 0) &&
           ((size_t)offset <= pkt->payload_len) &&
           (item_size <= ((size_t)pkt->payload_len - (size_t)offset));
}

/* Callback invoked only after the packet CRC has passed. */
static bool on_data_received(packet_t *pkt)
{
	int temp[3] = {0};
    int i = 0;
    int offset = 0;
    uint8_t *p = pkt->buf;

	if(pkt->type != 0xA5)
    {
        return false;
    }

	bitmap = 0;
	while(offset < pkt->payload_len)
	{
		switch(p[offset])
		{
            case kItemID:
                if (!item_fits(pkt, offset, 2u)) return false;
                bitmap |= BIT_VALID_ID;
                id0x91.id = p[offset + 1];
                offset += 2;
                break;

            case kItemAccRaw:
                if (!item_fits(pkt, offset, 7u)) return false;
                bitmap |= BIT_VALID_ACC;
                stream2int16(temp, p + offset + 1);
                id0x91.acc[0] = (float)temp[0] / 1000;
                id0x91.acc[1] = (float)temp[1] / 1000;
                id0x91.acc[2] = (float)temp[2] / 1000;
                offset += 7;
                break;

            case kItemGyrRaw:
                if (!item_fits(pkt, offset, 7u)) return false;
                bitmap |= BIT_VALID_GYR;
                stream2int16(temp, p + offset + 1);
                id0x91.gyr[0] = (float)temp[0] / 10;
                id0x91.gyr[1] = (float)temp[1] / 10;
                id0x91.gyr[2] = (float)temp[2] / 10;
                offset += 7;
                break;

            case kItemMagRaw:
                if (!item_fits(pkt, offset, 7u)) return false;
                bitmap |= BIT_VALID_MAG;
                stream2int16(temp, p + offset + 1);
                id0x91.mag[0] = (float)temp[0] / 10;
                id0x91.mag[1] = (float)temp[1] / 10;
                id0x91.mag[2] = (float)temp[2] / 10;
                offset += 7;
                break;

            case kItemRotationEul:
                if (!item_fits(pkt, offset, 7u)) return false;
                bitmap |= BIT_VALID_EUL;
                stream2int16(temp, p + offset + 1);
                id0x91.eul[1] = (float)temp[0] / 100;
                id0x91.eul[0] = (float)temp[1] / 100;
                id0x91.eul[2] = (float)temp[2] / 10;
                offset += 7;
                break;

            case kItemRotationQuat:
                if (!item_fits(pkt, offset, 17u)) return false;
                bitmap |= BIT_VALID_QUAT;
                memcpy((void *)id0x91.quat, p + offset + 1, sizeof( id0x91.quat));
                offset += 17;
                break;

            case kItemPressure:
                if (!item_fits(pkt, offset, 5u)) return false;
                offset += 5;
                break;

            case KItemIMUSOL:
                if (!item_fits(pkt, offset, sizeof(id0x91_t))) return false;
                bitmap = BIT_VALID_ALL;
                memcpy((void *)&id0x91, p + offset, sizeof(id0x91_t));
                offset += sizeof(id0x91_t);
                break;

            case KItemGWSOL:
            {
                if (!item_fits(pkt, offset, 8u)) return false;
                const uint8_t device_count = p[offset + 2];
                if ((device_count > MAX_LENGTH) ||
                    !item_fits(pkt,
                               offset,
                               8u + ((size_t)device_count * sizeof(id0x91_t))))
                {
                    return false;
                }
                memcpy((void *)&id0x62, p + offset, 8u);
                id0x62.n = device_count;
                offset += 8;
                for (i = 0; i < device_count; i++)
                {
                    bitmap = BIT_VALID_ALL;
                    memcpy((void *)&id0x62.id0x91[i],
                           p + offset,
                           sizeof(id0x91_t));
                    offset += sizeof(id0x91_t);
                }
                break;
			}

            default:
				offset++;
		}
    }
    const bool control_data_valid =
        (bitmap & (BIT_VALID_GYR | BIT_VALID_EUL)) ==
        (BIT_VALID_GYR | BIT_VALID_EUL);
    if (control_data_valid && (bitmap & BIT_VALID_ACC) != 0u)
    {
        acceleration_frame_count++;
    }
    return control_data_valid;
}


int imu_data_decode_init(void)
{
    packet_decode_init(&RxPkt, on_data_received);
    return 0;
}

