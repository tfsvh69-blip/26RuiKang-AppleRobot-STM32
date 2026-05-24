/**
 * @file sbus.c
 * @brief Implement SBUS protocol parsing to retrieve remote control channels.
 */

#include "sbus.h"
#include <string.h>

#define SBUS_FRAME_SIZE 25
#define SBUS_HEADER     0x0F
#define SBUS_FOOTER     0x00

/* Global storage for parsed channels */
//接收机获得的数据
volatile uint16_t sbus_channels[SBUS_CHANNELS] = {0};
static volatile uint8_t  frame_lost = 0;

/* Statistics */
static volatile uint32_t sbus_frame_ok_cnt = 0;
static volatile uint32_t sbus_frame_err_cnt = 0;

/* Frame synchronization state */
static uint8_t s_sbus_buf[SBUS_FRAME_SIZE];
static uint8_t s_sbus_idx = 0;

void SBUS_Init(void)
{
    memset((void*)sbus_channels, 0, sizeof(sbus_channels));
    frame_lost = 0;
    sbus_frame_ok_cnt = 0;
    sbus_frame_err_cnt = 0;
    s_sbus_idx = 0;
}

void SBUS_ProcessStream(const uint8_t *data, uint16_t len)
{
    if (!data || len == 0)
        return;

    for (uint16_t i = 0; i < len; i++)
    {
        uint8_t b = data[i];

        if (s_sbus_idx == 0)
        {
            if (b == SBUS_HEADER)
            {
                s_sbus_buf[s_sbus_idx++] = b;
            }
            continue;
        }

        s_sbus_buf[s_sbus_idx++] = b;

        if (s_sbus_idx >= SBUS_FRAME_SIZE)
        {
            /* Check frame checksum: Header and Footer must match */
            if (s_sbus_buf[0] == SBUS_HEADER && s_sbus_buf[24] == SBUS_FOOTER)
            {
                /* Decode 11-bit channels */
                sbus_channels[0] = (uint16_t)(( (s_sbus_buf[1]       | (uint16_t)s_sbus_buf[2]  << 8)                           ) & 0x07FFu);
                sbus_channels[1] = (uint16_t)((((s_sbus_buf[2] >> 3) | (uint16_t)s_sbus_buf[3]  << 5)                           ) & 0x07FFu);
                sbus_channels[2] = (uint16_t)((((s_sbus_buf[3] >> 6) | (uint16_t)s_sbus_buf[4]  << 2 | (uint16_t)s_sbus_buf[5] << 10)) & 0x07FFu);
                sbus_channels[3] = (uint16_t)((((s_sbus_buf[5] >> 1) | (uint16_t)s_sbus_buf[6]  << 7)                           ) & 0x07FFu);
                sbus_channels[4] = (uint16_t)((((s_sbus_buf[6] >> 4) | (uint16_t)s_sbus_buf[7]  << 4)                           ) & 0x07FFu);
                sbus_channels[5] = (uint16_t)((((s_sbus_buf[7] >> 7) | (uint16_t)s_sbus_buf[8]  << 1 | (uint16_t)s_sbus_buf[9] << 9 )) & 0x07FFu);
                sbus_channels[6] = (uint16_t)((((s_sbus_buf[9] >> 2) | (uint16_t)s_sbus_buf[10] << 6)                           ) & 0x07FFu);
                sbus_channels[7] = (uint16_t)((((s_sbus_buf[10]>> 5) | (uint16_t)s_sbus_buf[11] << 3)                           ) & 0x07FFu);
                sbus_channels[8] = (uint16_t)(( (s_sbus_buf[12]      | (uint16_t)s_sbus_buf[13] << 8)                           ) & 0x07FFu);
                sbus_channels[9] = (uint16_t)((((s_sbus_buf[13]>> 3) | (uint16_t)s_sbus_buf[14] << 5)                           ) & 0x07FFu);
                
                frame_lost = (s_sbus_buf[23] & 0x04u) ? 1u : 0u;
                sbus_frame_ok_cnt++;
            }
            else
            {
                sbus_frame_err_cnt++;
            }

            /* Reset frame byte index to synchronize next frame */
            s_sbus_idx = 0;
        }
    }
}

uint16_t SBUS_GetChannel(uint8_t channel_idx)
{
    if (channel_idx < SBUS_CHANNELS)
    {
        return sbus_channels[channel_idx];
    }
    return 0;
}

uint8_t SBUS_IsFrameLost(void)
{
    return frame_lost;
}

void SBUS_GetStats(uint32_t *ok_cnt, uint32_t *err_cnt)
{
    if (ok_cnt)
        *ok_cnt = sbus_frame_ok_cnt;
    if (err_cnt)
        *err_cnt = sbus_frame_err_cnt;
}
