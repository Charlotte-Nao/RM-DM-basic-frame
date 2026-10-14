//
// Created by ASUS on 2026/9/10.
//

#include "remote.h"

#include <string.h>

static uint8_t remote_frame_buffer[REMOTE_FRAME_LENGTH];
static uint16_t remote_frame_length = 0U;

static int remote_unpack_frame(const uint8_t frame[REMOTE_FRAME_LENGTH],
    volatile remote_data_t *remote_data
);

int remote_parse(
    const uint8_t *data,
    uint16_t length,
    volatile remote_data_t *remote_data
)
{
    int result;

    if ((data == NULL) || (remote_data == NULL))
    {
        return -1;
    }

    if (length == 0U)
    {
        return 0;
    }

    if (length == REMOTE_FRAME_LENGTH)
    {
        remote_frame_length = 0U;

        result = remote_unpack_frame(data, remote_data);

        return (result == 0) ? 1 : 0;
    }


    if (length < REMOTE_FRAME_LENGTH)
    {
        if ((remote_frame_length + length) > REMOTE_FRAME_LENGTH)
        {
            remote_frame_length = 0U;
            return 0;
        }

        memcpy(&remote_frame_buffer[remote_frame_length], data,
               length);

        remote_frame_length += length;

        if (remote_frame_length < REMOTE_FRAME_LENGTH)
        {
            return 0;
        }

        remote_frame_length = 0U;

        result = remote_unpack_frame(remote_frame_buffer,
                                     remote_data);

        return (result == 0) ? 1 : 0;
    }

    remote_frame_length = 0U;

    return 0;
}

static int remote_unpack_frame(
    const uint8_t frame[REMOTE_FRAME_LENGTH],
    volatile remote_data_t *remote_data
)
{
    uint16_t ch0_raw;
    uint16_t ch1_raw;
    uint16_t ch2_raw;
    uint16_t ch3_raw;

    uint8_t s1;
    uint8_t s2;

    if ((frame == NULL) || (remote_data == NULL))
    {
        return -1;
    }

    ch0_raw = ((uint16_t)frame[0] | ((uint16_t)frame[1] << 8)) & 0x07FFU;
    ch1_raw = (((uint16_t)frame[1] >> 3) | ((uint16_t)frame[2] << 5)) & 0x07FFU;
    ch2_raw = (((uint16_t)frame[2] >> 6) |((uint16_t)frame[3] << 2) | ((uint16_t)frame[4] << 10)) & 0x07FFU;
    ch3_raw = (((uint16_t)frame[4] >> 1) |((uint16_t)frame[5] << 7)) & 0x07FFU;

    s1 = (uint8_t)((frame[5] >> 6) & 0x03U);
    s2 = (uint8_t)((frame[5] >> 4) & 0x03U);


    if ((ch0_raw < REMOTE_CHANNEL_MIN) ||(ch0_raw > REMOTE_CHANNEL_MAX) ||(ch1_raw < REMOTE_CHANNEL_MIN) ||(ch1_raw > REMOTE_CHANNEL_MAX) ||
        (ch2_raw < REMOTE_CHANNEL_MIN) ||(ch2_raw > REMOTE_CHANNEL_MAX) ||(ch3_raw < REMOTE_CHANNEL_MIN) ||(ch3_raw > REMOTE_CHANNEL_MAX))
    {
        return -2;
    }

    /* Each raw switch is two bits; 0 is invalid, 1/2/3 are valid. */
    if ((s1 == 0U) || (s2 == 0U))
    {
        return -2;
    }

    remote_data->rc.ch0 = (int16_t)((int32_t)ch2_raw - (int32_t)REMOTE_CHANNEL_CENTER);
    remote_data->rc.ch1 = (int16_t)((int32_t)ch3_raw - (int32_t)REMOTE_CHANNEL_CENTER);
    remote_data->rc.ch2 = (int16_t)((int32_t)ch0_raw - (int32_t)REMOTE_CHANNEL_CENTER);
    remote_data->rc.ch3 = (int16_t)((int32_t)ch1_raw - (int32_t)REMOTE_CHANNEL_CENTER);

    /* DBUS raw values: up=1, middle=3, down=2. */
    remote_data->rc.s1 = (s1 == 1U) ? REMOTE_SWITCH_UP :
                         (s1 == 3U) ? REMOTE_SWITCH_MID : REMOTE_SWITCH_DOWN;
    remote_data->rc.s2 = (s2 == 1U) ? REMOTE_SWITCH_UP :
                         (s2 == 3U) ? REMOTE_SWITCH_MID : REMOTE_SWITCH_DOWN;
    remote_data->rc.reserved = (int16_t)((int32_t)REMOTE_CHANNEL_CENTER -
        (int32_t)((uint16_t)frame[16] | ((uint16_t)frame[17] << 8)));

    return 0;
}