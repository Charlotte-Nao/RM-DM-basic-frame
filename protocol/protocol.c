//
// Created by charlotte on 7/21/26.
//

#include "protocol.h"

#include <string.h>

static uint8_t protocol_checksum(const uint8_t *data, uint16_t length)
{
    uint8_t checksum = 0U;

    for (uint16_t i = 0U; i < length; i++) {
        checksum = (uint8_t)(checksum + data[i]);
    }

    return checksum;
}

int protocol_pack(const struct protocol_data *data, uint8_t frame[20])
{
    if (data == NULL || frame == NULL) {
        return -1;
    }

    frame[0] = 0xAAU;
    frame[1] = 0x55U;

    memcpy(&frame[2], &data->x, sizeof(data->x));
    memcpy(&frame[6], &data->y, sizeof(data->y));
    memcpy(&frame[10], &data->z, sizeof(data->z));
    memcpy(&frame[14], &data->roll, sizeof(data->roll));

    frame[16] = data->action;
    frame[17] = protocol_checksum(&frame[2], 15U);

    frame[18] = 0x0DU;
    frame[19] = 0x0AU;

    return 0;
}

int protocol_unpack(const uint8_t frame[20],
                    struct protocol_data *data)
{
    struct protocol_data unpacked_data;

    if (frame == NULL || data == NULL) {
        return -1;
    }

    if (frame[0] != 0xAAU || frame[1] != 0x55U) {
        return -1;
    }

    if (frame[18] != 0x0DU || frame[19] != 0x0AU) {
        return -1;
    }

    if (frame[17] != protocol_checksum(&frame[2], 15U)) {
        return -1;
    }

    memcpy(&unpacked_data.x, &frame[2], sizeof(unpacked_data.x));
    memcpy(&unpacked_data.y, &frame[6], sizeof(unpacked_data.y));
    memcpy(&unpacked_data.z, &frame[10], sizeof(unpacked_data.z));
    memcpy(&unpacked_data.roll, &frame[14], sizeof(unpacked_data.roll));

    unpacked_data.action = frame[16];

    *data = unpacked_data;

    return 0;
}

int protocol_parse(const uint8_t *bytes,
                   uint16_t length,
                   struct protocol_data *data)
{
    static uint8_t frame[20];
    static uint16_t frame_position = 0U;

    struct protocol_data unpacked_data;
    int valid_frame_count = 0;

    if (bytes == NULL || data == NULL) {
        return -1;
    }

    for (uint16_t i = 0U; i < length; i++) {
        uint8_t byte = bytes[i];

        if (frame_position == 0U) {
            if (byte == 0xAAU) {
                frame[0] = byte;
                frame_position = 1U;
            }

            continue;
        }

        if (frame_position == 1U) {
            if (byte == 0x55U) {
                frame[1] = byte;
                frame_position = 2U;
            } else if (byte == 0xAAU) {
                frame[0] = byte;
            } else {
                frame_position = 0U;
            }

            continue;
        }

        frame[frame_position] = byte;
        frame_position++;

        if (frame_position == 20U) {
            if (protocol_unpack(frame, &unpacked_data) == 0) {
                *data = unpacked_data;
                valid_frame_count++;
            }

            frame_position = 0U;
        }
    }

    return valid_frame_count;
}