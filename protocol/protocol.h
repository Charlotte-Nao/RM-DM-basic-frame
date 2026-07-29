//
// Created by charlotte on 7/21/26.
//

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

enum {
    PROTOCOL_FRAME_SIZE = 20
};

struct protocol_data {
    float x;
    float y;
    float z;
    int16_t roll;
    uint8_t action;
};

int protocol_pack(const struct protocol_data *data, uint8_t frame[20]);

int protocol_unpack(const uint8_t frame[20],
                    struct protocol_data *data);

/*
 * 将USB读取到的连续字节交给协议解析器。
 *
 * 返回值：
 * > 0：成功解析出的有效帧数量
 * = 0：暂时没有完整帧
 * < 0：参数错误
 *
 * 如果一次收到多帧，data中保存最后一帧的数据。
 */
int protocol_parse(const uint8_t *bytes,
                   uint16_t length,
                   struct protocol_data *data);

#endif