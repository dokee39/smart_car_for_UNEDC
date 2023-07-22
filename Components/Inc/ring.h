/**
 * @file ring.h
 * @author dokee (dokee.39@gmail.com)
 * @brief 先进先出的有序环状数组, 如果溢出则自动省略掉溢出部分
 * @version 0.1
 * @date 2023-07-21
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef _RING_H_
#define _RING_H_

#include <stdint.h>

typedef enum
{
    RING_SUCCESS = 0u,
    RING_OVERFLOW = 1u,
    RING_FAILURE = 2u,
} RING_STATUS_t;

typedef struct
{
    void *p;
    uint16_t size;

    uint16_t head; // 范围为 0 ~ size-1
    uint16_t tail; // 范围为 0 ~ 2*size-1, 这里当 tail < head 时将 tail 映射到 size ~ 2*size-1 上以简化运算
} ring_t;

void ring_init(ring_t *pring, void *p, uint16_t size);
RING_STATUS_t ring_append(ring_t *pring, void *p, uint16_t size);
RING_STATUS_t ring_fetch(ring_t *pring, void *p, uint16_t size);
uint16_t ring_usedspace_get(ring_t *pring);

#endif // !_RING_H_
