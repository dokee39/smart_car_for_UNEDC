/**
 * @file ring.c
 * @author dokee (dokee.39@gmail.com)
 * @brief 先进先出的有序环状数组, 如果溢出则自动省略掉溢出部分
 * @version 0.1
 * @date 2023-07-21
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "ring.h"
#include <string.h>

/**
 * @brief 初始化环状数组
 * 
 * @param pring 
 * @param p 
 * @param size 
 */
void ring_init(ring_t *pring, void *p, uint16_t size)
{
    pring->p = p;
    pring->size = size;

    pring->head = 0;
    pring->tail = 0;

    memset(pring->p, 0, pring->size);
}

/**
 * @brief 向环状数组末端添加数据
 * 
 * @param pring 
 * @param p 要添加的数据的地址
 * @param size 要添加的数据的字节数
 * @return RING_STATUS_t 
 */
RING_STATUS_t ring_append(ring_t *pring, void *p, uint16_t size)
{
    RING_STATUS_t RING_STATUS = RING_SUCCESS;

    // 判断是否溢出
    uint16_t space_remaining = pring->size - (pring->tail - pring->head);
    if (size > space_remaining)
    {
        RING_STATUS = RING_OVERFLOW;
        size = space_remaining;
    }

    // 数据添加
    if (pring->size > pring->tail && pring->tail + size > pring->size)
    {
        uint16_t size_part1 = pring->size - pring->tail;
        memcpy(((uint8_t *)pring->p + pring->tail), p, size_part1);
        memcpy(pring->p, ((uint8_t *)p + size_part1), size - size_part1);
    }
    else
    {
        memcpy(((uint8_t *)pring->p + (pring->tail % pring->size)), p, size);
    }

    // 光标移动
    pring->tail += size;

    return RING_STATUS;
}

/**
 * @brief 从环状数组的前端取出数据
 * 
 * @param pring 
 * @param p 取出的数据存放的地址
 * @param size 要取出的数据的字节数
 * @return RING_STATUS_t 
 */
RING_STATUS_t ring_fetch(ring_t *pring, void *p, uint16_t size)
{
    // 判断是否超范围读取
    uint16_t space_used = pring->tail - pring->head;
    if (size > space_used)
    {
        return RING_FAILURE;
    }
    else
    {

        // 数据读取
        if (pring->head + size < pring->size)
        {
            memcpy(p, ((uint8_t *)pring->p + pring->head), size);
            // 光标移动
            pring->head += size;
        }
        else
        {
            uint16_t size_part1 = pring->size - pring->head;
            memcpy(p, ((uint8_t *)pring->p + pring->head), size_part1);
            memcpy(((uint8_t *)p + size_part1), pring->p, size - size_part1);
            // 光标移动
            pring->head += size;
            pring->head -= pring->size;
            pring->tail -= pring->size;
        }
    }

    return RING_SUCCESS;
}

/**
 * @brief 获取环状数组中已经使用的字节数
 * 
 * @param pring 
 * @return uint16_t 
 */
uint16_t ring_usedspace_get(ring_t *pring)
{
    return pring->tail - pring->head;
}
