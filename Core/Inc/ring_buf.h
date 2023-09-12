/*
 * ring_buf.h
 *
 *  Created on: Sep 11, 2023
 *      Author: Valeriy Chudnikov
 */

#ifndef INC_RING_BUF_H_
#define INC_RING_BUF_H_

#include <stdint.h>
#include <stddef.h>

#ifndef RING_BUF_SIZE_T
#define RING_BUF_SIZE_T             size_t
#endif

typedef RING_BUF_SIZE_T             rbuf_size_t;

/**
 * @brief   Ring Buffer handler type;
 *
 */
typedef struct {
        rbuf_size_t len;                /*!< Ring buffer items number */
        rbuf_size_t item_size;          /*!< Item byte size */
        void *pBuf;                     /*!< Pointer to buffer storage */
        void *pHead;                    /*!< Head pointer */
        void *pTail;                    /*!< Tail pointer */
        void (* Lock)(void *arg);       /*!< Unlock function pointer */
        void (* Unlock)(void *arg);     /*!< Unlock function pointer */
} ring_buf_t;

/**
 * @brief   Macro for Ring Buffer handler definition.
 *
 */
#define RING_BUF_DEFINE(name, size, type, lock, unlock) \
static type ring_buf_##name[size]; \
static ring_buf_t hRing_buf_##name = { \
    size, \
    sizeof(type), \
    ring_buf_##name, \
    ring_buf_##name, \
    ring_buf_##name, \
    lock, \
    unlock \
}

/**
 * @brief   Macro for Lock function registration.
 *
 */
#define RING_BUF_SET_LOCK(name, func) \
    hRing_buf_##name.Lock = func

/**
 * @brief   Macro for Unlock function registration.
 *
 */
#define RING_BUF_SET_UNLOCK(name, func) \
    hRing_buf_##name.Unlock = func

/**
 * @brief   Macro may be used to get Ring Buffer handler pointer by it's name.
 *
 */
#define pRING_BUF(name)                 (&hRing_buf_##name)

#ifndef RING_BUF_ASSERT
#define RING_BUF_ASSERT(expr)           ((void) 0U)
#endif

void RingBuf_Init(ring_buf_t *hBuf);
rbuf_size_t RingBuf_Push(ring_buf_t *hBuf, void *item, rbuf_size_t num);
rbuf_size_t RingBuf_GetNum(ring_buf_t *hBuf);
rbuf_size_t RingBuf_Pop(ring_buf_t *hBuf, void *out, rbuf_size_t num);

#endif /* INC_RING_BUF_H_ */
