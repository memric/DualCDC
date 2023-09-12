/*
 * ring_buf.c
 *
 *  Created on: Sep 11, 2023
 *      Author: Valeriy Chudnikov
 */

#include "ring_buf.h"
#include <string.h>

/**
 * @brief       Ring Buffer initialization.
 *
 * @param hBuf  Pointer to Ring Buffer Handler.
 */
void RingBuf_Init(ring_buf_t *hBuf)
{
    RING_BUF_ASSERT(hBuf != NULL);
    RING_BUF_ASSERT(hBuf->item_size > 0);
    RING_BUF_ASSERT(hBuf->len > 0);

    hBuf->pHead = hBuf->pBuf;
    hBuf->pTail = hBuf->pBuf;
}

/**
 * @brief       Adding items to buffer.
 *
 * @param hBuf  Pointer to Ring Buffer Handler.
 * @param item  Pointer to items to push in buffer.
 * @param num   Items number.
 * @return      Number if items added to buffer.
 */
rbuf_size_t RingBuf_Push(ring_buf_t *hBuf, void *item, rbuf_size_t num)
{
    RING_BUF_ASSERT(hBuf != NULL);
    RING_BUF_ASSERT(item != NULL);

    void *lock_arg = NULL;
    rbuf_size_t ret = 0;
    rbuf_size_t i;

    if (hBuf->Lock != NULL)
    {
        hBuf->Lock(lock_arg);
    }

    for (i = 0; i < num; i++)
    {
        /* Check for overlap */
        if ((hBuf->pHead + hBuf->item_size) == hBuf->pTail)
        {
            break;
        }

        /* Append item */
        memcpy(hBuf->pHead, item, hBuf->item_size);
        hBuf->pHead += hBuf->item_size;
        item += hBuf->item_size;

        if (hBuf->pHead >= (hBuf->pBuf + hBuf->item_size * hBuf->len))
        {
            /* The head has reached the end of buffer */
            hBuf->pHead = hBuf->pBuf;
        }

        ret++;
    }

    if (hBuf->Unlock != NULL)
    {
        hBuf->Unlock(lock_arg);
    }

    return ret;
}

/**
 * @brief       Returns number of available items to read.
 *
 * @param hBuf  Pointer to Ring Buffer Handler.
 * @return      Number of items.
 */
rbuf_size_t RingBuf_GetNum(ring_buf_t *hBuf)
{
    RING_BUF_ASSERT(hBuf != NULL);

    void *lock_arg = NULL;
    rbuf_size_t ret = 0;

    if (hBuf->Lock != NULL)
    {
        hBuf->Lock(lock_arg);
    }

    rbuf_size_t head_ind = (hBuf->pHead - hBuf->pBuf) / hBuf->item_size;
    rbuf_size_t tail_ind = (hBuf->pTail - hBuf->pBuf) / hBuf->item_size;

    if (head_ind < tail_ind)
    {
        /* Wrap is needed */
        ret = hBuf->len - tail_ind + head_ind;
    }
    else
    {
        ret = head_ind - tail_ind;
    }

    if (hBuf->Unlock != NULL)
    {
        hBuf->Unlock(lock_arg);
    }

    return ret;
}

/**
 * @brief       Reading items from ring buffer.
 *
 * @param hBuf  Pointer to Ring Buffer Handler.
 * @param out   Pointer to output buffer.
 * @param num   Number of items to read.
 * @return      Number of items have been successfully read.
 */
rbuf_size_t RingBuf_Pop(ring_buf_t *hBuf, void *out, rbuf_size_t num)
{
    RING_BUF_ASSERT(hBuf != NULL);
    RING_BUF_ASSERT(out != NULL);

    void *lock_arg = NULL;
    rbuf_size_t ret = 0;

    if (hBuf->Lock != NULL)
    {
        hBuf->Lock(lock_arg);
    }

    while (num && (hBuf->pTail != hBuf->pHead))
    {
        /* Copy items to output buffer */

        memcpy(out, hBuf->pTail, hBuf->item_size);
        out += hBuf->item_size;
        hBuf->pTail += hBuf->item_size;

        if (hBuf->pTail >= (hBuf->pBuf + hBuf->item_size * hBuf->len))
        {
            /* The tail has reached the end of buffer */
            hBuf->pTail = hBuf->pBuf;
        }

        num--;
        ret++;
    }

    if (hBuf->Unlock != NULL)
    {
        hBuf->Unlock(lock_arg);
    }

    return ret;
}
