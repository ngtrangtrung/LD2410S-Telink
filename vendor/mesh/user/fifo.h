/*
 * Copyright (c) 2019
 * Lumi, JSC.
 * All Rights Reserved
 *
 *
 * Description:
 *
 * Author: DungTran
 *
 * Last Changed By:  $Author: DungTran $
 * Revision:         $Revision: 1.0 $
 * Last Changed:     $Date: 1/7/19 $
 */

#ifndef FIFO_H_
#define FIFO_H_

#include "../../../proj/tl_common.h"

/* Can manage any type of item, must decide size in byte of item */
typedef struct Fifo_s
{
    uint16_t head;      // in items, not bytes
    uint16_t tail;      // in items, not bytes
    uint16_t length;    // in items, not bytes
    void* buffer;
    uint8_t itemSize;   // in bytes
} Fifo_t;

void FifoInit(Fifo_t* fifo, void* buffer, uint8_t itemSize, uint16_t length);
BOOL FifoPush(Fifo_t* fifo, void* data);
BOOL FifoPop(Fifo_t* fifo, void* data);
void FifoFlush(Fifo_t* fifo);
BOOL FifoIsEmpty(Fifo_t* fifo);
BOOL FifoIsFull(Fifo_t* fifo);

#endif /* FIFO_H_ */
