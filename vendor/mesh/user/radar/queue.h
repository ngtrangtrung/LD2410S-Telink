#ifndef _QUEUE_H_
#define _QUEUE_H_

#include "proj/common/types.h"

typedef struct queue_t
{
    u8 *p;
    u8  element_size;
    u8  queue_size;
    u8  count;
    u16 in;
    u16 out;
}queue_t;

void  queues_init(queue_t *q,u8* buffer,u8 queue_size, u8 element_size);
u8    queues_push(queue_t *q,u8* data);
u8    queues_get(queue_t *q, u8* buffer);
u8    queues_empty(queue_t *q);
u8    queue_count(queue_t *q);

#endif

/* _QUEUE_H_ */
