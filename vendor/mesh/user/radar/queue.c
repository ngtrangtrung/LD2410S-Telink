#include "queue.h"
#include "proj/common/tstring.h"

//static u8 queue_get_count(queue_t *q);

void  queues_init(queue_t *q,u8* buffer,u8 queue_size, u8 element_size)
{
    q->p=buffer;
    q->count=0;
    q->in=0;
    q->out=0;
    q->element_size=element_size;
    q->queue_size  =queue_size;
}

u8  queues_push(queue_t *q,u8* data)
{
    if(queue_count(q)>=q->queue_size)
    {
        return 0;
    }

    if(q->in>=((q->element_size)*(q->queue_size)))
    {
        q->in=0;
    }

    memcpy((u8*)&(q->p[q->in]),(u8*)data,q->element_size);
    q->in +=q->element_size;

    q->count++;

    return 1;
}

u8 queues_get(queue_t *q, u8* buffer)
{
    if(queue_count(q))
    {
        if(q->out>=((q->element_size)*(q->queue_size)))
        {
            q->out=0;
        }

        memcpy((u8*)buffer,(u8*)&(q->p[q->out]),q->element_size);
        q->out+=q->element_size;        
        q->count--;

        return 1;
    }

    return 0;
}

u8  queues_empty(queue_t *q)
{
    if(q->count)
    {
        return 0;
    }

    return 1;
}

u8 queue_count(queue_t *q)
{
    return q->count;
}

/* END_FILE */
