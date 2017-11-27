#include <stdlib.h>
#include <string.h>

#include "cros_message_queue.h"

void cRosMessageQueueInit(cRosMessageQueue *q)
{
  unsigned int msg_ind;
  // Initialize all messages in the queue so that the inserted messages only need to be copied over these ones
  for(msg_ind=0;msg_ind<MAX_QUEUE_LEN;msg_ind++)
    cRosMessageInit(&q->msgs[msg_ind]);
  q->length = 0;
  q->first_msg_ind = 0;
}

void cRosMessageQueueClear(cRosMessageQueue *q)
{
  // Empty the queue
  while(q->length > 0)
  {
    // Delete fields from message to remove
    cRosMessageFieldsFree(&q->msgs[q->first_msg_ind]);
    // The queue is internally implemented as a circular buffer
    q->first_msg_ind = (q->first_msg_ind + 1) % MAX_QUEUE_LEN;
    q->length--;
  }
  q->first_msg_ind = 0;
}

unsigned int cRosMessageQueueVacancies(cRosMessageQueue *q)
{
  return MAX_QUEUE_LEN - q->length;
}

unsigned int cRosMessageQueueUsage(cRosMessageQueue *q)
{
  return q->length;
}

void cRosMessageQueueRelease(cRosMessageQueue *q)
{
  unsigned int msg_ind;
  cRosMessageQueueClear(q);
  // Release the memory of all container messages in the queue
  for(msg_ind=0;msg_ind<MAX_QUEUE_LEN;msg_ind++)
    cRosMessageRelease(&q->msgs[msg_ind]);
}

int cRosMessageQueueAdd(cRosMessageQueue *q, cRosMessage *m)
{
  int ret;
  if(q->length < MAX_QUEUE_LEN)
  {
    unsigned int next_msg_pos;
    // The queue is internally implemented as a circular buffer
    next_msg_pos = (q->first_msg_ind + q->length) % MAX_QUEUE_LEN;
    ret = cRosMessageFieldsCopy(&q->msgs[next_msg_pos], m);
    q->length++;
  }
  else
    ret=-2;

  return ret;
}

int cRosMessageQueueGet(cRosMessageQueue *q, cRosMessage *m)
{
  int ret;
  if(q->length > 0)
  {
    cRosMessage *msg_to_peek;
    msg_to_peek = &q->msgs[q->first_msg_ind];
    ret = cRosMessageFieldsCopy(m, msg_to_peek);
  }
  else
    ret=-2;

  return ret;
}

int cRosMessageQueueRemove(cRosMessageQueue *q)
{
  int ret;

  if(q->length > 0)
  {
    cRosMessage *msg_to_remove;
    msg_to_remove = &q->msgs[q->first_msg_ind];
    // Delete fields from removed message
    cRosMessageFieldsFree(msg_to_remove);
    // The queue is internally implemented as a circular buffer
    q->first_msg_ind = (q->first_msg_ind + 1) % MAX_QUEUE_LEN;
    q->length--;
    ret=0;
  }
  else
    ret=-2;

  return ret;
}

int cRosMessageQueueExtract(cRosMessageQueue *q, cRosMessage *m)
{
  int ret;

  ret = cRosMessageQueueGet(q, m);
  if(ret == 0)
    ret = cRosMessageQueueRemove(q);

  return ret;
}

cRosMessage *cRosMessageQueuePeekFirst(cRosMessageQueue *q)
{
  cRosMessage *first_msg;
  if(q->length > 0)
    first_msg = &q->msgs[q->first_msg_ind];
  else
    first_msg = NULL;

  return first_msg;
}

cRosMessage *cRosMessageQueuePeekLast(cRosMessageQueue *q)
{
  cRosMessage *last_msg;
  if(q->length > 0)
  {
    unsigned int last_msg_pos;
    // The queue is internally implemented as a circular buffer
    last_msg_pos = (q->first_msg_ind + q->length - 1) % MAX_QUEUE_LEN;
    last_msg = &q->msgs[last_msg_pos];
  }
  else
    last_msg = NULL;

  return last_msg;
}
