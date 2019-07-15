/*! \file cros_message_queue.h
 *  \brief This header file declares the cRosMessageQueue type and associated management functions
 *
 *  cRosMessageQueue implements a queue of TCPROS-protocol messages. This queue behaves as a
 *  FIFO (First In First Out).
 *  This queue only stores the fields of the messages ('fields' fields of the struct), not the message definition (msgDef field).
 *  \author Richard R. Carrillo. Aging in Vision and Action lab, Institut de la Vision, Sorbonne University, Paris, France.
 *  \date 31 Oct 2017
 */

#ifndef _CROS_MESSAGE_QUEUE_H_
#define _CROS_MESSAGE_QUEUE_H_

#include "cros_message.h"


#define MAX_QUEUE_LEN 10 //! Maximum number of messages that can be hold in the queue

struct cRosMessageQueue
{
  cRosMessage msgs[MAX_QUEUE_LEN]; //! Content of the queue
  unsigned int length; //! Number of messages currently in the queue
  unsigned int first_msg_ind; //! Index of the oldest message in the queue (the one that was inserted first)
};

typedef struct cRosMessageQueue cRosMessageQueue;

/*! \brief Initializes a queue.
 *
 *  This function must be called before using a queue for the first time.
 *  \param q Pointer to the queue.
 */
void cRosMessageQueueInit(cRosMessageQueue *q);

/*! \brief Empty queue.
 *
 *  This function removes all the messages in a queue.
 *  \param q Pointer to the queue.
 */
void cRosMessageQueueClear(cRosMessageQueue *q);

/*! \brief Deletes the content of a queue and free all its allocated memory.
 *
 *  This function frees the memory allocated for its content so cRosMessageQueueInit() must be called before
 *  using the queue again.
 *  \param q Pointer to the queue.
 */
void cRosMessageQueueRelease(cRosMessageQueue *q);

/*! \brief Calculates the free space still available in the queue.
 *
 *  \return Number of messages that can still be added to the queue without causing an overflow.
 */
unsigned int cRosMessageQueueVacancies(cRosMessageQueue *q);

/*! \brief Calculates the number of messages stored in the queue.
 *
 *  \return Number of messages that can be removed from the queue.
 */
unsigned int cRosMessageQueueUsage(cRosMessageQueue *q);

/*! \brief Add a new message at the end of the queue.
 *
 *  This function adds a new element (message) at the end of the queue. The fields of the message pointed by m will be copied
 *  in internal memory of the queue, so the message pointed by m can be freed after being added.
 *  \param q Pointer to the queue.
 *  \param m Pointer to the message to be added.
 *  \return 0 on success, otherwise an error code: -1 = error allocating memory, -2 = No free space to add a new element.
 */
int cRosMessageQueueAdd(cRosMessageQueue *q, cRosMessage *m);

/*! \brief Extract the first message of the queue.
 *
 *  This function removes a element (message) at the start of the queue. The fields of the message at the start of the queue will be copied
 *  to the message pointed by m, so the message pointed by m should be freed independently after being used.
 *  \param q Pointer to the queue.
 *  \param m Pointer to the message where the fields of the removed message are copied.
 *  \return 0 on success, otherwise an error code: -1 = error allocating memory, -2 = No messages in the queue. If an error occurs, the
 *          message is not removed from the queue.
 */
int cRosMessageQueueExtract(cRosMessageQueue *q, cRosMessage *m);

/*! \brief Get a copy of the first message from the queue.
 *
 *  This function obtains the element (message) at the start of the queue. The fields of the message at the start of the queue will be copied
 *  to the message pointed by m, so the message pointed by m should be freed independently after being used.
 *  The queue is not modified.
 *  \param q Pointer to the queue.
 *  \param m Pointer to the message where the fields of the removed message are copied.
 *  \return 0 on success, otherwise an error code: -1 = error allocating memory, -2 = No messages in the queue.
 */
int cRosMessageQueueGet(cRosMessageQueue *q, cRosMessage *m);

/*! \brief Delete the first message from the queue.
 *
 *  This function removes a element (message) at the start of the queue.
 *  \param q Pointer to the queue.
 *  \return 0 on success, otherwise an error code: -2 = No messages in the queue.
 */
int cRosMessageQueueRemove(cRosMessageQueue *q);

/*! \brief Get a reference of the first message from the queue.
 *
 *  This function obtains a pointer to the element (message) at the start of the queue (oldest element). The fields of the message at the
 *  start of the queue will not be copied, so if the message queue is modified after obtaining this pointer, the pointer become invalid.
 *  The queue is not modified.
 *  \param q Pointer to the queue.
 *  \return Pointer to the first message. If the first message could not be obtained, NULL is returned.
 */
cRosMessage *cRosMessageQueuePeekFirst(cRosMessageQueue *q);

/*! \brief Get a reference of the last message from the queue.
 *
 *  This function obtains a pointer to the element (message) at the end of the queue (newest element). The fields of this message
 *  of the queue will not be copied, so if the message queue is modified after obtaining this pointer, the pointer become invalid.
 *  The queue is not modified by this function.
 *  \param q Pointer to the queue.
 *  \return Pointer to the last message. If the last message could not be obtained, NULL is returned.
 */
cRosMessage *cRosMessageQueuePeekLast(cRosMessageQueue *q);

#endif // _CROS_MESSAGE_QUEUE_H_
