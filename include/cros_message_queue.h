/*! \file cros_message_queue.h
 *  \brief This header file declares the cRosMessageQueue type and associated management functions
 *
 *  cRosMessageQueue implements a queue of TCPROS-protocol messages. This queue behaves as a
 *  FIFO (First In First Out).
 *  This queue is only stores the fields of the messages (fields fields of the struct), not the message definition (msgDef field).
 *  \author Richard R. Carrillo (of the Service-client implementation). Aging in Vision and Action lab, Institut de la Vision, Sorbonne University, Paris, France.
 *  \date 31 Oct 2017
 */

#ifndef _CROS_MESSAGE_QUEUE_H_
#define _CROS_MESSAGE_QUEUE_H_

#include "cros_message.h"


#define MAX_QUEUE_LEN 3 //! Maximum number of messages that can be hold in the queue

typedef struct cRosMessageQueue cRosMessageQueue;

struct cRosMessageQueue
{
  cRosMessage msgs[MAX_QUEUE_LEN]; //! Content of the queue
  unsigned int length; //! Number of messages currently in the queue
  unsigned int first_msg_ind; //! Index of the oldest message in the queue (the one that was inserted first)
};

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

/*! \brief Add a new message to the queue.
 *
 *  This function adds a new element (message) at the end of the queue. The fields of the message pointed by m will be copied
 *  in internal memory of the queue, so the message pointed by m can be freed after being added.
 *  \param q Pointer to the queue.
 *  \param m Pointer to the message to be added.
 *  \return 0 on success, otherwise an error code: -1 = error allocating memory, -2 = No free space to add a new element.
 */
int cRosMessageQueueAdd(cRosMessageQueue *q, cRosMessage *m);

/*! \brief Remove a message from the queue.
 *
 *  This function removes a element (message) at the start of the queue. The fields of the message at the start of the queue will be copied
 *  to the message pointed by m, so the message pointed by m can be freed independently after being used.
 *  \param q Pointer to the queue.
 *  \param m Pointer to the message where the fields of the removed message are copied.
 *  \return 0 on success, otherwise an error code: -1 = error allocating memory, -2 = No messages in the queue. If an error occurs, the
 *          message is not removed from the queue.
 */
int cRosMessageQueueRemove(cRosMessageQueue *q, cRosMessage *m);

#endif // _CROS_MESSAGE_QUEUE_H_
