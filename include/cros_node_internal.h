#ifndef _CROS_NODE_INTERNAL_H_
#define _CROS_NODE_INTERNAL_H_

#include "cros_node.h"

void restartAdversing(CrosNode* node);
int enqueueRequestTopic(CrosNode *node, int subidx);

#endif // _CROS_NODE_INTERNAL_H_
