#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <ctype.h>

#include "cros_api.h"
#include "cros_tcpros.h"
#include "cros_defs.h"
#include "tcpros_tags.h"
#include "tcpros_process.h"
#include "dyn_buffer.h"
#include "cros_log.h"

static uint32_t getLen( DynBuffer *pkt )
{
  uint32_t len;
  const unsigned char *data = dynBufferGetCurrentData(pkt);
  len = ROS_TO_HOST_UINT32(*(uint32_t *)data);
  dynBufferMovePoseIndicator(pkt,sizeof(uint32_t));

  return len;
}

static uint32_t pushBackField( DynBuffer *pkt, TcprosTagStrDim *tag, const char *val )
{
  size_t val_len = strlen( val );
  uint32_t out_len, field_len = tag->dim + val_len;
  //PRINT_VDEBUG("pushBackField() : filed : %s field_len ; %d\n", tag->str, field_len);
  out_len = HOST_TO_ROS_UINT32( field_len );
  dynBufferPushBackUInt32( pkt, out_len );
  dynBufferPushBackBuf( pkt, (const unsigned char*)tag->str, tag->dim );
  dynBufferPushBackBuf( pkt, (const unsigned char*)val, val_len );

  return field_len + sizeof( uint32_t );
}

static void printPacket( DynBuffer *pkt, int print_data )
{
  /* Save position indicator: it will be restored */
  int initial_pos_idx = dynBufferGetPoseIndicatorOffset ( pkt );
  dynBufferRewindPoseIndicator ( pkt );

  uint32_t bytes_to_read = getLen( pkt );

  fprintf(cRosOutStreamGet(),"Header len %d\n",bytes_to_read);
  while ( bytes_to_read > 0)
  {
    uint32_t field_len = getLen( pkt );
    const char *field = (const char *)dynBufferGetCurrentData( pkt );
    if( field_len )
    {
      fwrite ( field, 1, field_len, stdout );
      fprintf(cRosOutStreamGet(),"\n" );
      dynBufferMovePoseIndicator( pkt, field_len );
    }
    bytes_to_read -= ( sizeof(uint32_t) + field_len );
  }

  if( print_data )
  {
    bytes_to_read = getLen( pkt );

    fprintf(cRosOutStreamGet(),"Data len %d\n",bytes_to_read);
    while ( bytes_to_read > 0)
    {
      uint32_t field_len = getLen( pkt );
      const char *field = (const char *)dynBufferGetCurrentData( pkt );
      if( field_len )
      {
        fwrite ( field, 1, field_len, stdout );
        fprintf(cRosOutStreamGet(),"\n");
        dynBufferMovePoseIndicator( pkt, field_len );
      }
      bytes_to_read -= ( sizeof(uint32_t) + field_len );
    }
  }

  /* Restore position indicator */
  dynBufferSetPoseIndicator ( pkt, initial_pos_idx );
}

static TcprosParserState readSubcriptionHeader( TcprosProcess *p, uint32_t *flags )
{
  PRINT_VVDEBUG("readSubcriptioHeader()\n");
  DynBuffer *packet = &(p->packet);
  uint32_t bytes_to_read = getLen( packet );
  size_t packet_len = dynBufferGetSize( packet );

  if( bytes_to_read > packet_len - sizeof( uint32_t ) )
    return TCPROS_PARSER_HEADER_INCOMPLETE;

  *flags = 0x0;

  PRINT_VDEBUG("readSubcriptioHeader() : Header len=%d\n",bytes_to_read);

  while ( bytes_to_read > 0)
  {
    uint32_t field_len = getLen( packet );

    PRINT_VDEBUG("readSubcriptioHeader() : Field len=%d\n",field_len);

    const char *field = (const char *)dynBufferGetCurrentData( packet );
    if( field_len )
    {
      if ( field_len > (uint32_t)TCPROS_CALLERID_TAG.dim &&
          strncmp ( field, TCPROS_CALLERID_TAG.str, TCPROS_CALLERID_TAG.dim ) == 0 )
      {
        field += TCPROS_CALLERID_TAG.dim;

        dynStringReplaceWithStrN( &(p->caller_id), field,
                               field_len - TCPROS_CALLERID_TAG.dim );
        *flags |= TCPROS_CALLER_ID_FLAG;
        dynBufferMovePoseIndicator( packet, field_len );
      }
      else if ( field_len > (uint32_t)TCPROS_TOPIC_TAG.dim &&
          strncmp ( field, TCPROS_TOPIC_TAG.str, TCPROS_TOPIC_TAG.dim ) == 0 )
      {
        field += TCPROS_TOPIC_TAG.dim;

        dynStringReplaceWithStrN( &(p->topic), field,
                               field_len - TCPROS_TOPIC_TAG.dim );
        *flags |= TCPROS_TOPIC_FLAG;
        dynBufferMovePoseIndicator( packet, field_len );
      }
      else if ( field_len > (uint32_t)TCPROS_TYPE_TAG.dim &&
          strncmp ( field, TCPROS_TYPE_TAG.str, TCPROS_TYPE_TAG.dim ) == 0 )
      {
        field += TCPROS_TYPE_TAG.dim;

        dynStringReplaceWithStrN( &(p->type), field,
                               field_len - TCPROS_TYPE_TAG.dim );
        *flags |= TCPROS_TYPE_FLAG;
        dynBufferMovePoseIndicator( packet, field_len );
      }
      else if ( field_len > (uint32_t)TCPROS_MD5SUM_TAG.dim &&
          strncmp ( field, TCPROS_MD5SUM_TAG.str, TCPROS_MD5SUM_TAG.dim ) == 0 )
      {
        field += TCPROS_MD5SUM_TAG.dim;

        dynStringReplaceWithStrN( &(p->md5sum), field,
                               field_len - TCPROS_MD5SUM_TAG.dim );
        *flags |= TCPROS_MD5SUM_FLAG;
        dynBufferMovePoseIndicator( packet, field_len );
      }
      else if ( field_len > (uint32_t)TCPROS_MESSAGE_DEFINITION_TAG.dim &&
          strncmp ( field, TCPROS_MESSAGE_DEFINITION_TAG.str, TCPROS_MESSAGE_DEFINITION_TAG.dim ) == 0 )
      {
        *flags |= TCPROS_MESSAGE_DEFINITION_FLAG;
        dynBufferMovePoseIndicator( packet, field_len );
      }
      else if ( field_len > (uint32_t)TCPROS_TCP_NODELAY_TAG.dim &&
          strncmp ( field, TCPROS_TCP_NODELAY_TAG.str, TCPROS_TCP_NODELAY_TAG.dim ) == 0 )
      {
        field += TCPROS_TCP_NODELAY_TAG.dim;
        p->tcp_nodelay = (*field == '1')?1:0;
        *flags |= TCPROS_TCP_NODELAY_FLAG;
        dynBufferMovePoseIndicator( packet, field_len );
      }
      else if ( field_len > (uint32_t)TCPROS_LATCHING_TAG.dim &&
          strncmp ( field, TCPROS_LATCHING_TAG.str, TCPROS_LATCHING_TAG.dim ) == 0 )
      {
        PRINT_INFO("readSubcriptioHeader() WARNING : TCPROS_LATCHING_TAG not implemented\n");
        field += TCPROS_LATCHING_TAG.dim;
        p->latching = (*field == '1')?1:0;
        *flags |= TCPROS_LATCHING_FLAG;
        dynBufferMovePoseIndicator( packet, field_len );
      }
      else if ( field_len > (uint32_t)TCPROS_ERROR_TAG.dim &&
          strncmp ( field, TCPROS_ERROR_TAG.str, TCPROS_ERROR_TAG.dim ) == 0 )
      {
        PRINT_INFO("readSubcriptioHeader() WARNING : TCPROS_ERROR_TAG not implemented\n");
        *flags |= TCPROS_ERROR_FLAG;
        dynBufferMovePoseIndicator( packet, field_len );
      }
      else
      {
        PRINT_ERROR("readSubcriptioHeader() : unknown field\n");
        *flags = 0x0;
        break;
      }
    }

    bytes_to_read -= ( sizeof(uint32_t) + field_len );
  }

  return TCPROS_PARSER_DONE;

}

static TcprosParserState readPublicationHeader( TcprosProcess *p, uint32_t *flags )
{
  PRINT_VVDEBUG("readPublicationHeader()\n");
  DynBuffer *packet = &(p->packet);
  size_t bytes_to_read = dynBufferGetSize( packet );

  *flags = 0x0;

  PRINT_VDEBUG("readPublicationHeader() : Header len=%lu\n", (long unsigned)bytes_to_read);

  while ( bytes_to_read > 0)
  {
    uint32_t field_len = getLen( packet );

    PRINT_VDEBUG("readPublicationHeader() : Field len=%d\n",field_len);

    const char *field = (const char *)dynBufferGetCurrentData( packet );
    if( field_len )
    {
      // http://wiki.ros.org/ROS/TCPROS doesn't mention it but it's sent anyway in ros groovy
      if ( field_len > (uint32_t)TCPROS_MESSAGE_DEFINITION_TAG.dim &&
          strncmp ( field, TCPROS_MESSAGE_DEFINITION_TAG.str, TCPROS_MESSAGE_DEFINITION_TAG.dim ) == 0 )
      {
        *flags |= TCPROS_MESSAGE_DEFINITION_FLAG;
        dynBufferMovePoseIndicator( packet, field_len );
      } else if ( field_len > (uint32_t)TCPROS_CALLERID_TAG.dim &&
          strncmp ( field, TCPROS_CALLERID_TAG.str, TCPROS_CALLERID_TAG.dim ) == 0 )
      {
        field += TCPROS_CALLERID_TAG.dim;

        dynStringReplaceWithStrN( &(p->caller_id), field,
                               field_len - TCPROS_CALLERID_TAG.dim );
        *flags |= TCPROS_CALLER_ID_FLAG;
        dynBufferMovePoseIndicator( packet, field_len );
      }
      else if ( field_len > (uint32_t)TCPROS_TYPE_TAG.dim &&
          strncmp ( field, TCPROS_TYPE_TAG.str, TCPROS_TYPE_TAG.dim ) == 0 )
      {
        field += TCPROS_TYPE_TAG.dim;

        dynStringReplaceWithStrN( &(p->type), field,
                               field_len - TCPROS_TYPE_TAG.dim );
        *flags |= TCPROS_TYPE_FLAG;
        dynBufferMovePoseIndicator( packet, field_len );
      }
      else if ( field_len > (uint32_t)TCPROS_MD5SUM_TAG.dim &&
          strncmp ( field, TCPROS_MD5SUM_TAG.str, TCPROS_MD5SUM_TAG.dim ) == 0 )
      {
        field += TCPROS_MD5SUM_TAG.dim;

        dynStringReplaceWithStrN( &(p->md5sum), field,
                               field_len - TCPROS_MD5SUM_TAG.dim );
        *flags |= TCPROS_MD5SUM_FLAG;
        dynBufferMovePoseIndicator( packet, field_len );
      }
      else if ( field_len > (uint32_t)TCPROS_LATCHING_TAG.dim &&
          strncmp ( field, TCPROS_LATCHING_TAG.str, TCPROS_LATCHING_TAG.dim ) == 0 )
      {
        PRINT_INFO("readPublicationHeader() WARNING : TCPROS_LATCHING_TAG not implemented\n");
        field += TCPROS_LATCHING_TAG.dim;
        p->latching = (*field == '1')?1:0;
        *flags |= TCPROS_LATCHING_FLAG;
        dynBufferMovePoseIndicator( packet, field_len );
      }
      else if ( field_len > (uint32_t)TCPROS_TOPIC_TAG.dim &&
          strncmp ( field, TCPROS_TOPIC_TAG.str, TCPROS_TOPIC_TAG.dim ) == 0 )
      {
        // http://wiki.ros.org/ROS/TCPROS doesn't mention it but it's sent anyway in ros groovy
        field += TCPROS_TOPIC_TAG.dim;

        dynStringReplaceWithStrN( &(p->topic), field,
                               field_len - TCPROS_TOPIC_TAG.dim );
        *flags |= TCPROS_TOPIC_FLAG;
        dynBufferMovePoseIndicator( packet, field_len );
      }
      else if ( field_len > (uint32_t)TCPROS_ERROR_TAG.dim &&
          strncmp ( field, TCPROS_ERROR_TAG.str, TCPROS_ERROR_TAG.dim ) == 0 )
      {
        PRINT_INFO("readPublicationHeader() WARNING : TCPROS_ERROR_TAG not implemented\n");
        *flags |= TCPROS_ERROR_FLAG;
        dynBufferMovePoseIndicator( packet, field_len );
      }
      else if ( field_len > (uint32_t)TCPROS_TCP_NODELAY_TAG.dim &&
          strncmp ( field, TCPROS_TCP_NODELAY_TAG.str, TCPROS_TCP_NODELAY_TAG.dim ) == 0 )
      {
        field += TCPROS_TCP_NODELAY_TAG.dim;
        p->tcp_nodelay = (*field == '1')?1:0;
        *flags |= TCPROS_TCP_NODELAY_FLAG;
        dynBufferMovePoseIndicator( packet, field_len );
      }
      else
      {
        PRINT_ERROR("readPublicationHeader() : unknown field\n");
        *flags = 0x0;
        break;
      }
    }

    bytes_to_read -= ( sizeof(uint32_t) + field_len );
  }

  return TCPROS_PARSER_DONE;
}

TcprosParserState cRosMessageParseSubcriptionHeader( CrosNode *n, int server_idx )
{
  PRINT_VVDEBUG("cRosMessageParseSubcriptionHeader()\n");

  TcprosProcess *server_proc = &(n->tcpros_server_proc[server_idx]);
  DynBuffer *packet = &(server_proc->packet);

  // Save position indicator: it will be restored
  int initial_pos_idx = dynBufferGetPoseIndicatorOffset ( packet );
  dynBufferRewindPoseIndicator ( packet );

  uint32_t header_flags;
  TcprosParserState ret = readSubcriptionHeader( server_proc, &header_flags );
  if( ret != TCPROS_PARSER_DONE )
    return ret;

  if( TCPROS_SUBCRIPTION_HEADER_FLAGS != ( header_flags&TCPROS_SUBCRIPTION_HEADER_FLAGS) )
  {
    PRINT_ERROR("cRosMessageParseSubcriptionHeader() : There are missing fields in the received header.\n");
    ret = TCPROS_PARSER_ERROR;
  }
  else
  {
    int topic_found = 0;
    int i = 0;
    for( i = 0 ; i < n->n_pubs; i++)
    {
      PublisherNode *pub = &n->pubs[i];
      if (pub->topic_name == NULL)
        continue;

      //printf("cRosMessageParseSubcriptionHeader() : checking if the topic in received header is what we expect: Topic name: %s Topic type: %s MD5: %s.\n", pub->topic_name, pub->topic_type, pub->md5sum);

      if( strcmp(pub->topic_name, dynStringGetData(&(server_proc->topic))) == 0 &&
          (strcmp(pub->topic_type, dynStringGetData(&(server_proc->type))) == 0 || strcmp(dynStringGetData(&(server_proc->type)), "*") == 0) &&
          (strcmp(pub->md5sum, dynStringGetData(&(server_proc->md5sum))) == 0 || strcmp(dynStringGetData(&(server_proc->md5sum)), "*") == 0))
      {
        int list_elem;

        topic_found = 1;
        server_proc->topic_idx = i; // Assign a topic (publisher index) to the TCPROS process
        // Add the TcprosProcess index to the Publisher
        for(list_elem=0;pub->tcpros_id_list[list_elem]!=-1;list_elem++); // Locate the list end
        pub->tcpros_id_list[list_elem] = server_idx;
        pub->tcpros_id_list[list_elem+1] = -1; // Set a new list end (sentinel)
        break;
      }
    }

    if( ! topic_found )
    {
      PRINT_ERROR("cRosMessageParseSubcriptionHeader() : Wrong service, type or md5sum in the received header. "
      "Received: Topic name: %s Topic type: %s MD5: %s.\n", \
      dynStringGetData(&(server_proc->topic)), dynStringGetData(&(server_proc->type)), dynStringGetData(&(server_proc->md5sum)));
      server_proc->topic_idx = -1;
      ret = TCPROS_PARSER_ERROR;
    }
    else
    {
      if(server_proc->tcp_nodelay)
        tcpIpSocketSetNoDelay(&server_proc->socket);
    }
  }

  // Restore position indicator
  dynBufferSetPoseIndicator ( packet, initial_pos_idx );

  return ret;
}

TcprosParserState cRosMessageParsePublicationHeader( CrosNode *n, int client_idx )
{
  PRINT_VVDEBUG("cRosMessageParsePublicationHeader()\n");

  TcprosProcess *client_proc = &(n->tcpros_client_proc[client_idx]);
  DynBuffer *packet = &(client_proc->packet);

  /* Save position indicator: it will be restored */
  int initial_pos_idx = dynBufferGetPoseIndicatorOffset ( packet );
  dynBufferRewindPoseIndicator ( packet );

  uint32_t header_flags;
  TcprosParserState ret = readPublicationHeader( client_proc, &header_flags );
  if( ret != TCPROS_PARSER_DONE )
    return ret;

  if( TCPROS_PUBLICATION_HEADER_FLAGS != ( header_flags&TCPROS_PUBLICATION_HEADER_FLAGS) )
  {
    PRINT_ERROR("cRosMessageParsePublicationHeader() : Missing fields\n");
    ret = TCPROS_PARSER_ERROR;
  }
  else
  {
    int subscriber_found = 0;
    int i = 0;
    for( i = 0 ; i < n->n_subs; i++)
    {
      SubscriberNode *sub = &n->subs[i];
      if (sub->topic_name == NULL)
        continue;

      if( strcmp(sub->topic_type, dynStringGetData(&(client_proc->type))) == 0 &&
          strcmp(sub->md5sum, dynStringGetData(&(client_proc->md5sum))) == 0)
      {
        subscriber_found = 1;
        break;
      }
    }

    if( ! subscriber_found )
    {
      PRINT_ERROR("cRosMessageParsePublicationHeader() : Wrong topic, type or md5sum\n");
      ret = TCPROS_PARSER_ERROR;
    }
    else
    {
      if(client_proc->tcp_nodelay)
        tcpIpSocketSetNoDelay(&client_proc->socket); // Not necessary really because subscribers do not write massage packets (only read)
    }
  }

  /* Restore position indicator */
  dynBufferSetPoseIndicator ( packet, initial_pos_idx );

  return ret;
}

void cRosMessagePrepareSubcriptionHeader( CrosNode *n, int client_idx )
{
  PRINT_VVDEBUG("cRosMessagePrepareSubcriptionHeader()\n");

  TcprosProcess *client_proc = &(n->tcpros_client_proc[client_idx]);
  int sub_idx = client_proc->topic_idx;
  DynBuffer *packet = &(client_proc->packet);
  uint32_t header_len = 0, header_out_len = 0;
  dynBufferPushBackUInt32( packet, header_out_len );

  header_len += pushBackField( packet, &TCPROS_MESSAGE_DEFINITION_TAG, n->subs[sub_idx].message_definition );
  header_len += pushBackField( packet, &TCPROS_CALLERID_TAG, n->name );
  header_len += pushBackField( packet, &TCPROS_TOPIC_TAG, n->subs[sub_idx].topic_name );
  header_len += pushBackField( packet, &TCPROS_MD5SUM_TAG, n->subs[sub_idx].md5sum );
  header_len += pushBackField( packet, &TCPROS_TYPE_TAG, n->subs[sub_idx].topic_type );
  if(n->subs[sub_idx].tcp_nodelay)
    header_len += pushBackField( packet, &TCPROS_TCP_NODELAY_TAG, "1" );

  header_out_len= HOST_TO_ROS_UINT32( header_len );
  uint32_t *header_len_p = (uint32_t *)dynBufferGetData( packet );
  *header_len_p = header_out_len;
}

cRosErrCodePack cRosMessageParsePublicationPacket( CrosNode *n, int client_idx )
{
  cRosErrCodePack ret_err;
  SubscriberNode *sub_node;
  TcprosProcess *client_proc;
  DynBuffer *packet;
  void *data_context;

  client_proc = &(n->tcpros_client_proc[client_idx]);
  packet = &(client_proc->packet);
  sub_node = &n->subs[client_proc->topic_idx];
  data_context = sub_node->context;

  if(cRosMessageQueueVacancies(&sub_node->msg_queue) == 0)
    sub_node->msg_queue_overflow = 1; // No space in the queue for the new message

  ret_err = cRosNodeDeserializeIncomingPacket(packet, data_context);
  if(ret_err == CROS_SUCCESS_ERR_PACK)
    ret_err = cRosNodeSubscriberCallback(data_context); // Calls the subscriber application-defined callback
  else
    cRosPrintErrCodePack(ret_err, "cRosNodeSubscriberCallback() failed decoding the received packet");

  return ret_err;
}

void cRosMessagePreparePublicationHeader( CrosNode *n, int server_idx )
{
  PRINT_VVDEBUG("cRosMessagePreparePublicationHeader()\n");

  TcprosProcess *server_proc = &(n->tcpros_server_proc[server_idx]);
  int pub_idx = server_proc->topic_idx;
  DynBuffer *packet = &(server_proc->packet);
  uint32_t header_len = 0, header_out_len = 0;
  dynBufferPushBackUInt32( packet, header_out_len );

  // http://wiki.ros.org/ROS/TCPROS doesn't mention to send message_definition and topic_name
  // but they are sent anyway in ros groovy
  header_len += pushBackField( packet, &TCPROS_MESSAGE_DEFINITION_TAG, n->pubs[pub_idx].message_definition );
  header_len += pushBackField( packet, &TCPROS_CALLERID_TAG, n->name );
  header_len += pushBackField( packet, &TCPROS_LATCHING_TAG, "1" );
  header_len += pushBackField( packet, &TCPROS_MD5SUM_TAG, n->pubs[pub_idx].md5sum );
  header_len += pushBackField( packet, &TCPROS_TOPIC_TAG, n->pubs[pub_idx].topic_name );
  header_len += pushBackField( packet, &TCPROS_TYPE_TAG, n->pubs[pub_idx].topic_type );
  header_len += pushBackField( packet, &TCPROS_TCP_NODELAY_TAG, (server_proc->tcp_nodelay)?"1":"0" );

  header_out_len = HOST_TO_ROS_UINT32( header_len );
  uint32_t *header_len_p = (uint32_t *)dynBufferGetData( packet );
  *header_len_p = header_out_len;
}

cRosErrCodePack cRosMessagePreparePublicationPacket( CrosNode *node, int server_idx )
{
  cRosErrCodePack ret_err;
  PublisherNode *pub_node;
  TcprosProcess *server_proc;
  int pub_idx;
  DynBuffer *packet;
  uint32_t *packet_data_size_ptr;
  uint32_t packet_size;
  PRINT_VVDEBUG("cRosMessagePreparePublicationPacket()\n");

  server_proc = &(node->tcpros_server_proc[server_idx]);
  pub_idx = server_proc->topic_idx;
  packet = &(server_proc->packet);
  dynBufferPushBackUInt32( packet, 0 ); // Placeholder for packet size

  pub_node = &node->pubs[pub_idx];

  ret_err = cRosNodeSerializeOutgoingMessage(packet, pub_node->context);

  packet_size = (uint32_t)dynBufferGetSize(packet) - sizeof(uint32_t);
  packet_data_size_ptr = (uint32_t *)dynBufferGetData(packet);
  *packet_data_size_ptr = packet_size;

  return ret_err;
}

static TcprosParserState readServiceCallHeader( TcprosProcess *p, uint32_t *flags )
{
  PRINT_VVDEBUG("readServiceCallHeader()\n");
  DynBuffer *packet = &(p->packet);
  size_t bytes_to_read = dynBufferGetSize( packet );

  *flags = 0x0;

  PRINT_VDEBUG("readServiceCallHeader() : Header len=%lu\n", (long unsigned)bytes_to_read);

  while ( bytes_to_read > 0)
  {
    uint32_t field_len = getLen( packet );

    PRINT_VDEBUG("readServiceCallHeader() : Field len=%d\n",field_len);

    const char *field = (const char *)dynBufferGetCurrentData( packet );

    if( field_len )
    {
      if ( field_len > (uint32_t)TCPROS_CALLERID_TAG.dim &&
          strncmp ( field, TCPROS_CALLERID_TAG.str, TCPROS_CALLERID_TAG.dim ) == 0 )
      {
         field += TCPROS_CALLERID_TAG.dim;

        dynStringReplaceWithStrN( &(p->caller_id), field,
                               field_len - TCPROS_CALLERID_TAG.dim );
        *flags |= TCPROS_CALLER_ID_FLAG;
        dynBufferMovePoseIndicator( packet, field_len );
      }
      else if ( field_len > (uint32_t)TCPROS_TYPE_TAG.dim &&
          strncmp ( field, TCPROS_TYPE_TAG.str, TCPROS_TYPE_TAG.dim ) == 0 )
      {
        field += TCPROS_TYPE_TAG.dim;

        dynStringReplaceWithStrN( &(p->type), field,
                               field_len - TCPROS_TYPE_TAG.dim );
        *flags |= TCPROS_TYPE_FLAG;
        dynBufferMovePoseIndicator( packet, field_len );
      }
      else if ( field_len >= (uint32_t)TCPROS_EMPTY_MD5SUM_TAG.dim &&
          strncmp ( field, TCPROS_EMPTY_MD5SUM_TAG.str, TCPROS_EMPTY_MD5SUM_TAG.dim ) == 0 )
      {
        *flags |= TCPROS_EMPTY_MD5SUM_FLAG;
        dynBufferMovePoseIndicator( packet, field_len );
      }
      else if ( field_len > (uint32_t)TCPROS_MD5SUM_TAG.dim &&
          strncmp ( field, TCPROS_MD5SUM_TAG.str, TCPROS_MD5SUM_TAG.dim ) == 0 )
      {
        field += TCPROS_MD5SUM_TAG.dim;

        dynStringReplaceWithStrN( &(p->md5sum), field,
                               field_len - TCPROS_MD5SUM_TAG.dim );
        *flags |= TCPROS_MD5SUM_FLAG;
        dynBufferMovePoseIndicator( packet, field_len );
      }
      else if ( field_len > (uint32_t)TCPROS_SERVICE_TAG.dim &&
          strncmp ( field, TCPROS_SERVICE_TAG.str, TCPROS_SERVICE_TAG.dim ) == 0 )
      {
        field += TCPROS_SERVICE_TAG.dim;

        dynStringReplaceWithStrN( &(p->service), field,
                               field_len - TCPROS_SERVICE_TAG.dim );
        *flags |= TCPROS_SERVICE_FLAG;
        dynBufferMovePoseIndicator( packet, field_len );
      }
      else if ( field_len > (uint32_t)TCPROS_PERSISTENT_TAG.dim &&
          strncmp ( field, TCPROS_PERSISTENT_TAG.str, TCPROS_PERSISTENT_TAG.dim ) == 0 )
      {
        //PRINT_INFO("readPublicationHeader() WARNING : TCPROS_PERSISTENT_TAG not implemented\n");
        field += TCPROS_PERSISTENT_TAG.dim;
        p->persistent = (*field == '1')?1:0;
        *flags |= TCPROS_PERSISTENT_FLAG;
        dynBufferMovePoseIndicator( packet, field_len );
      }
      else if ( field_len > (uint32_t)TCPROS_PROBE_TAG.dim &&
          strncmp ( field, TCPROS_PROBE_TAG.str, TCPROS_PROBE_TAG.dim ) == 0 )
      {
        //PRINT_INFO("readServiceCallHeader() WARNING : TCPROS_PROBE_TAG implemented only\n");
        field += TCPROS_PROBE_TAG.dim;
        p->probe = (*field == '1')?1:0;
        *flags |= TCPROS_PROBE_FLAG;
        dynBufferMovePoseIndicator( packet, field_len );
      }
      else if ( field_len > (uint32_t)TCPROS_ERROR_TAG.dim &&
          strncmp ( field, TCPROS_ERROR_TAG.str, TCPROS_ERROR_TAG.dim ) == 0 )
      {
        PRINT_INFO("readServiceCallHeader() WARNING : TCPROS_ERROR_TAG not implemented\n");
        *flags |= TCPROS_ERROR_FLAG;
        dynBufferMovePoseIndicator( packet, field_len );
      }
      else if ( field_len > (uint32_t)TCPROS_MESSAGE_DEFINITION_TAG.dim &&
          strncmp ( field, TCPROS_MESSAGE_DEFINITION_TAG.str, TCPROS_MESSAGE_DEFINITION_TAG.dim ) == 0 )
      {
        PRINT_INFO("readServiceCallHeader() WARNING : TCPROS_MESSAGE_DEFINITION_TAG not implemented\n");
        *flags |= TCPROS_MESSAGE_DEFINITION_FLAG;
        dynBufferMovePoseIndicator( packet, field_len );
      }
      else if ( field_len > (uint32_t)TCPROS_TCP_NODELAY_TAG.dim &&
          strncmp ( field, TCPROS_TCP_NODELAY_TAG.str, TCPROS_TCP_NODELAY_TAG.dim ) == 0 )
      {
        field += TCPROS_TCP_NODELAY_TAG.dim;
        p->tcp_nodelay = (*field == '1')?1:0;
        *flags |= TCPROS_TCP_NODELAY_FLAG;
        dynBufferMovePoseIndicator( packet, field_len );
      }
      else
      {
        PRINT_ERROR("readServiceCallHeader() : unknown field\n");
        *flags = 0x0;
        break;
      }
    }
    bytes_to_read -= ( sizeof(uint32_t) + field_len );
  }

  return TCPROS_PARSER_DONE;
}

TcprosParserState cRosMessageParseServiceCallerHeader( CrosNode *n, int server_idx)
{
  PRINT_VVDEBUG("cRosMessageParseServiceCallerHeader()\n");

  TcprosProcess *server_proc = &(n->rpcros_server_proc[server_idx]);
  DynBuffer *packet = &(server_proc->packet);

  /* Save position indicator: it will be restored */
  int initial_pos_idx = dynBufferGetPoseIndicatorOffset ( packet );
  dynBufferRewindPoseIndicator ( packet );

  uint32_t header_flags;
  TcprosParserState ret = readServiceCallHeader( server_proc, &header_flags );
  if( ret != TCPROS_PARSER_DONE )
    return ret;

  int service_found = 0;

  if( header_flags == ( header_flags & TCPROS_SERVICECALL_HEADER_FLAGS) || header_flags == ( header_flags & TCPROS_SERVICECALL_MATLAB_HEADER_FLAGS) )
  {
    int svc_name_match = 0;
    int i = 0;
    for( i = 0 ; i < n->n_service_providers; i++)
    {
      if( strcmp( n->service_providers[i].service_name, dynStringGetData(&(server_proc->service))) == 0)
      {
        svc_name_match = 1;
        if(strcmp( n->service_providers[i].md5sum, dynStringGetData(&(server_proc->md5sum))) == 0)
        {
          service_found = 1;
          server_proc->service_idx = i;
          break;
        }
      }
    }
    if( ! service_found )
    {
      if(svc_name_match == 0)
        PRINT_ERROR("cRosMessageParseServiceCallerHeader() : Received a service call header specifying a unknown service name\n");
      else
        PRINT_ERROR("cRosMessageParseServiceCallerHeader() : Received a service call header specifying a known service name with a wrong MD5 sum\n");
    }
  }
  else if( header_flags == ( header_flags & TCPROS_SERVICEPROBE_HEADER_FLAGS) || header_flags == ( header_flags & TCPROS_SERVICEPROBE_MATLAB_HEADER_FLAGS) )
  {
    int i = 0;
    for( i = 0 ; i < n->n_service_providers; i++)
    {
      if( strcmp( n->service_providers[i].service_name, dynStringGetData(&(server_proc->service))) == 0)
      {
        service_found = 1;
        server_proc->service_idx = i;
        break;
      }
    }
    if( ! service_found )
      PRINT_ERROR("cRosMessageParseServiceCallerHeader() : Received a service probe header specifying a unknown service name\n");
  }
  else
  {
    PRINT_ERROR("cRosMessageParseServiceCallerHeader() : Received a service call header missing fields\n");
    ret = TCPROS_PARSER_ERROR;
  }

  if( ! service_found )
  {
    server_proc->service_idx = -1;
    ret = TCPROS_PARSER_ERROR;
  }
  else
  {
    if(server_proc->tcp_nodelay)
      tcpIpSocketSetNoDelay(&server_proc->socket);
  }

  /* Restore position indicator */
  dynBufferSetPoseIndicator ( packet, initial_pos_idx );

  return ret;
}

static TcprosParserState readServiceProvisionHeader( TcprosProcess *p, uint32_t *flags )
{
  PRINT_VVDEBUG("readServiceProvisionHeader()\n");
  DynBuffer *packet = &(p->packet);
  size_t bytes_to_read = dynBufferGetSize( packet );

  *flags = 0x0;

  PRINT_VDEBUG("readServiceProvisionHeader() : Header len=%lu\n", (long unsigned)bytes_to_read);

  while ( bytes_to_read > 0)
  {
    uint32_t field_len = getLen( packet );

    PRINT_VDEBUG("readServiceProvisionHeader() : Field len=%d\n",field_len);

    const char *field = (const char *)dynBufferGetCurrentData( packet );

    if( field_len )
    {
      if ( field_len > (uint32_t)TCPROS_CALLERID_TAG.dim &&
          strncmp ( field, TCPROS_CALLERID_TAG.str, TCPROS_CALLERID_TAG.dim ) == 0 )
      {
         field += TCPROS_CALLERID_TAG.dim;

        dynStringReplaceWithStrN( &(p->caller_id), field,
                               field_len - TCPROS_CALLERID_TAG.dim );
        *flags |= TCPROS_CALLER_ID_FLAG;
        dynBufferMovePoseIndicator( packet, field_len );
      }
      else if ( field_len > (uint32_t)TCPROS_TYPE_TAG.dim &&
          strncmp ( field, TCPROS_TYPE_TAG.str, TCPROS_TYPE_TAG.dim ) == 0 )
      {
        field += TCPROS_TYPE_TAG.dim;

        dynStringReplaceWithStrN( &(p->type), field,
                               field_len - TCPROS_TYPE_TAG.dim );
        *flags |= TCPROS_TYPE_FLAG;
        dynBufferMovePoseIndicator( packet, field_len );
      }
      else if ( field_len >= (uint32_t)TCPROS_EMPTY_MD5SUM_TAG.dim &&
          strncmp ( field, TCPROS_EMPTY_MD5SUM_TAG.str, TCPROS_EMPTY_MD5SUM_TAG.dim ) == 0 )
      {
        *flags |= TCPROS_EMPTY_MD5SUM_FLAG;
        dynBufferMovePoseIndicator( packet, field_len );
      }
      else if ( field_len > (uint32_t)TCPROS_MD5SUM_TAG.dim &&
          strncmp ( field, TCPROS_MD5SUM_TAG.str, TCPROS_MD5SUM_TAG.dim ) == 0 )
      {
        field += TCPROS_MD5SUM_TAG.dim;

        dynStringReplaceWithStrN( &(p->md5sum), field,
                               field_len - TCPROS_MD5SUM_TAG.dim );
        *flags |= TCPROS_MD5SUM_FLAG;
        dynBufferMovePoseIndicator( packet, field_len );
      }
      else if ( field_len > (uint32_t)TCPROS_SERVICE_TAG.dim &&
          strncmp ( field, TCPROS_SERVICE_TAG.str, TCPROS_SERVICE_TAG.dim ) == 0 )
      {
        field += TCPROS_SERVICE_TAG.dim;

        dynStringReplaceWithStrN( &(p->service), field, field_len - TCPROS_SERVICE_TAG.dim );
        *flags |= TCPROS_SERVICE_FLAG;
        dynBufferMovePoseIndicator( packet, field_len );
      }
      else if ( field_len > (uint32_t)TCPROS_SERVICE_REQUESTTYPE_TAG.dim &&
          strncmp ( field, TCPROS_SERVICE_REQUESTTYPE_TAG.str, TCPROS_SERVICE_REQUESTTYPE_TAG.dim ) == 0 )
      {
        field += TCPROS_SERVICE_REQUESTTYPE_TAG.dim;
        dynStringReplaceWithStrN( &(p->servicerequest_type), field, field_len - TCPROS_SERVICE_REQUESTTYPE_TAG.dim );
        *flags |= TCPROS_SERVICE_REQUESTTYPE_FLAG;
        dynBufferMovePoseIndicator( packet, field_len );
      }
      else if ( field_len > (uint32_t)TCPROS_SERVICE_RESPONSETYPE_TAG.dim &&
          strncmp ( field, TCPROS_SERVICE_RESPONSETYPE_TAG.str, TCPROS_SERVICE_RESPONSETYPE_TAG.dim ) == 0 )
      {
        field += TCPROS_SERVICE_RESPONSETYPE_TAG.dim;
        dynStringReplaceWithStrN( &(p->serviceresponse_type), field, field_len - TCPROS_SERVICE_RESPONSETYPE_TAG.dim );
        *flags |= TCPROS_SERVICE_RESPONSETYPE_FLAG;
        dynBufferMovePoseIndicator( packet, field_len );
      }
      else if ( field_len > (uint32_t)TCPROS_ERROR_TAG.dim &&
          strncmp ( field, TCPROS_ERROR_TAG.str, TCPROS_ERROR_TAG.dim ) == 0 )
      {
        PRINT_INFO("readServiceProvisionHeader() WARNING : TCPROS_ERROR_TAG not implemented\n");
        *flags |= TCPROS_ERROR_FLAG;
        dynBufferMovePoseIndicator( packet, field_len );
      }
      else if ( field_len > (uint32_t)TCPROS_MESSAGE_DEFINITION_TAG.dim &&
          strncmp ( field, TCPROS_MESSAGE_DEFINITION_TAG.str, TCPROS_MESSAGE_DEFINITION_TAG.dim ) == 0 )
      {
        PRINT_INFO("readServiceProvisionHeader() WARNING : TCPROS_MESSAGE_DEFINITION_TAG not implemented\n");
        *flags |= TCPROS_MESSAGE_DEFINITION_FLAG;
        dynBufferMovePoseIndicator( packet, field_len );
      }
      else if ( field_len > (uint32_t)TCPROS_TCP_NODELAY_TAG.dim &&
          strncmp ( field, TCPROS_TCP_NODELAY_TAG.str, TCPROS_TCP_NODELAY_TAG.dim ) == 0 )
      {
        field += TCPROS_TCP_NODELAY_TAG.dim;
        p->tcp_nodelay = (*field == '1')?1:0;
        *flags |= TCPROS_TCP_NODELAY_FLAG;
        dynBufferMovePoseIndicator( packet, field_len );
      }
      else
      {
        PRINT_ERROR("readServiceProvisionHeader() : unknown field\n");
        *flags = 0x0;
        break;
      }
    }
    bytes_to_read -= ( sizeof(uint32_t) + field_len );
  }

  return TCPROS_PARSER_DONE;
}

TcprosParserState cRosMessageParseServiceProviderHeader( CrosNode *n, int client_idx )
{
  PRINT_VVDEBUG("cRosMessageParseServiceProviderHeader()\n");

  TcprosProcess *client_proc = &(n->rpcros_client_proc[client_idx]);
  DynBuffer *packet = &(client_proc->packet);

  /* Save position indicator: it will be restored */
  int initial_pos_idx = dynBufferGetPoseIndicatorOffset ( packet );
  dynBufferRewindPoseIndicator ( packet );

  uint32_t header_flags;
  TcprosParserState ret = readServiceProvisionHeader( client_proc, &header_flags );
  if( ret != TCPROS_PARSER_DONE )
    return ret;

  if( TCPROS_SERVICEPROVISION_HEADER_FLAGS != ( header_flags&TCPROS_SERVICEPROVISION_HEADER_FLAGS) )
  {
    PRINT_ERROR("cRosMessageParseServiceProviderHeader() : Missing fields\n");
    ret = TCPROS_PARSER_ERROR;
  }
  else
  {
    ServiceCallerNode *svc_caller = &n->service_callers[client_proc->service_idx];

    if (header_flags&TCPROS_SERVICE_FLAG && strcmp(svc_caller->service_name, dynStringGetData(&(client_proc->service))) != 0)
    {
      PRINT_ERROR("cRosMessageParseServiceProviderHeader() : Wrong service name from service provider\n");
      ret = TCPROS_PARSER_ERROR;
    }
    if(strcmp(svc_caller->md5sum, dynStringGetData(&(client_proc->md5sum))) != 0)
    {
      PRINT_ERROR("cRosMessageParseServiceProviderHeader() : Wrong MD5 sum from service provider\n");
      ret = TCPROS_PARSER_ERROR;
    }
    if(strcmp(svc_caller->service_type, dynStringGetData(&(client_proc->type))) != 0)
    {
      PRINT_ERROR("cRosMessageParseServiceProviderHeader() : Wrong service type from service provider\n");
      ret = TCPROS_PARSER_ERROR;
    }
    if(header_flags&TCPROS_SERVICE_REQUESTTYPE_FLAG && strcmp(svc_caller->servicerequest_type, dynStringGetData(&(client_proc->servicerequest_type))) != 0)
    {
      PRINT_ERROR("cRosMessageParseServiceProviderHeader() : Wrong service request type from service provider\n");
      ret = TCPROS_PARSER_ERROR;
    }
    if(header_flags&TCPROS_SERVICE_RESPONSETYPE_FLAG && strcmp(svc_caller->serviceresponse_type, dynStringGetData(&(client_proc->serviceresponse_type))) != 0)
    {
      PRINT_ERROR("cRosMessageParseServiceProviderHeader() : Wrong service response type from service provider\n");
      ret = TCPROS_PARSER_ERROR;
    }
    if(client_proc->tcp_nodelay)
      tcpIpSocketSetNoDelay(&client_proc->socket);
  }

  /* Restore position indicator */
  dynBufferSetPoseIndicator ( packet, initial_pos_idx );

  return ret;
}

void cRosMessagePrepareServiceCallHeader(CrosNode *n, int client_idx)
{
  PRINT_VVDEBUG("cRosMessagePrepareServiceCallHeader()\n");

  TcprosProcess *client_proc = &(n->rpcros_client_proc[client_idx]);
  int srv_idx = client_proc->service_idx;
  DynBuffer *packet = &(client_proc->packet);
  uint32_t header_len = 0, header_out_len = 0;
  dynBufferPushBackUInt32( packet, header_out_len );

  // Same format as MATLAB second header (not the probe one)
  header_len += pushBackField( packet, &TCPROS_SERVICE_TAG, n->service_callers[srv_idx].service_name );
  header_len += pushBackField( packet, &TCPROS_MESSAGE_DEFINITION_TAG, n->service_callers[srv_idx].message_definition );
  header_len += pushBackField( packet, &TCPROS_CALLERID_TAG, n->name );
  header_len += pushBackField( packet, &TCPROS_MD5SUM_TAG, n->service_callers[srv_idx].md5sum );
  if(client_proc->persistent)
    header_len += pushBackField( packet, &TCPROS_PERSISTENT_TAG, "1" );
  if(client_proc->tcp_nodelay)
    header_len += pushBackField( packet, &TCPROS_TCP_NODELAY_TAG, "1" );

 //header_len += pushBackField( packet, &TCPROS_SERVICE_REQUESTTYPE_TAG, n->service_callers[srv_idx].servicerequest_type );
 //header_len += pushBackField( packet, &TCPROS_SERVICE_RESPONSETYPE_TAG, n->service_callers[srv_idx].serviceresponse_type );
  header_len += pushBackField( packet, &TCPROS_TYPE_TAG, n->service_callers[srv_idx].service_type );

  header_out_len = HOST_TO_ROS_UINT32( header_len );
  uint32_t *header_len_p = (uint32_t *)dynBufferGetData( packet );
  *header_len_p = header_out_len;
}

cRosErrCodePack cRosMessagePrepareServiceCallPacket( CrosNode *n, int client_idx )
{
  cRosErrCodePack ret_err;

  PRINT_VVDEBUG("cRosMessagePrepareServiceCallPacket()\n");
  TcprosProcess *client_proc = &(n->rpcros_client_proc[client_idx]);
  int svc_idx = client_proc->service_idx;
  DynBuffer *packet = &(client_proc->packet);
  dynBufferPushBackUInt32( packet, 0 ); // Placehoder for packet size

  void* data_context = n->service_callers[svc_idx].context;
  ret_err = cRosNodeSerializeOutgoingMessage(packet, data_context); // Serialize the call outgoing message into the outgoing packet

  uint32_t data_size = (uint32_t)dynBufferGetSize(packet) - sizeof(uint32_t);
  uint32_t *packet_data_size_ptr = (uint32_t *)dynBufferGetData(packet);
  *packet_data_size_ptr = data_size;

  return ret_err;
}

cRosErrCodePack cRosMessageParseServiceResponsePacket( CrosNode *n, int client_idx )
{
  cRosErrCodePack ret_err;

  TcprosProcess *client_proc = &(n->rpcros_client_proc[client_idx]);
  DynBuffer *packet = &(client_proc->packet);
  if(client_proc->ok_byte == TCPROS_OK_BYTE_SUCCESS)
  {
    int svc_idx = client_proc->service_idx;
    void* data_context = n->service_callers[svc_idx].context;

    ret_err = cRosNodeDeserializeIncomingPacket(packet, data_context); // Deserialize the message response

    if(ret_err == CROS_SUCCESS_ERR_PACK)
      ret_err = cRosNodeServiceCallerCallback(1, data_context); // Call the service-caller application-defined callback function to process the service response
  }
  else
  {
     size_t n_char;
     PRINT_ERROR("cRosMessageParseServiceResponsePacket() : Error in service call response. 'ok' byte=%i. Error message='",client_proc->ok_byte);
     for(n_char=0;n_char<dynBufferGetSize(packet);n_char++)
     {
        dynBufferSetPoseIndicator(packet, n_char);
        PRINT_ERROR("%c",*dynBufferGetCurrentData(packet));
     }
     PRINT_ERROR("'\n");
     ret_err = CROS_SVC_RES_OK_BYTE_ERR;
  }
  return ret_err;
}

void cRosMessagePrepareServiceProviderHeader( CrosNode *n, int server_idx)
{
  PRINT_VVDEBUG("cRosMessagePreparePublicationHeader()\n");

  TcprosProcess *server_proc = &(n->rpcros_server_proc[server_idx]);
  int srv_idx = server_proc->service_idx;
  DynBuffer *packet = &(server_proc->packet);
  uint32_t header_len = 0, header_out_len = 0;
  dynBufferPushBackUInt32( packet, header_out_len );

  // http://wiki.ros.org/ROS/TCPROS doesn't mention to send message_definition and topic_name
  // but they are sent anyway in ros groovy
  header_len += pushBackField( packet, &TCPROS_CALLERID_TAG, n->name );
  header_len += pushBackField( packet, &TCPROS_MD5SUM_TAG, n->service_providers[srv_idx].md5sum );

  //if(server_proc->probe)
  //{
    header_len += pushBackField( packet, &TCPROS_SERVICE_REQUESTTYPE_TAG, n->service_providers[srv_idx].servicerequest_type );
    header_len += pushBackField( packet, &TCPROS_SERVICE_RESPONSETYPE_TAG, n->service_providers[srv_idx].serviceresponse_type );
    header_len += pushBackField( packet, &TCPROS_TYPE_TAG, n->service_providers[srv_idx].service_type );
  //}

  header_out_len = HOST_TO_ROS_UINT32( header_len );
  uint32_t *header_len_p = (uint32_t *)dynBufferGetData( packet );
  *header_len_p = header_out_len;
}

cRosErrCodePack cRosMessagePrepareServiceResponsePacket( CrosNode *n, int server_idx)
{
  cRosErrCodePack ret_err;
  uint8_t ok_byte; // OK field (byte size) of the service response packet

  PRINT_VVDEBUG("cRosMessageParseServiceArgumentsPacket()\n");
  TcprosProcess *server_proc = &(n->rpcros_server_proc[server_idx]);
  DynBuffer *packet = &(server_proc->packet);
  int srv_idx = server_proc->service_idx;
  void* service_context = n->service_providers[srv_idx].context;
  DynBuffer service_response;
  dynBufferInit(&service_response);

  ret_err = cRosNodeDeserializeIncomingPacket(packet, service_context); // prepare the context incoming message used by the user callback function
  if(ret_err == CROS_SUCCESS_ERR_PACK)
    ret_err = cRosNodeServiceProviderCallback(service_context); // calls the service-provider application-defined callback function
  else
    cRosPrintErrCodePack(ret_err, "cRosMessagePrepareServiceResponsePacket() failed decoding the received packet");

  if(ret_err == CROS_SUCCESS_ERR_PACK)
  {
    ret_err =  cRosNodeSerializeOutgoingMessage(&service_response, service_context); // Create the output packet from outgoing message in the context
    if(ret_err != CROS_SUCCESS_ERR_PACK)
      cRosPrintErrCodePack(ret_err, "cRosNodeServiceProviderCallback() failed encoding the packet to send");
  }

  dynBufferClear(packet); // clear packet buffer

  if(ret_err == CROS_SUCCESS_ERR_PACK)
  {
    ok_byte = TCPROS_OK_BYTE_SUCCESS;
    dynBufferPushBackBuf( packet, &ok_byte, sizeof(uint8_t) );
    dynBufferPushBackUInt32( packet, dynBufferGetSize(&service_response)); // data size field
    dynBufferPushBackBuf( packet, dynBufferGetData(&service_response), dynBufferGetSize(&service_response)); // Response data
  }
  else
  {
    ok_byte = TCPROS_OK_BYTE_FAIL;
    dynBufferPushBackBuf( packet, &ok_byte, sizeof(uint8_t) );
    dynBufferPushBackUInt32( packet, 0); // Serialize an error string of size 0: Just add the data size field
  }

  dynBufferRelease(&service_response);

  return ret_err;
}
