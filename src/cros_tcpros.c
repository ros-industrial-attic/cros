#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <ctype.h>

#include "cros_tcpros.h"
#include "cros_defs.h"
#include "tcpros_tags.h"
#include "tcpros_process.h"
#include "dyn_buffer.h"

static uint32_t getLen( DynBuffer *pkt )
{
  uint32_t len;
  const unsigned char *data = dynBufferGetCurrentData(pkt);
  ROS_TO_HOST_UINT32(*((uint32_t *)data), len);
  dynBufferMovePoseIndicator(pkt,sizeof(uint32_t));
  
  return len;
}

static uint32_t pushBackField( DynBuffer *pkt, TcprosTagStrDim *tag, const char *val )
{
  size_t val_len = strlen( val );
  uint32_t out_len, field_len = tag->dim + val_len;
  //PRINT_DEBUG("pushBackField() : filed : %s field_len ; %d\n", tag->str, field_len);
  HOST_TO_ROS_UINT32( field_len, out_len );
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
  
  printf("Header len %d\n",bytes_to_read);
  while ( bytes_to_read > 0)
  {
    uint32_t field_len = getLen( pkt );
    const char *field = (const char *)dynBufferGetCurrentData( pkt );
    if( field_len )
    {
      fwrite ( field, 1, field_len, stdout );
      printf("\n" );
      dynBufferMovePoseIndicator( pkt, field_len );
    }
    bytes_to_read -= ( sizeof(uint32_t) + field_len );
  }
  
  if( print_data )
  {
    bytes_to_read = getLen( pkt );
    
    printf("Data len %d\n",bytes_to_read);
    while ( bytes_to_read > 0)
    {
      uint32_t field_len = getLen( pkt );
      const char *field = (const char *)dynBufferGetCurrentData( pkt );
      if( field_len )
      {
        fwrite ( field, 1, field_len, stdout );
        printf("\n");
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
  PRINT_VDEBUG("readSubcriptioHeader()\n");
  DynBuffer *packet = &(p->packet);
  uint32_t bytes_to_read = getLen( packet );
  size_t packet_len = dynBufferGetSize( packet );

  if( bytes_to_read > packet_len - sizeof( uint32_t ) )
    return TCPROS_PARSER_HEADER_INCOMPLETE;

  *flags = 0x0;

  PRINT_DEBUG("readSubcriptioHeader() : Header len=%d\n",bytes_to_read);

  while ( bytes_to_read > 0)
  {
    uint32_t field_len = getLen( packet );

    PRINT_DEBUG("readSubcriptioHeader() : Field len=%d\n",field_len);

    const char *field = (const char *)dynBufferGetCurrentData( packet );
    if( field_len )
    {
      if ( field_len > (uint32_t)TCPROS_CALLERID_TAG.dim &&
          strncmp ( field, TCPROS_CALLERID_TAG.str, TCPROS_CALLERID_TAG.dim ) == 0 )
      {
        field += TCPROS_CALLERID_TAG.dim;

        dynStringPushBackStrN( &(p->caller_id), field, 
                               field_len - TCPROS_CALLERID_TAG.dim );
        *flags |= TCPROS_CALLER_ID_FLAG;
        dynBufferMovePoseIndicator( packet, field_len );
      }
      else if ( field_len > (uint32_t)TCPROS_TOPIC_TAG.dim &&
          strncmp ( field, TCPROS_TOPIC_TAG.str, TCPROS_TOPIC_TAG.dim ) == 0 )
      {
        field += TCPROS_TOPIC_TAG.dim;

        dynStringPushBackStrN( &(p->topic), field, 
                               field_len - TCPROS_TOPIC_TAG.dim );
        *flags |= TCPROS_TOPIC_FLAG;
        dynBufferMovePoseIndicator( packet, field_len );
      }
      else if ( field_len > (uint32_t)TCPROS_TYPE_TAG.dim &&
          strncmp ( field, TCPROS_TYPE_TAG.str, TCPROS_TYPE_TAG.dim ) == 0 )
      {
        field += TCPROS_TYPE_TAG.dim;

        dynStringPushBackStrN( &(p->type), field, 
                               field_len - TCPROS_TYPE_TAG.dim );
        *flags |= TCPROS_TYPE_FLAG;
        dynBufferMovePoseIndicator( packet, field_len );
      }
      else if ( field_len > (uint32_t)TCPROS_MD5SUM_TAG.dim &&
          strncmp ( field, TCPROS_MD5SUM_TAG.str, TCPROS_MD5SUM_TAG.dim ) == 0 )
      {
        field += TCPROS_MD5SUM_TAG.dim;

        dynStringPushBackStrN( &(p->md5sum), field, 
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
        PRINT_INFO("readSubcriptioHeader() WARNING : TCPROS_TCP_NODELAY_TAG not implemented\n");
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
  PRINT_VDEBUG("readPublicationHeader()\n");
  DynBuffer *packet = &(p->packet);
  size_t bytes_to_read = dynBufferGetSize( packet );

  *flags = 0x0;

  PRINT_DEBUG("readPublicationHeader() : Header len=%d\n",bytes_to_read);

  while ( bytes_to_read > 0)
  {
    uint32_t field_len = getLen( packet );

    PRINT_DEBUG("readPublicationHeader() : Field len=%d\n",field_len);

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

        dynStringPushBackStrN( &(p->caller_id), field,
                               field_len - TCPROS_CALLERID_TAG.dim );
        *flags |= TCPROS_CALLER_ID_FLAG;
        dynBufferMovePoseIndicator( packet, field_len );
      }
      else if ( field_len > (uint32_t)TCPROS_TYPE_TAG.dim &&
          strncmp ( field, TCPROS_TYPE_TAG.str, TCPROS_TYPE_TAG.dim ) == 0 )
      {
        field += TCPROS_TYPE_TAG.dim;

        dynStringPushBackStrN( &(p->type), field,
                               field_len - TCPROS_TYPE_TAG.dim );
        *flags |= TCPROS_TYPE_FLAG;
        dynBufferMovePoseIndicator( packet, field_len );
      }
      else if ( field_len > (uint32_t)TCPROS_MD5SUM_TAG.dim &&
          strncmp ( field, TCPROS_MD5SUM_TAG.str, TCPROS_MD5SUM_TAG.dim ) == 0 )
      {
        field += TCPROS_MD5SUM_TAG.dim;

        dynStringPushBackStrN( &(p->md5sum), field,
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

        dynStringPushBackStrN( &(p->topic), field,
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
  PRINT_VDEBUG("cRosMessageParseSubcriptionHeader()\n");
  
  TcprosProcess *server_proc = &(n->tcpros_server_proc[server_idx]);
  DynBuffer *packet = &(server_proc->packet);
  
  /* Save position indicator: it will be restored */
  int initial_pos_idx = dynBufferGetPoseIndicatorOffset ( packet );
  dynBufferRewindPoseIndicator ( packet );

  uint32_t header_flags; 
  TcprosParserState ret = readSubcriptionHeader( server_proc, &header_flags );
  if( ret != TCPROS_PARSER_DONE )
    return ret;
  
  if( TCPROS_SUBCRIPTION_HEADER_FLAGS != ( header_flags&TCPROS_SUBCRIPTION_HEADER_FLAGS) )
  {
    PRINT_ERROR("cRosMessageParseSubcriptionHeader() : Missing fields\n");
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

      if( strcmp(pub->topic_name, dynStringGetData(&(server_proc->topic))) == 0 &&
          strcmp(pub->topic_type, dynStringGetData(&(server_proc->type))) == 0 &&
          strcmp(pub->md5sum, dynStringGetData(&(server_proc->md5sum))) == 0)
      {
        topic_found = 1;
        server_proc->topic_idx = i;
        pub->client_tcpros_id = server_idx;
        break;
      }
    }

    if( ! topic_found )
    {
      PRINT_ERROR("cRosMessageParseSubcriptionHeader() : Wrong service, type or md5sum\n");
      server_proc->topic_idx = -1;
      ret = TCPROS_PARSER_ERROR;
    }
  }

  /* Restore position indicator */
  dynBufferSetPoseIndicator ( packet, initial_pos_idx );

  return ret;
}

TcprosParserState cRosMessageParsePublicationHeader( CrosNode *n, int client_idx )
{
  PRINT_VDEBUG("cRosMessageParsePublicationHeader()\n");

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
  }

  /* Restore position indicator */
  dynBufferSetPoseIndicator ( packet, initial_pos_idx );

  return ret;
}

void cRosMessagePrepareSubcriptionHeader( CrosNode *n, int client_idx )
{
  PRINT_VDEBUG("cRosMessagePrepareSubcriptionHeader()\n");

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

  HOST_TO_ROS_UINT32( header_len, header_out_len );
  uint32_t *header_len_p = (uint32_t *)dynBufferGetData( packet );
  *header_len_p = header_out_len;
}

void cRosMessageParsePublicationPacket( CrosNode *n, int client_idx )
{
  TcprosProcess *client_proc = &(n->tcpros_client_proc[client_idx]);
  DynBuffer *packet = &(client_proc->packet);
  int sub_idx = client_proc->topic_idx;
  void* data_context = n->subs[sub_idx].context;
  n->subs[sub_idx].callback(packet,data_context);
}

void cRosMessagePreparePublicationHeader( CrosNode *n, int server_idx )
{
  PRINT_VDEBUG("cRosMessagePreparePublicationHeader()\n");
    
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
  
  HOST_TO_ROS_UINT32( header_len, header_out_len );
  uint32_t *header_len_p = (uint32_t *)dynBufferGetData( packet );
  *header_len_p = header_out_len;
}

void cRosMessagePreparePublicationPacket( CrosNode *n, int server_idx )
{
  PRINT_VDEBUG("cRosMessagePreparePublicationPacket()\n");
  //cRosMessagePreparePublicationHeader( n, server_idx );
  TcprosProcess *server_proc = &(n->tcpros_server_proc[server_idx]);
  int pub_idx = server_proc->topic_idx;
  DynBuffer *packet = &(server_proc->packet);
  dynBufferPushBackUInt32( packet, 0 ); // Placehoder for packet size

  void* data_context = n->pubs[pub_idx].context;
  n->pubs[pub_idx].callback( packet, data_context);

  uint32_t size = (uint32_t)dynBufferGetSize(packet) - sizeof(uint32_t);
  memcpy(packet->data, &size, sizeof(uint32_t));
}

static TcprosParserState readServiceCallHeader( TcprosProcess *p, uint32_t *flags )
{
  PRINT_VDEBUG("readServiceCallHeader()\n");
  DynBuffer *packet = &(p->packet);
  size_t bytes_to_read = dynBufferGetSize( packet );

  *flags = 0x0;

  PRINT_DEBUG("readServiceCallHeader() : Header len=%d\n",bytes_to_read);

  while ( bytes_to_read > 0)
  {
    uint32_t field_len = getLen( packet );

    PRINT_DEBUG("readServiceCallHeader() : Field len=%d\n",field_len);

    const char *field = (const char *)dynBufferGetCurrentData( packet );

    if( field_len )
    {
      if ( field_len > (uint32_t)TCPROS_CALLERID_TAG.dim &&
          strncmp ( field, TCPROS_CALLERID_TAG.str, TCPROS_CALLERID_TAG.dim ) == 0 )
      {
        field += TCPROS_CALLERID_TAG.dim;

        dynStringPushBackStrN( &(p->caller_id), field,
                               field_len - TCPROS_CALLERID_TAG.dim );
        *flags |= TCPROS_CALLER_ID_FLAG;
        dynBufferMovePoseIndicator( packet, field_len );
      }
      else if ( field_len > (uint32_t)TCPROS_TYPE_TAG.dim &&
          strncmp ( field, TCPROS_TYPE_TAG.str, TCPROS_TYPE_TAG.dim ) == 0 )
      {
        field += TCPROS_TYPE_TAG.dim;

        dynStringPushBackStrN( &(p->type), field,
                               field_len - TCPROS_TYPE_TAG.dim );
        *flags |= TCPROS_TYPE_FLAG;
        dynBufferMovePoseIndicator( packet, field_len );
      }
      else if ( field_len > (uint32_t)TCPROS_MD5SUM_TAG.dim &&
          strncmp ( field, TCPROS_MD5SUM_TAG.str, TCPROS_MD5SUM_TAG.dim ) == 0 )
      {
        field += TCPROS_MD5SUM_TAG.dim;

        dynStringPushBackStrN( &(p->md5sum), field,
                               field_len - TCPROS_MD5SUM_TAG.dim );
        *flags |= TCPROS_MD5SUM_FLAG;
        dynBufferMovePoseIndicator( packet, field_len );
      }
      else if ( field_len > (uint32_t)TCPROS_SERVICE_TAG.dim &&
          strncmp ( field, TCPROS_SERVICE_TAG.str, TCPROS_SERVICE_TAG.dim ) == 0 )
      {
        field += TCPROS_SERVICE_TAG.dim;

        dynStringPushBackStrN( &(p->service), field,
                               field_len - TCPROS_SERVICE_TAG.dim );
        *flags |= TCPROS_SERVICE_FLAG;
        dynBufferMovePoseIndicator( packet, field_len );
      }
      else if ( field_len > (uint32_t)TCPROS_PERSISTENT_TAG.dim &&
          strncmp ( field, TCPROS_PERSISTENT_TAG.str, TCPROS_PERSISTENT_TAG.dim ) == 0 )
      {
        PRINT_INFO("readPublicationHeader() WARNING : TCPROS_LATCHING_TAG not implemented\n");
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
  PRINT_VDEBUG("cRosMessageParseServiceCallerHeader()\n");

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

  if( header_flags == ( header_flags & TCPROS_SERVICECALL_HEADER_FLAGS) )
  {
    int i = 0;
    for( i = 0 ; i < n->n_services; i++)
    {
      if( strcmp( n->services[i].service_name, dynStringGetData(&(server_proc->service))) == 0 &&
          strcmp( n->services[i].md5sum, dynStringGetData(&(server_proc->md5sum))) == 0
          )
      {
        service_found = 1;
        server_proc->service_idx = i;
        break;
      }
    }
  }
  else if( header_flags == ( header_flags & TCPROS_SERVICEPROBE_HEADER_FLAGS) )
  {
    int i = 0;
    for( i = 0 ; i < n->n_services; i++)
    {
      if( strcmp( n->services[i].service_name, dynStringGetData(&(server_proc->service))) == 0)
      {
        service_found = 1;
        server_proc->service_idx = i;
        break;
      }
    }
  }
  else
  {
    PRINT_ERROR("cRosMessageParseServiceCallerHeader() : Missing fields\n");
    ret = TCPROS_PARSER_ERROR;
  }

  if( ! service_found )
  {
    PRINT_ERROR("cRosMessageParseServiceCallerHeader() : Wrong service, type or md5sum\n");
    server_proc->service_idx = -1;
    ret = TCPROS_PARSER_ERROR;
  }

  /* Restore position indicator */
  dynBufferSetPoseIndicator ( packet, initial_pos_idx );

  return ret;
}

void cRosMessagePrepareServiceProviderHeader( CrosNode *n, int server_idx)
{
  PRINT_VDEBUG("cRosMessagePreparePublicationHeader()\n");

  TcprosProcess *server_proc = &(n->rpcros_server_proc[server_idx]);
  int srv_idx = server_proc->service_idx;
  DynBuffer *packet = &(server_proc->packet);
  uint32_t header_len = 0, header_out_len = 0;
  dynBufferPushBackUInt32( packet, header_out_len );

  // http://wiki.ros.org/ROS/TCPROS doesn't mention to send message_definition and topic_name
  // but they are sent anyway in ros groovy
  header_len += pushBackField( packet, &TCPROS_CALLERID_TAG, n->name );
  header_len += pushBackField( packet, &TCPROS_MD5SUM_TAG, n->services[srv_idx].md5sum );

  //if(server_proc->probe)
  //{
    header_len += pushBackField( packet, &TCPROS_SERVICE_REQUESTTYPE_TAG, n->services[srv_idx].servicerequest_type );
    header_len += pushBackField( packet, &TCPROS_SERVICE_RESPONSETYPE_TAG, n->services[srv_idx].serviceresponse_type );
    header_len += pushBackField( packet, &TCPROS_TYPE_TAG, n->services[srv_idx].service_type );
  //}

  HOST_TO_ROS_UINT32( header_len, header_out_len );
  uint32_t *header_len_p = (uint32_t *)dynBufferGetData( packet );
  *header_len_p = header_out_len;
}

void cRosMessagePrepareServiceResponsePacket( CrosNode *n, int server_idx)
{
  PRINT_VDEBUG("cRosMessageParseServiceArgumentsPacket()\n");
  TcprosProcess *server_proc = &(n->rpcros_server_proc[server_idx]);
  DynBuffer *packet = &(server_proc->packet);
  int srv_idx = server_proc->service_idx;
  void* service_context = n->services[srv_idx].context;
  DynBuffer service_response;
  dynBufferInit(&service_response);

  CallbackResponse callback_response = n->services[srv_idx].callback(packet, &service_response, service_context);

  //clear packet buffer
  dynBufferClear(packet);

  //OK field (byte size)
  unsigned char ok = 1;
  dynBufferPushBackBuf( packet, &ok, 1 );

  //Size data field
  dynBufferPushBackUInt32( packet, service_response.size);

  //Response data
  dynBufferPushBackBuf( packet, service_response.data, service_response.size);

  dynBufferRelease(&service_response);
}
