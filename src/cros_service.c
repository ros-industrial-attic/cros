#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <ctype.h>

#include "cros_service.h"
#include "tcpros_tags.h"
#include "cros_defs.h"
#include "md5.h"

void initCrosSrv(cRosSrvDef* srv)
{
    srv->request = (cRosMessageDef*) malloc(sizeof(cRosMessageDef));
    initCrosMsg(srv->request);
    srv->response = (cRosMessageDef*) malloc(sizeof(cRosMessageDef));
    initCrosMsg(srv->response);
    srv->name = NULL;
    srv->package = NULL;
    srv->plain_text = NULL;
    srv->root_dir = NULL;
}

int loadFromFileSrv(char* filename, cRosSrvDef* srv)
{
    char* file_tokenized = (char*) malloc(strlen(filename)+1);
    file_tokenized[0] = '\0';
    strcpy(file_tokenized, filename);
    char* token_pack;
    char* token_root = NULL;
    char* token_name = NULL;

    FILE *f = fopen(filename, "rb");

    if(f != NULL)
    {
        fseek(f, 0, SEEK_END);
        long fsize = ftell(f);
        fseek(f, 0, SEEK_SET);
        char *srv_req = NULL;
        char *srv_res = NULL;
        char *srv_text = malloc(fsize + 1);
        fread(srv_text, fsize, 1, f);
        fclose(f);

        srv_text[fsize] = '\0';
        srv->plain_text = malloc(strlen(srv_text) + 1);
        memcpy(srv->plain_text,srv_text,strlen(srv_text) + 1);

        //splitting msg_text into the request response parts
        srv_res = strstr(srv_text, SRV_DELIMITER);

        //if the srv has some request parameters
        if(srv_res != srv_text)
        {
          //split before the first delim char
          *(srv_res - 1) = '\0';
          srv_req = srv_text;
        }

        //move over the delimiter and the new line char
        srv_res += strlen(SRV_DELIMITER) + 1;

        char* tok = strtok(file_tokenized,"/.");

        while(tok != NULL)
        {
            if(strcmp(tok, "srv") != 0)
            {
                token_root = token_pack;
                token_pack = token_name;
                token_name = tok ;
            }
            tok = strtok(NULL,"/.");
        }

        //build up the root path
        char* it = file_tokenized;
        while(it != token_root)
        {
            if(*it == '\0')
                *it='/';
            it++;
        }

        srv->root_dir = (char*) malloc (strlen(file_tokenized)+1); srv->root_dir[0] = '\0';
        strcpy(srv->root_dir,file_tokenized);

        srv->package = (char*) malloc (strlen(token_pack)+1); srv->package[0] = '\0';
        strcpy(srv->package,token_pack);

        srv->name = (char*) malloc (strlen(token_name)+1); srv->name[0] = '\0';
        strcpy(srv->name,token_name);

        if(srv_req != NULL)
        {
          srv->request->package = srv->package;
          srv->request->root_dir = srv->root_dir;
          loadFromStringMsg(srv_req, srv->request);
        }

        srv->response->package = srv->package;
        srv->response->root_dir = srv->root_dir;
        loadFromStringMsg(srv_res, srv->response);

        free(srv_text);
    }

    free(file_tokenized);
    return EXIT_SUCCESS;
}

//  Compute dependencies of the specified service file
int getFileDependenciesSrv(char* filename, cRosSrvDef* srv, msgDep* deps)
{
    loadFromFileSrv(filename, srv);
    getDependenciesMsg(srv->request,deps);
    getDependenciesMsg(srv->response,deps);
    return EXIT_SUCCESS;
}

//  Compute full text of service, including text of embedded
//  types.  The text of the main srv is listed first. Embedded
//  srv files are denoted first by an 80-character '=' separator,
//  followed by a type declaration line,'MSG: pkg/type', followed by
//  the text of the embedded type.
char* computeFullTextSrv(cRosSrvDef* srv, msgDep* deps)
{
    char* full_text = NULL;
    char* msg_tag = "MSG: ";
    int full_size = 0;
    char separator[81]; separator[80] = '\0';
    int i;
    memset(&separator,'=', 80);
    full_size += strlen(srv->plain_text);

    while(deps->next != NULL)
    {
        //printf("%s\nMSG: %s\n", separator, deps->msg->name);
        full_size = strlen(deps->msg->plain_text) + strlen(separator) + strlen(msg_tag) + 3/*New lines*/;
        deps = deps->next;
    }

    //rollback
    while(deps->prev != NULL) deps = deps->prev;
    full_text = (char*) malloc(full_size + 1);
    memcpy(full_text,srv->plain_text,strlen(srv->plain_text) + 1);

    while(deps->next != NULL)
    {
        //printf("%s\nMSG: %s\n", separator, deps->msg->name);
        strcat(full_text, "\n");
        strcat(full_text, separator);
        strcat(full_text, "\n");
        strcat(full_text, msg_tag);
        strcat(full_text, deps->msg->package);
        strcat(full_text, "/");
        strcat(full_text, deps->msg->name);
        strcat(full_text, "\n");
        strcat(full_text, deps->msg->plain_text);
        deps = deps->next;
    }
    return full_text;
}

cRosService * cRosServiceNew()
{
  cRosService *ret = (cRosService *)calloc(1, sizeof(cRosService));
  cRosServiceInit(ret);
  return ret;
}

void cRosServiceInit(cRosService* service)
{
  cRosMessageInit(&service->request);
  cRosMessageInit(&service->response);
  service->md5sum = (char*) malloc(33);// 32 chars + '\0';
}

void cRosServiceBuild(cRosService* service, const char* filepath)
{
  cRosServiceBuildInner(&service->request, &service->response, service->md5sum, filepath);
}

void cRosServiceBuildInner(cRosMessage *request, cRosMessage *response, char *md5sum, const char* filepath)
{
  cRosSrvDef* srv = (cRosSrvDef*) malloc(sizeof(cRosSrvDef));
  initCrosSrv(srv);
  char* copy_filepath = malloc(strlen(filepath)+1);
  *copy_filepath = '\0';
  strcpy(copy_filepath,filepath);

  loadFromFileSrv(copy_filepath,srv);

  unsigned char* res = NULL;
  DynString buffer;
  dynStringInit(&buffer);

  MD5_CTX md5_t;
  MD5_Init(&md5_t);

  if(srv->request->plain_text != NULL)
  {
    getMD5Txt(srv->request, &buffer);
  }

  if(buffer.len != 0)
  {
      MD5_Update(&md5_t,buffer.data,buffer.len - 1);
      dynStringClear(&buffer);
  }

  getMD5Txt(srv->response, &buffer);
  MD5_Update(&md5_t,buffer.data,buffer.len - 1);

  unsigned char* result = (unsigned char*) malloc(16);
  MD5_Final(result, &md5_t);
  DynString output;
  dynStringInit(&output);
  cRosMD5Readable(result,&output);

  strcpy(md5sum, output.data);
  if(srv->request->plain_text != NULL)
  {
    cRosMessageBuildFromDef(request, srv->request);
  }
  cRosMessageBuildFromDef(response, srv->response);
}

void cRosServiceFree(cRosService* service)
{
  cRosServiceRelease(service);
  free(service);
}

void cRosServiceRelease(cRosService* service)
{
  cRosMessageRelease(&service->request);
  cRosMessageRelease(&service->response);
  free(service->md5sum);
  service->md5sum = NULL;
}

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
  dynBufferPushBackUint32( pkt, out_len );
  dynBufferPushBackBuf( pkt, (const unsigned char*)tag->str, tag->dim );
  dynBufferPushBackBuf( pkt, (const unsigned char*)val, val_len ); 
  
  return field_len + sizeof( uint32_t );
}
//

//static void printPacket( DynBuffer *pkt, int print_data )
//{
//  /* Save position indicator: it will be restored */
//  int initial_pos_idx = dynBufferGetPoseIndicatorOffset ( pkt );
//  dynBufferRewindPoseIndicator ( pkt );
//
//  uint32_t bytes_to_read = getLen( pkt );
//
//  printf("Header len %d\n",bytes_to_read);
//  while ( bytes_to_read > 0)
//  {
//    uint32_t field_len = getLen( pkt );
//    const char *field = (const char *)dynBufferGetCurrentData( pkt );
//    if( field_len )
//    {
//      fwrite ( field, 1, field_len, stdout );
//      printf("\n" );
//      dynBufferMovePoseIndicator( pkt, field_len );
//    }
//    bytes_to_read -= ( sizeof(uint32_t) + field_len );
//  }
//
//  if( print_data )
//  {
//    bytes_to_read = getLen( pkt );
//
//    printf("Data len %d\n",bytes_to_read);
//    while ( bytes_to_read > 0)
//    {
//      uint32_t field_len = getLen( pkt );
//      const char *field = (const char *)dynBufferGetCurrentData( pkt );
//      if( field_len )
//      {
//        fwrite ( field, 1, field_len, stdout );
//        printf("\n");
//        dynBufferMovePoseIndicator( pkt, field_len );
//      }
//      bytes_to_read -= ( sizeof(uint32_t) + field_len );
//    }
//  }
//
//  /* Restore position indicator */
//  dynBufferSetPoseIndicator ( pkt, initial_pos_idx );
//}
//
//static TcprosParserState readSubcriptionHeader( TcprosProcess *p, uint32_t *flags )
//{
//  PRINT_VDEBUG("readSubcriptioHeader()\n");
//  DynBuffer *packet = &(p->packet);
//  uint32_t bytes_to_read = getLen( packet );
//  size_t packet_len = dynBufferGetSize( packet );
//
//  if( bytes_to_read > packet_len - sizeof( uint32_t ) )
//    return TCPROS_PARSER_HEADER_INCOMPLETE;
//
//  *flags = 0x0;
//
//  PRINT_DEBUG("readSubcriptioHeader() : Header len=%d\n",bytes_to_read);
//
//  while ( bytes_to_read > 0)
//  {
//    uint32_t field_len = getLen( packet );
//
//    PRINT_DEBUG("readSubcriptioHeader() : Field len=%d\n",field_len);
//
//    const char *field = (const char *)dynBufferGetCurrentData( packet );
//    if( field_len )
//    {
//      if ( field_len > (uint32_t)TCPROS_CALLERID_TAG.dim &&
//          strncmp ( field, TCPROS_CALLERID_TAG.str, TCPROS_CALLERID_TAG.dim ) == 0 )
//      {
//        field += TCPROS_CALLERID_TAG.dim;
//
//        dynStringPushBackStrN( &(p->caller_id), field,
//                               field_len - TCPROS_CALLERID_TAG.dim );
//        *flags |= TCPROS_CALLER_ID_FLAG;
//        dynBufferMovePoseIndicator( packet, field_len );
//      }
//      else if ( field_len > (uint32_t)TCPROS_TOPIC_TAG.dim &&
//          strncmp ( field, TCPROS_TOPIC_TAG.str, TCPROS_TOPIC_TAG.dim ) == 0 )
//      {
//        field += TCPROS_TOPIC_TAG.dim;
//
//        dynStringPushBackStrN( &(p->topic), field,
//                               field_len - TCPROS_TOPIC_TAG.dim );
//        *flags |= TCPROS_TOPIC_FLAG;
//        dynBufferMovePoseIndicator( packet, field_len );
//      }
//      else if ( field_len > (uint32_t)TCPROS_TYPE_TAG.dim &&
//          strncmp ( field, TCPROS_TYPE_TAG.str, TCPROS_TYPE_TAG.dim ) == 0 )
//      {
//        field += TCPROS_TYPE_TAG.dim;
//
//        dynStringPushBackStrN( &(p->type), field,
//                               field_len - TCPROS_TYPE_TAG.dim );
//        *flags |= TCPROS_TYPE_FLAG;
//        dynBufferMovePoseIndicator( packet, field_len );
//      }
//      else if ( field_len > (uint32_t)TCPROS_MD5SUM_TAG.dim &&
//          strncmp ( field, TCPROS_MD5SUM_TAG.str, TCPROS_MD5SUM_TAG.dim ) == 0 )
//      {
//        field += TCPROS_MD5SUM_TAG.dim;
//
//        dynStringPushBackStrN( &(p->md5sum), field,
//                               field_len - TCPROS_MD5SUM_TAG.dim );
//        *flags |= TCPROS_MD5SUM_FLAG;
//        dynBufferMovePoseIndicator( packet, field_len );
//      }
//      else if ( field_len > (uint32_t)TCPROS_MESSAGE_DEFINITION_TAG.dim &&
//          strncmp ( field, TCPROS_MESSAGE_DEFINITION_TAG.str, TCPROS_MESSAGE_DEFINITION_TAG.dim ) == 0 )
//      {
//        *flags |= TCPROS_MESSAGE_DEFINITION_FLAG;
//        dynBufferMovePoseIndicator( packet, field_len );
//      }
//      else if ( field_len > (uint32_t)TCPROS_TCP_NODELAY_TAG.dim &&
//          strncmp ( field, TCPROS_TCP_NODELAY_TAG.str, TCPROS_TCP_NODELAY_TAG.dim ) == 0 )
//      {
//        PRINT_INFO("readSubcriptioHeader() WARNING : TCPROS_TCP_NODELAY_TAG not implemented\n");
//        field += TCPROS_TCP_NODELAY_TAG.dim;
//        p->tcp_nodelay = (*field == '1')?1:0;
//        *flags |= TCPROS_TCP_NODELAY_FLAG;
//        dynBufferMovePoseIndicator( packet, field_len );
//      }
//      else if ( field_len > (uint32_t)TCPROS_LATCHING_TAG.dim &&
//          strncmp ( field, TCPROS_LATCHING_TAG.str, TCPROS_LATCHING_TAG.dim ) == 0 )
//      {
//        PRINT_INFO("readSubcriptioHeader() WARNING : TCPROS_LATCHING_TAG not implemented\n");
//        field += TCPROS_LATCHING_TAG.dim;
//        p->latching = (*field == '1')?1:0;
//        *flags |= TCPROS_LATCHING_FLAG;
//        dynBufferMovePoseIndicator( packet, field_len );
//      }
//      else if ( field_len > (uint32_t)TCPROS_ERROR_TAG.dim &&
//          strncmp ( field, TCPROS_ERROR_TAG.str, TCPROS_ERROR_TAG.dim ) == 0 )
//      {
//        PRINT_INFO("readSubcriptioHeader() WARNING : TCPROS_ERROR_TAG not implemented\n");
//        *flags |= TCPROS_ERROR_FLAG;
//        dynBufferMovePoseIndicator( packet, field_len );
//      }
//      else
//      {
//        PRINT_ERROR("readSubcriptioHeader() : unknown field\n");
//        *flags = 0x0;
//        break;
//      }
//    }
//
//    bytes_to_read -= ( sizeof(uint32_t) + field_len );
//  }
//
//  return TCPROS_PARSER_DONE;
//
//}
//
//static TcprosParserState readPublicationHeader( TcprosProcess *p, uint32_t *flags )
//{
//  PRINT_VDEBUG("readPublicationHeader()\n");
//  DynBuffer *packet = &(p->packet);
//  size_t bytes_to_read = dynBufferGetSize( packet );
//
//  *flags = 0x0;
//
//  PRINT_DEBUG("readPublicationHeader() : Header len=%d\n",bytes_to_read);
//
//  while ( bytes_to_read > 0)
//  {
//    uint32_t field_len = getLen( packet );
//
//    PRINT_DEBUG("readPublicationHeader() : Field len=%d\n",field_len);
//
//    const char *field = (const char *)dynBufferGetCurrentData( packet );
//    if( field_len )
//    {
//      // http://wiki.ros.org/ROS/TCPROS doesn't mention it but it's sent anyway in ros groovy
//      if ( field_len > (uint32_t)TCPROS_MESSAGE_DEFINITION_TAG.dim &&
//          strncmp ( field, TCPROS_MESSAGE_DEFINITION_TAG.str, TCPROS_MESSAGE_DEFINITION_TAG.dim ) == 0 )
//      {
//        *flags |= TCPROS_MESSAGE_DEFINITION_FLAG;
//        dynBufferMovePoseIndicator( packet, field_len );
//      } else if ( field_len > (uint32_t)TCPROS_CALLERID_TAG.dim &&
//          strncmp ( field, TCPROS_CALLERID_TAG.str, TCPROS_CALLERID_TAG.dim ) == 0 )
//      {
//        field += TCPROS_CALLERID_TAG.dim;
//
//        dynStringPushBackStrN( &(p->caller_id), field,
//                               field_len - TCPROS_CALLERID_TAG.dim );
//        *flags |= TCPROS_CALLER_ID_FLAG;
//        dynBufferMovePoseIndicator( packet, field_len );
//      }
//      else if ( field_len > (uint32_t)TCPROS_TYPE_TAG.dim &&
//          strncmp ( field, TCPROS_TYPE_TAG.str, TCPROS_TYPE_TAG.dim ) == 0 )
//      {
//        field += TCPROS_TYPE_TAG.dim;
//
//        dynStringPushBackStrN( &(p->type), field,
//                               field_len - TCPROS_TYPE_TAG.dim );
//        *flags |= TCPROS_TYPE_FLAG;
//        dynBufferMovePoseIndicator( packet, field_len );
//      }
//      else if ( field_len > (uint32_t)TCPROS_MD5SUM_TAG.dim &&
//          strncmp ( field, TCPROS_MD5SUM_TAG.str, TCPROS_MD5SUM_TAG.dim ) == 0 )
//      {
//        field += TCPROS_MD5SUM_TAG.dim;
//
//        dynStringPushBackStrN( &(p->md5sum), field,
//                               field_len - TCPROS_MD5SUM_TAG.dim );
//        *flags |= TCPROS_MD5SUM_FLAG;
//        dynBufferMovePoseIndicator( packet, field_len );
//      }
//      else if ( field_len > (uint32_t)TCPROS_LATCHING_TAG.dim &&
//          strncmp ( field, TCPROS_LATCHING_TAG.str, TCPROS_LATCHING_TAG.dim ) == 0 )
//      {
//        PRINT_INFO("readPublicationHeader() WARNING : TCPROS_LATCHING_TAG not implemented\n");
//        field += TCPROS_LATCHING_TAG.dim;
//        p->latching = (*field == '1')?1:0;
//        *flags |= TCPROS_LATCHING_FLAG;
//        dynBufferMovePoseIndicator( packet, field_len );
//      }
//      else if ( field_len > (uint32_t)TCPROS_TOPIC_TAG.dim &&
//          strncmp ( field, TCPROS_TOPIC_TAG.str, TCPROS_TOPIC_TAG.dim ) == 0 )
//      {
//        // http://wiki.ros.org/ROS/TCPROS doesn't mention it but it's sent anyway in ros groovy
//        field += TCPROS_TOPIC_TAG.dim;
//
//        dynStringPushBackStrN( &(p->topic), field,
//                               field_len - TCPROS_TOPIC_TAG.dim );
//        *flags |= TCPROS_TOPIC_FLAG;
//        dynBufferMovePoseIndicator( packet, field_len );
//      }
//      else if ( field_len > (uint32_t)TCPROS_ERROR_TAG.dim &&
//          strncmp ( field, TCPROS_ERROR_TAG.str, TCPROS_ERROR_TAG.dim ) == 0 )
//      {
//        PRINT_INFO("readPublicationHeader() WARNING : TCPROS_ERROR_TAG not implemented\n");
//        *flags |= TCPROS_ERROR_FLAG;
//        dynBufferMovePoseIndicator( packet, field_len );
//      }
//      else
//      {
//        PRINT_ERROR("readPublicationHeader() : unknown field\n");
//        *flags = 0x0;
//        break;
//      }
//    }
//
//    bytes_to_read -= ( sizeof(uint32_t) + field_len );
//  }
//
//  return TCPROS_PARSER_DONE;
//}
//
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
  dynBufferPushBackUint32( packet, header_out_len );

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
  dynBufferPushBackUint32( packet, service_response.size);

  //Response data
  dynBufferPushBackBuf( packet, service_response.data, service_response.size);

  dynBufferRelease(&service_response);
}
