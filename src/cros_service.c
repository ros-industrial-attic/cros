#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <ctype.h>

#include "cros_service.h"
#include "cros_service_internal.h"
#include "cros_message_internal.h"
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
