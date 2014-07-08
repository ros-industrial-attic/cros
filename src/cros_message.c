#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <ctype.h>
#include <assert.h>

#include "cros_message.h"
#include "tcpros_tags.h"
#include "cros_defs.h"
#include "md5.h"

static int arrayFieldValueAt(cRosMessageField *field, int position, int element_size, void* data);

char* base_msg_type(char* type)
{
    //  """
    //  Compute the base data type, e.g. for arrays, get the underlying array item type
    //  @param type_: ROS msg type (e.g. 'std_msgs/String')
    //  @type  type_: str
    //  @return: base type
    //  @rtype: str
    //  """
    char* base = NULL;
    int char_count = 0;
    char* iterator = type;

    while( *iterator != '\0' && *iterator != '[')
    {
        char_count++;
        iterator++;
    }

    base = malloc(char_count + 1);
    memcpy(base, type, char_count);
    base[char_count] = '\0';

    return base;
}

int is_valid_msg_type(char* type_statement)
{
    //@return: True if the name is a syntatically legal message type name
    //    @rtype: bool
  //if not x or len(x) != len(x.strip()):
  //    return False
    char* iterator = type_statement;
    while(*iterator != '\0')
    {
        if(*iterator == ' ')
        {
            return 0;
        }
        iterator ++;
    }

//  base = base_msg_type(x)
    char* base = base_msg_type(type_statement);
    //  if not roslib.names.is_legal_resource_name(base):
    //          return False
    int slash_found = 0;
    char* base_itr = base;

    while(*base_itr != '\0')
    {
        if((*base_itr == ']') ||
            (*base_itr == '/' && slash_found))
        {
            free(base);
            return 0;
        }
        if(*base_itr == '/')
            slash_found = 1;
        base_itr++;
    }

    // TODO Check that the array index is a number

    free(base);
    return 1;
}

int is_array_type(char* type_statement, int* size)
{
    //@return: True if the name is a syntatically legal message type name
    //    @rtype: bool
  //if not x or len(x) != len(x.strip()):
  //    return False
  char* type_it = type_statement;
  char* array_size = "-1";
  char* num_start;
  int count = 0;
  int start_count = 0;

  while(*type_it != '\0' && *type_it != ']' && *type_it!= ' ')
  {
    if(start_count)
    {
      count++;
    }

    if(*type_it == '[' )
    {
      start_count = 1;
      num_start = type_it;
    }

    type_it++;
  }

  if(count == 0 && start_count == 0)
    return 0;

  array_size = calloc(count + 1, sizeof(char));
  memcpy(array_size,num_start + 1,count);
  *size = atoi(array_size);
  return 1;
}

int containsDep(msgDep* iterator, char* depName)
{
    char* dep = (char*) malloc(strlen(depName)+1);
    memcpy(dep,depName, strlen(depName)+1);
    char* pack = dep;
    char* name = dep;
    while(*name != '/') name++;
    *(name++) = '\0';

    int found = 0;
    //rollback
    while((iterator)->prev != NULL) iterator = iterator->prev;
    while(iterator->next != NULL && iterator->msg != NULL)
    {
        if(strcmp(iterator->msg->package, pack) == 0 &&
             strcmp(iterator->msg->name, name) == 0)
        {
            found = 1;
            break;
        }
        iterator = iterator->next;
    }

    free(dep);
    return found;
}

//Add the list of message types that spec depends on to depends.
int getDependenciesMsg(cRosMessageDef* msg, msgDep* msgDeps)
{
    //move on until you reach the head
    while(msgDeps->next != NULL) msgDeps = msgDeps->next;
    msgDep* currentDep = msgDeps ;

    msgFieldDef* fields = msg->first_field;

    while(fields->next != NULL)
    {
        msgFieldDef* currentField = fields;

        if(!is_builtin_type(currentField->type))
        {
            char *base_type = base_msg_type(currentField->type_s);
            if(containsDep(msgDeps, base_type))
            {
                fields = fields->next;
                free(base_type);
                continue;
            }

            currentDep->msg = (cRosMessageDef*) malloc(sizeof(cRosMessageDef));
            // special mapping for header
            if(currentField->type == CROS_STD_MSGS_HEADER)
            {
                //have to re-names Header
                currentDep->msg->package = (char*) malloc(strlen(HEADER_DEFAULT_PACK) + 1);
                currentDep->msg->package[0] = '\0';
                strcpy(currentDep->msg->package,HEADER_DEFAULT_PACK);
                currentDep->msg->name = (char*) malloc(strlen(HEADER_DEFAULT_NAME) + 1);
                currentDep->msg->name[0] = '\0';
                strcpy(currentDep->msg->name,HEADER_DEFAULT_NAME);

                currentDep->msg->plain_text = (char*) malloc(strlen(HEADER_DEFAULT_TYPEDEF) + 1);
                memcpy(currentDep->msg->plain_text,"\0",1);
                strcpy(currentDep->msg->plain_text, HEADER_DEFAULT_TYPEDEF);
                msgDep* next = malloc(sizeof(msgDep));
                next->msg = NULL;
                next->prev = currentDep;
                currentDep->next = next;
                currentDep = next;
            }
            else
            {

                if(strstr(base_type,"/") == NULL)
                {
                    char* trail = msg->package;
                    currentDep->msg->name = (char*) malloc(strlen(trail) + strlen(base_type) + 1 + 1);
                    memcpy(currentDep->msg->name, trail, strlen(trail) + 1);
                    strcat(currentDep->msg->name, "/");
                    strcat(currentDep->msg->name, base_type);
                }
                else
                {

                    char* dep = (char*) malloc(strlen(base_type)+1);
                    memcpy(dep,base_type, strlen(base_type)+1);
                    char* pack = dep;
                    char* name = dep;
                    while(*name != '/') name++;
                    *(name++) = '\0';
                    currentDep->msg->name = name;
                    currentDep->msg->package = pack;
                }

                currentDep->msg->fields = (msgFieldDef*) calloc(1, sizeof(msgFieldDef));
                initFieldDef(currentDep->msg->fields);
                currentDep->msg->first_field = currentDep->msg->fields;
                currentDep->msg->constants = (msgConst*) malloc(sizeof(msgConst));
                currentDep->msg->first_const = currentDep->msg->constants;

                char* path = (char*) malloc(
                                            strlen(msg->root_dir) + 1 +
                                            strlen(currentDep->msg->package) + 1 +
                                            strlen(currentDep->msg->name) + 4 + 1);
                memcpy(path, msg->root_dir, strlen(msg->root_dir) + 1);
                strcat(path, "/");
                strcat(path, currentDep->msg->package);
                strcat(path, "/");
                strcat(path,currentDep->msg->name);
                strcat(path,".msg");

                loadFromFileMsg(path,currentDep->msg);
                msgDep* next = malloc(sizeof(msgDep));
                next->msg = NULL;
                next->prev = currentDep;
                currentDep->next = next;
                currentDep = next;
                getDependenciesMsg(currentDep->prev->msg, currentDep);
            }

            free(base_type);
        }
        fields = fields->next;
    }
    return EXIT_SUCCESS;
}

//  Compute dependencies of the specified message file
int getFileDependenciesMsg(char* filename, cRosMessageDef* msg, msgDep* deps)
{
    loadFromFileMsg(filename, msg);
    getDependenciesMsg(msg,deps);
    return EXIT_SUCCESS;
}

//  Compute full text of message, including text of embedded
//  types.  The text of the main msg is listed first. Embedded
//  msg files are denoted first by an 80-character '=' separator,
//  followed by a type declaration line,'MSG: pkg/type', followed by
//  the text of the embedded type.
char* computeFullTextMsg(cRosMessageDef* msg, msgDep* deps)
{
    char* full_text = NULL;
    char* msg_tag = "MSG: ";
    int full_size = 0;
    char separator[81]; separator[80] = '\0';
    int i;
    for(i = 0; i < 80; i++)
    {
        separator[i] = '=';
    }

    full_size += strlen(msg->plain_text);

    while(deps->next != NULL)
    {
        //printf("%s\nMSG: %s\n", separator, deps->msg->name);
        full_size = strlen(deps->msg->plain_text) + strlen(separator) + strlen(msg_tag) + 3/*New lines*/;
        deps = deps->next;
    }

    //rollback
    while(deps->prev != NULL) deps = deps->prev;
    full_text = (char*) malloc(full_size + 1);
    memcpy(full_text,msg->plain_text,strlen(msg->plain_text) + 1);
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

void initCrosMsg(cRosMessageDef* msg)
{
  msg->constants = (msgConst*) malloc(sizeof(msgConst));
  initMsgConst(msg->constants);
  msg->first_const = msg->constants;
  msg->fields = (msgFieldDef*) calloc(1, sizeof(msgFieldDef));
  initFieldDef(msg->fields);
  msg->first_field = msg->fields;
  msg->name = NULL;
  msg->package = NULL;
  msg->plain_text = NULL;
  msg->root_dir = NULL;
}

void initMsgConst(msgConst *msg)
{
  msg->type = CROS_CUSTOM_TYPE;
  msg->type_s = NULL;
  msg->name = NULL;
  msg->value = NULL;
  msg->prev = NULL;
  msg->next = NULL;
}

void initCrosDep(msgDep* dep)
{
    dep->msg = NULL;
    dep->prev = NULL;
    dep->next = NULL;
}

void initFieldDef(msgFieldDef* field)
{
  field->array_size = -1;
  field->is_array = 0;
  field->name = NULL;
  field->type = CROS_CUSTOM_TYPE;
  field->type_s = NULL;
  field->next = NULL;
  field->prev = NULL;
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

unsigned char* getMD5Msg(cRosMessageDef* msg)
{
    DynString buffer;
    unsigned char* result = (unsigned char*) malloc(16);
    int i;

    dynStringInit(&buffer);
    msgConst* const_it = msg->first_const;

    while(const_it->next != NULL)
    {
        dynStringPushBackStr(&buffer,const_it->type_s);
        dynStringPushBackStr(&buffer," ");
        dynStringPushBackStr(&buffer,const_it->name);
        dynStringPushBackStr(&buffer,"=");
        dynStringPushBackStr(&buffer,const_it->value);
        dynStringPushBackStr(&buffer,"\n");
        const_it = const_it->next;
    }

    msgFieldDef* fields_it = msg->first_field;

    while(fields_it->next != NULL)
    {
        const char* base_type;
        if (fields_it->type_s == NULL)
          base_type = getMessageTypeDeclaration(fields_it->type);
        else
          base_type = fields_it->type_s;

        if(is_builtin_type(fields_it->type))
        {
          if(fields_it->is_array)
          {
            dynStringPushBackStr(&buffer, base_type);
            dynStringPushBackStr(&buffer,"[");
            if(fields_it->array_size)
            {
              char num[15];
              sprintf(num, "%d",fields_it->array_size);
              dynStringPushBackStr(&buffer,num);
            }
            dynStringPushBackStr(&buffer,"]");
          }
          else
          {
            dynStringPushBackStr(&buffer, base_type);
          }

          dynStringPushBackStr(&buffer," ");
          dynStringPushBackStr(&buffer,fields_it->name);
          dynStringPushBackStr(&buffer,"\n");
        }
        else if(fields_it->type == CROS_STD_MSGS_HEADER)
        {
            cRosMessageDef* msg = (cRosMessageDef*) malloc(sizeof(cRosMessageDef));
            initCrosMsg(msg);
            char* header_text = malloc(strlen(HEADER_DEFAULT_TYPEDEF) + 1);
            memcpy(header_text,HEADER_DEFAULT_TYPEDEF,strlen(HEADER_DEFAULT_TYPEDEF) + 1);
            loadFromStringMsg(header_text, msg);
            unsigned char* res =  getMD5Msg(msg);
            cRosMD5Readable(res, &buffer);
            dynStringPushBackStr(&buffer," ");
            dynStringPushBackStr(&buffer,fields_it->name);
            dynStringPushBackStr(&buffer,"\n");
        }
        else
        {
            DynString filename_dep;
            dynStringInit(&filename_dep);
            dynStringPushBackStr(&filename_dep,msg->root_dir);
            dynStringPushBackStr(&filename_dep,"/");
            dynStringPushBackStr(&filename_dep, base_type);
            dynStringPushBackStr(&filename_dep,".msg");

            cRosMessage msg;
            cRosMessageInit(&msg);
            cRosMessageBuild(&msg,filename_dep.data);
            char *md5sum = calloc(strlen(msg.md5sum)+1,sizeof(char));
            strcpy(md5sum,msg.md5sum);
            cRosMessageRelease(&msg);

            dynStringPushBackStr(&buffer, md5sum);
            dynStringPushBackStr(&buffer," ");
            dynStringPushBackStr(&buffer,fields_it->name);
            dynStringPushBackStr(&buffer,"\n");
        }
        fields_it = fields_it->next;
    }

    if(buffer.len == 0)
        return NULL;

    MD5_CTX md5_t;
    MD5_Init(&md5_t);
    MD5_Update(&md5_t,buffer.data,buffer.len - 1);
    MD5_Final(result, &md5_t);

    return result;
}

void getMD5Txt(cRosMessageDef* msg, DynString* buffer)
{
    int i;

    dynStringInit(buffer);
    msgConst* const_it = msg->first_const;

    while(const_it->next != NULL)
    {
        dynStringPushBackStr(buffer,const_it->type_s);
        dynStringPushBackStr(buffer," ");
        dynStringPushBackStr(buffer,const_it->name);
        dynStringPushBackStr(buffer,"=");
        dynStringPushBackStr(buffer,const_it->value);
        dynStringPushBackStr(buffer,"\n");
        const_it = const_it->next;
    }

    msgFieldDef* fields_it = msg->first_field;

    while(fields_it->next != NULL)
    {
        if(is_builtin_type(fields_it->type))
        {
            dynStringPushBackStr(buffer,fields_it->type_s);
            dynStringPushBackStr(buffer," ");
            dynStringPushBackStr(buffer,fields_it->name);
            dynStringPushBackStr(buffer,"\n");
        }
        else if(fields_it->type == CROS_STD_MSGS_HEADER)
        {
            cRosMessageDef* msg = (cRosMessageDef*) malloc(sizeof(cRosMessageDef));
            initCrosMsg(msg);
            char* header_text = malloc(strlen(HEADER_DEFAULT_TYPEDEF) + 1);
            memcpy(header_text,HEADER_DEFAULT_TYPEDEF,strlen(HEADER_DEFAULT_TYPEDEF) + 1);
            loadFromStringMsg(header_text, msg);
            unsigned char* res =  getMD5Msg(msg);
            cRosMD5Readable(res, buffer);
            dynStringPushBackStr(buffer," ");
            dynStringPushBackStr(buffer,fields_it->name);
            dynStringPushBackStr(buffer,"\n");
        }
        else
        {
            DynString filename_dep;
            dynStringInit(&filename_dep);
            dynStringPushBackStr(&filename_dep,msg->root_dir);
            dynStringPushBackStr(&filename_dep,"/");
            dynStringPushBackStr(&filename_dep,base_msg_type(fields_it->type_s));
            dynStringPushBackStr(&filename_dep,".msg");

            cRosMessage msg;
            cRosMessageInit(&msg);
            cRosMessageBuild(&msg,filename_dep.data);
            char *md5sum = calloc(strlen(msg.md5sum)+1,sizeof(char));
            strcpy(md5sum,msg.md5sum);
            cRosMessageRelease(&msg);

            dynStringPushBackStr(buffer, md5sum);
            dynStringPushBackStr(buffer," ");
            dynStringPushBackStr(buffer,fields_it->name);
            dynStringPushBackStr(buffer,"\n");
        }
        fields_it = fields_it->next;
    }
}

void cRosMD5Readable(unsigned char* data, DynString* output)
{
    char val[5];
    int i;
    for(i = 0; i < 16; i++)
    {
      snprintf(val, 4, "%02x", (unsigned char) data[i]);
        dynStringPushBackStr(output,val);
    }
}

void cRosMessageInit(cRosMessage *message)
{
    message->fields = NULL;
    message->n_fields = 0;
    message->msgDef = NULL;
    message->md5sum = (char*) malloc(33);// 32 chars + '\0';
    *message->md5sum = '\0';
}

cRosMessage * cRosMessageNew()
{
  cRosMessage *ret = (cRosMessage *)calloc(1, sizeof(cRosMessage));
  if (ret == NULL)
    return NULL;

  ret->md5sum = (char*) calloc(1, 33); // 32 chars + '\0';
  if (ret->md5sum == NULL)
  {
    free(ret);
    return NULL;
  }

  return ret;
}

void cRosMessageFieldInit(cRosMessageField *field)
{
  field->is_const = 0;
  field->name = NULL;
  field->type = CROS_CUSTOM_TYPE;
  field->type_s = NULL;
  field->size = -1;
  field->is_array = 0;
  field->is_fixed_array = 0;
  field->array_size = -1;
  field->array_capacity = -1;
  field->data.opaque = NULL;
}

int loadFromStringMsg(char* text, cRosMessageDef* msg)
{
//  Load message specification from a string:
//  types, names, constants

        char* new_line = NULL;
        char* new_line_saveptr = NULL;
        const char* delimiter = "\n";

        int txt_len = strlen(text);
        msg->plain_text = (char*) malloc(strlen(text)+1);
        memcpy(msg->plain_text,"\0",1);
        strcpy(msg->plain_text,text);
        int plain_txt_len = strlen(msg->plain_text);

        new_line = strtok_r(text, delimiter, &new_line_saveptr);
        while(new_line != NULL)
        {
            //printf("%s\n", new_line);
            char* iterator = new_line;
            int char_found = 0;
            int char_count = 0;

            //strip comments
            while(((char)*iterator) != '\0')
            {
                if(((char)*iterator) == *CHAR_COMMENT)
                {
                    char_found = 1;
                    break;
                }
                iterator++;
                char_count++;
            }

            // ignore empty lines
            if(char_found && char_count == 0)
            {
                new_line = strtok_r(NULL, delimiter, &new_line_saveptr);
                continue;
            }

            char* msg_entry = (char*) malloc(char_count + 1); // type/name
            strcpy(msg_entry, new_line);
            char* entry_type = NULL;
            char* entry_name = NULL;
            char* entry_const_val = NULL;

            char* msg_entry_itr = msg_entry;
            entry_type = msg_entry;

            while(*(msg_entry_itr++) != ' ');
            *(msg_entry_itr - 1) = '\0';

            entry_name = (char*) malloc(strlen(msg_entry_itr) + 1);
            strcpy(entry_name, msg_entry_itr);
            entry_type = base_msg_type(entry_type);

            char* filled_count = msg_entry_itr;

            while(*filled_count != '\0')
            {
                if(*filled_count == ' ')
                {
                    msg_entry_itr = filled_count + 1;
                    while(*msg_entry_itr != '\0' &&  *msg_entry_itr == ' ' )
                        msg_entry_itr++;
                    *filled_count = *msg_entry_itr;
                    if(*msg_entry_itr != '\0')
                        *msg_entry_itr = ' ';
                }
                else
                {
                    filled_count ++;
                }
            }

            if(!is_valid_msg_type(entry_type))
            {
                free(entry_type);
                free(msg_entry);
                return -1;
            }

            char* const_char_ptr = strpbrk(entry_name, CHAR_CONST);

            if( const_char_ptr != NULL)
            {
                if(strcmp(entry_type, "string") == 0)
                {
                    //  String constants
                    //  Strings contain anything to the right of the equals sign,
                    //  there are no comments allowed

                    char* entry_name_saveptr = NULL;
                    strtok_r(entry_name,CHAR_CONST,&entry_name_saveptr);
                    entry_const_val = strtok_r(NULL,CHAR_CONST,&entry_name_saveptr);
                }
                else
                {
                    //Number constants
                    char* entry_name_saveptr = NULL;
                    strtok_r(entry_name,CHAR_CONST,&entry_name_saveptr);
                    entry_const_val = strtok_r(NULL,CHAR_CONST,&entry_name_saveptr);
                }

                // TODO: try to cast number values

                msgConst* current = msg->constants;

                current->name = entry_name;
                current->type = getMessageType(entry_type);
                if (current->type == CROS_CUSTOM_TYPE)
                {
                  current->type_s = entry_type;
                  entry_type = NULL;
                }
                current->value = entry_const_val;
                current->next = (msgConst*)malloc(sizeof(msgConst));
                msgConst* next = current->next;
                initMsgConst(next);
                next->prev = current;
                msg->constants = next;
            }
            else
            {
                msgFieldDef* current = msg->fields;

                current->name = entry_name;
                current->type = getMessageType(entry_type);
                if (current->type == CROS_CUSTOM_TYPE)
                {
                  if(strstr(entry_type,"/") == NULL)
                  {
                      current->type_s = (char*) malloc(strlen(msg->package)+ 1 + strlen(entry_type) + 1);
                      memcpy(current->type_s, msg->package, strlen(msg->package)+ 1);
                      strcat(current->type_s,"/");
                      strcat(current->type_s,entry_type);
                  }
                  else
                  {
                    current->type_s = entry_type;
                    entry_type = NULL;
                  }
                }

                int array_size = -1;
                if(is_array_type(msg_entry, &array_size))
                {
                  current->is_array = 1;
                  current->array_size = array_size;
                }

                current->next = (msgFieldDef*)malloc(sizeof(msgFieldDef));
                msgFieldDef* next = current->next;
                initFieldDef(next);
                next->prev = current;
                msg->fields = next;
            }

            if (entry_type)
              free(entry_type);

            free(msg_entry);

            new_line = strtok_r(NULL, delimiter, &new_line_saveptr);
        }

        return EXIT_SUCCESS;
}

int loadFromFileMsg(char* filename, cRosMessageDef* msg)
{
    char* file_tokenized = (char*) calloc(strlen(filename)+1, sizeof(char));
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
        char *msg_text = malloc(fsize + 1);
        fread(msg_text, fsize, 1, f);
        fclose(f);

        msg_text[fsize] = '\0';
        char* tok = strtok(file_tokenized,"/.");

        while(tok != NULL)
        {
            if(strcmp(tok, "msg") != 0)
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

        msg->root_dir = (char*) malloc (strlen(file_tokenized)+1); msg->root_dir[0] = '\0';
        strcpy(msg->root_dir,file_tokenized);

        msg->package = (char*) malloc (strlen(token_pack)+1); msg->package[0] = '\0';
        strcpy(msg->package,token_pack);

        msg->name = (char*) malloc (strlen(token_name)+1); msg->name[0] = '\0';
        strcpy(msg->name,token_name);

        loadFromStringMsg(msg_text, msg);
        free(msg_text);
    }

    free(file_tokenized);
    return EXIT_SUCCESS;
}

void build_time_field(cRosMessageField* field)
{
  cRosMessage* time = calloc(1, sizeof(cRosMessage));
  cRosMessageInit(time);
  time->fields = (cRosMessageField**) calloc(2,sizeof(cRosMessageField*));
  time->n_fields = 2;
  cRosMessageField** fields_it = time->fields;

  //int32 sec
  cRosMessageField* sec = calloc(1, sizeof(cRosMessageField));
  cRosMessageFieldInit(sec);

  sec->name = "sec";
  sec->type = CROS_STD_MSGS_INT32;
  sec->size = 4;

  *fields_it = sec;
  fields_it++;

  //int32 nsec
  cRosMessageField* nsec = calloc(1, sizeof(cRosMessageField));
  cRosMessageFieldInit(nsec);

  nsec->name = "nsec";
  nsec->type = CROS_STD_MSGS_INT32;
  nsec->size = 4;

  *fields_it = nsec;

  field->size = 8;
  field->data.as_msg = time;
}

void build_header_field(cRosMessageField* field)
{
  cRosMessage* header = calloc(1, sizeof(cRosMessage));
  cRosMessageInit(header);
  header->fields = (cRosMessageField**) calloc(3,sizeof(cRosMessageField*));
  header->n_fields = 3;
  cRosMessageField** fields_it = header->fields;

  //uint32 seq
  cRosMessageField* sequence_id = calloc(1, sizeof(cRosMessageField));
  cRosMessageFieldInit(sequence_id);

  sequence_id->name = "seq";
  sequence_id->type = CROS_STD_MSGS_UINT32;
  sequence_id->size = 4;

  *fields_it = sequence_id;
  fields_it++;

  // time stamp
  // Two-integer timestamp that is expressed as:
  // * stamp.secs: seconds (stamp_secs) since epoch
  // * stamp.nsecs: nanoseconds since stamp_secs
  cRosMessageField* time_stamp = calloc(1, sizeof(cRosMessageField));
  cRosMessageFieldInit(time_stamp);
  time_stamp->type = CROS_STD_MSGS_UINT32;
  build_time_field(time_stamp);

  *fields_it = time_stamp;
  fields_it++;

  //string frame_id
  cRosMessageField* frame_id = calloc(1, sizeof(cRosMessageField));
  cRosMessageFieldInit(frame_id);
  frame_id->name = "frame_id";
  frame_id->type = CROS_STD_MSGS_STRING;

  *fields_it = frame_id;

  field->data.as_msg = header;
}
/*
void cRosMessageBuild(cRosMessage* message, const char* message_path)
{
  DynString output;
  dynStringInit(&output);

  cRosMessageDef* msg_def = (cRosMessageDef*) malloc(sizeof(cRosMessageDef));
  initCrosMsg(msg_def);
  char* message_path_cpy = calloc(strlen(message_path) + 1, sizeof(char));
  strcpy(message_path_cpy,message_path);
  loadFromFileMsg(message_path_cpy,msg_def);
  message->msgDef = msg_def;
  unsigned char* res = getMD5Msg(msg_def);
  cRosMD5Readable(res, &output);
  strcpy(message->md5sum,output.data);
  dynStringRelease(&output);

  msgFieldDef* field_def_itr =  msg_def->first_field;
  while(field_def_itr->next != NULL)
  {
    message->n_fields++;
    field_def_itr = field_def_itr->next;
  }

  message->fields = (cRosMessageField**) calloc(message->n_fields, sizeof(cRosMessageField*));

  field_def_itr =  msg_def->first_field;
  cRosMessageField** msg_field_itr = message->fields;

  while(field_def_itr->next != NULL)
  {
    *msg_field_itr = (cRosMessageField*) malloc(sizeof(cRosMessageField));
    cRosMessageField* field = *msg_field_itr;
    cRosMessageFieldInit(field);

    // TODO: add header flag
    field->is_builtin = is_builtin_type(field_def_itr->type);// || is_header_type(field_def_itr->type);
    field->name = (char*) calloc(strlen(field_def_itr->name)+1,sizeof(char));
    strcpy(field->name, field_def_itr->name);
    field->type = (char*) calloc(strlen(field_def_itr->type)+1,sizeof(char));
    strcpy(field->type, field_def_itr->type);

    if(field->is_builtin)
    {
      if(strcmp(field->type, "int8")==0 || strcmp(field->type, "uint8")==0 )
        field->size = sizeof(int8_t);
      else if(strcmp(field->type, "int16")==0 || strcmp(field->type, "uint16")==0 )
        field->size = sizeof(int16_t);
      else if(strcmp(field->type, "int32")==0 || strcmp(field->type, "uint32")==0 )
        field->size = sizeof(int32_t);
      else if(strcmp(field->type, "int64")==0 || strcmp(field->type, "uint64")==0 )
        field->size = sizeof(int64_t);
      else if(strcmp(field->type, "float32")==0)
        field->size = sizeof(float);
      else if(strcmp(field->type, "float64")==0)
        field->size = sizeof(double);
      else if(strcmp(field->type, "bool")==0)
        field->size = sizeof(unsigned char);
      else if(strcmp(field->type, "duration")==0)
        field->size = 1; //TODO: missing
      else if(strcmp(field->type, "string")==0)
        field->size = 0;
    }
    else if(is_header_type(field->type))
    {
      build_header_field(field);
    }
    else if(strcmp(field->type,"time") == 0)
    {
      build_time_field(field);
    }
    else
    {
      field->data.as_msg = malloc(sizeof(cRosMessage));
      cRosMessageInit((cRosMessage*)field->data.as_msg);
      char* path = calloc(strlen(msg_def->root_dir) +
                          strlen("/") +
                          strlen(field_def_itr->type) +
                          strlen(".msg") + 1, // '\0'
                          sizeof(char));
      strcat(path, msg_def->root_dir);
      strcat(path, "/");
      strcat(path, field_def_itr->type);
      strcat(path, ".msg");
      cRosMessageBuild((cRosMessage*) field->data.as_msg, path);
    }

    if(field_def_itr->is_array)
    {
      field->is_array = field_def_itr->is_array;
      field->array_max_size = field_def_itr->array_size;
      field->array_capacity = 1;
      field->data.as_array = calloc(1,field->size);
    }

    msg_field_itr++;
    field_def_itr = field_def_itr->next;
  }

}
*/

void cRosMessageBuild(cRosMessage* message, const char* message_path)
{
  DynString output;
  dynStringInit(&output);

  cRosMessageDef* msg_def = (cRosMessageDef*) malloc(sizeof(cRosMessageDef));
  initCrosMsg(msg_def);
  char* message_path_cpy = calloc(strlen(message_path) + 1, sizeof(char));
  strcpy(message_path_cpy,message_path);
  loadFromFileMsg(message_path_cpy,msg_def);
  message->msgDef = msg_def;
  cRosMessageBuildFromDef(message,msg_def);
}

void cRosMessageBuildFromDef(cRosMessage* message, cRosMessageDef* msg_def )
{
  DynString output;
  dynStringInit(&output);

  message->msgDef = msg_def;
  unsigned char* res = getMD5Msg(msg_def);
  cRosMD5Readable(res, &output);
  strcpy(message->md5sum,output.data);
  dynStringRelease(&output);

  msgFieldDef* field_def_itr =  msg_def->first_field;
  while(field_def_itr->next != NULL)
  {
    message->n_fields++;
    field_def_itr = field_def_itr->next;
  }

  message->fields = (cRosMessageField**) calloc(message->n_fields, sizeof(cRosMessageField*));

  field_def_itr =  msg_def->first_field;
  cRosMessageField** msg_field_itr = message->fields;

  while(field_def_itr->next != NULL)
  {
    *msg_field_itr = (cRosMessageField*) malloc(sizeof(cRosMessageField));
    cRosMessageField* field = *msg_field_itr;
    cRosMessageFieldInit(field);

    field->type = field_def_itr->type;
    field->name = (char*) calloc(strlen(field_def_itr->name)+1,sizeof(char));
    strcpy(field->name, field_def_itr->name);
    if (field_def_itr->type_s != NULL)
    {
      field->type_s = (char*) calloc(strlen(field_def_itr->type_s)+1,sizeof(char));
      strcpy(field->type_s, field_def_itr->type_s);
    }

    switch (field->type)
    {
      case CROS_STD_MSGS_INT8:
      case CROS_STD_MSGS_UINT8:
      case CROS_STD_MSGS_INT16:
      case CROS_STD_MSGS_UINT16:
      case CROS_STD_MSGS_INT32:
      case CROS_STD_MSGS_UINT32:
      case CROS_STD_MSGS_INT64:
      case CROS_STD_MSGS_UINT64:
      case CROS_STD_MSGS_FLOAT32:
      case CROS_STD_MSGS_FLOAT64:
      case CROS_STD_MSGS_BOOL:
      case CROS_STD_MSGS_CHAR:
      case CROS_STD_MSGS_BYTE:
      {
        field->size = getMessageTypeSizeOf(field->type);
        break;
      }
      case CROS_STD_MSGS_TIME:
      {
        build_time_field(field);
        break;
      }
      case CROS_STD_MSGS_DURATION:
      {
        // TODO
        break;
      }
      case CROS_STD_MSGS_HEADER:
      {
        build_header_field(field);
        break;
      }
      default:
      {
        field->data.as_msg = malloc(sizeof(cRosMessage));
        cRosMessageInit((cRosMessage*)field->data.as_msg);
        char* path = calloc(strlen(msg_def->root_dir) +
                            strlen("/") +
                            strlen(field_def_itr->type_s) +
                            strlen(".msg") + 1, // '\0'
                            sizeof(char));
        strcat(path, msg_def->root_dir);
        strcat(path, "/");
        strcat(path, field_def_itr->type_s);
        strcat(path, ".msg");
        cRosMessageBuild((cRosMessage*) field->data.as_msg, path);
        break;
      }
    }

    if(field_def_itr->is_array)
    {
      field->is_array = field_def_itr->is_array;
      if (field_def_itr->array_size == -1)
      {
        field->data.opaque = calloc(1,field->size);
        field->array_size = 0;
        field->array_capacity = 1;
      }
      else
      {
        field->data.opaque = calloc(field_def_itr->array_size,field->size);
        field->array_size = field_def_itr->array_size;
        field->array_capacity = field_def_itr->array_size;
        field->is_fixed_array = 1;
      }
    }

    msg_field_itr++;
    field_def_itr = field_def_itr->next;
  }
}

void cRosMessageRelease(cRosMessage *message)
{
  free(message->fields);
  message->fields = NULL;
  message->n_fields = 0;
  free(message->msgDef);
  message->msgDef = NULL;
}

void cRosMessageFree(cRosMessage *message)
{
  if (message == NULL)
    return;

  cRosMessageRelease(message);
  free(message);
}

cRosMessageField* cRosMessageGetField(cRosMessage *message, char *field_name)
{
  cRosMessageField* matching_field = NULL;
  cRosMessageField** fields_itr = message->fields;

  while(fields_itr != NULL && matching_field == NULL)
  {
    cRosMessageField* curr_field = *fields_itr;
    if(strcmp(curr_field->name, field_name) == 0)
    {
      matching_field = curr_field;
    }
    fields_itr++;
  }

  return matching_field;
}

int cRosMessageSetFieldValueString(cRosMessageField* field, const char* value)
{
  if (field->type != CROS_STD_MSGS_STRING)
    return -1;

  size_t len = strlen(value);
  free(field->data.as_string);
  field->data.as_string = calloc(strlen(value) + 1, sizeof(char));
  strcpy(field->data.as_string,value);
  field->size = (int)len;
  return 0;
}

int arrayFieldValuePushBack(cRosMessageField *field, const void* data, int element_size)
{
  if(!field->is_array || field->is_fixed_array)
    return -1;

  if(field->array_capacity == field->array_size)
  {
    void* new_location;
    new_location = realloc(field->data.opaque, 2 * field->array_capacity * element_size);
    if(new_location != NULL)
    {
      field->data.opaque = new_location;
      field->array_capacity *= 2;
    }
    else
    {
      return -1;
    }
  }

  memcpy(field->data.opaque + field->array_size * element_size, data, element_size);
  field->array_size ++;
  return 0;
}

int cRosMessageFieldArrayPushBackInt8(cRosMessageField *field, int8_t val)
{
  if(field->type != CROS_STD_MSGS_INT8)
    return -1;

  return arrayFieldValuePushBack(field, &val, getMessageTypeSizeOf(CROS_STD_MSGS_INT8));
}

int cRosMessageFieldArrayPushBackInt16(cRosMessageField *field, int16_t val)
{
  if(field->type != CROS_STD_MSGS_INT16)
    return -1;

  return arrayFieldValuePushBack(field, &val, getMessageTypeSizeOf(CROS_STD_MSGS_INT16));
}

int cRosMessageFieldArrayPushBackInt32(cRosMessageField *field, int32_t val)
{
  if(field->type != CROS_STD_MSGS_INT32)
    return -1;

  return arrayFieldValuePushBack(field, &val, getMessageTypeSizeOf(CROS_STD_MSGS_INT32));
}

int cRosMessageFieldArrayPushBackInt64(cRosMessageField *field, int64_t val)
{
  if(field->type != CROS_STD_MSGS_INT64)
    return -1;

  return arrayFieldValuePushBack(field, &val, getMessageTypeSizeOf(CROS_STD_MSGS_INT64));

}

int cRosMessageFieldArrayPushBackUInt8(cRosMessageField *field, uint8_t val)
{
  if(field->type != CROS_STD_MSGS_UINT8)
    return -1;

  return arrayFieldValuePushBack(field, &val, getMessageTypeSizeOf(CROS_STD_MSGS_UINT8));
}

int cRosMessageFieldArrayPushBackUInt16(cRosMessageField *field, uint16_t val)
{
  if(field->type != CROS_STD_MSGS_UINT16)
    return -1;

  return arrayFieldValuePushBack(field, &val, getMessageTypeSizeOf(CROS_STD_MSGS_UINT16));
}

int cRosMessageFieldArrayPushBackUInt32(cRosMessageField *field, uint32_t val)
{
  if(field->type != CROS_STD_MSGS_UINT32)
    return -1;

  return arrayFieldValuePushBack(field, &val, getMessageTypeSizeOf(CROS_STD_MSGS_UINT32));
}

int cRosMessageFieldArrayPushBackUint64(cRosMessageField *field, uint64_t val)
{
  if(field->type != CROS_STD_MSGS_UINT64)
    return -1;

  return arrayFieldValuePushBack(field, &val, getMessageTypeSizeOf(CROS_STD_MSGS_UINT64));
}

int cRosMessageFieldArrayPushBackFloat32(cRosMessageField *field, float val)
{
  if(field->type != CROS_STD_MSGS_FLOAT32)
    return -1;

  return arrayFieldValuePushBack(field, &val, getMessageTypeSizeOf(CROS_STD_MSGS_FLOAT32));
}

int cRosMessageFieldArrayPushBackFloat64(cRosMessageField *field, double val)
{
  if(field->type != CROS_STD_MSGS_FLOAT64)
    return -1;

  return arrayFieldValuePushBack(field, &val, getMessageTypeSizeOf(CROS_STD_MSGS_FLOAT64));
}

int cRosMessageFieldArrayPushBackString(cRosMessageField *field, const char* val)
{
  if(field->type != CROS_STD_MSGS_STRING)
    return -1;

  if(!field->is_array || field->is_fixed_array)
    return -1;

  if(field->array_capacity == field->array_size)
  {
    void* new_location;
    new_location = realloc(field->data.as_string, 2 * field->array_capacity * sizeof(char));
    if(new_location != NULL)
    {
      field->data.opaque = new_location;
      field->array_capacity *= 2;
    }
    else
    {
      return -1;
    }
  }

  char* element_val = (char*) calloc(strlen(val) + 1, sizeof(char));
  strcpy(element_val, val);
  field->data.as_string_array[field->array_size] = element_val;
  field->array_size ++;
  return 0;
}

int cRosMessageFieldArrayPushBackMsg(cRosMessageField *field, cRosMessage* msg)
{
  if(field->type != CROS_CUSTOM_TYPE)
    return -1;

  if(!field->is_array || field->is_fixed_array)
    return -1;

  size_t element_size = sizeof(cRosMessage*);

  if(field->array_capacity == field->array_size)
  {
    void* new_location;
    new_location = realloc(field->data.opaque, 2 * field->array_capacity * element_size);
    if(new_location != NULL)
    {
      field->data.opaque = new_location;
      field->array_capacity *= 2;
    }
    else
    {
      return -1;
    }
  }

  field->data.as_msg_array[field->array_size] = msg;
  field->array_size ++;
  return 0;
}

int cRosMessageFieldArrayAtInt8(cRosMessageField *field, int position, int8_t* val)
{
  if(field->type != CROS_STD_MSGS_INT8)
    return -1;

  return arrayFieldValueAt(field, position, sizeof(int8_t), val);
}

int cRosMessageFieldArrayAtInt16(cRosMessageField *field, int position, int16_t* val)
{
  if(field->type != CROS_STD_MSGS_INT16)
    return -1;

  return arrayFieldValueAt(field, position, sizeof(int16_t), val);
}

int cRosMessageFieldArrayAtInt32(cRosMessageField *field, int position, int32_t* val)
{
  if(field->type != CROS_STD_MSGS_INT32)
    return -1;

  return arrayFieldValueAt(field, position, sizeof(int32_t), val);
}

int cRosMessageFieldArrayAtInt64(cRosMessageField *field, int position, int64_t* val)
{
  if(field->type != CROS_STD_MSGS_INT64)
    return -1;

  return arrayFieldValueAt(field, position, sizeof(int64_t), val);
}

int cRosMessageFieldArrayAtUInt8(cRosMessageField *field, int position, uint8_t* val)
{
  if(field->type != CROS_STD_MSGS_UINT8)
    return -1;

  return arrayFieldValueAt(field, position, sizeof(uint8_t), val);
}

int cRosMessageFieldArrayAtUInt16(cRosMessageField *field, int position, uint16_t* val)
{
  if(field->type != CROS_STD_MSGS_UINT16)
    return -1;

  return arrayFieldValueAt(field, position, sizeof(uint16_t), val);
}

int cRosMessageFieldArrayAtUInt32(cRosMessageField *field, int position, uint32_t* val)
{
  if(field->type != CROS_STD_MSGS_UINT32)
    return -1;

  return arrayFieldValueAt(field, position, sizeof(uint32_t), val);
}

int cRosMessageFieldArrayAtUInt64(cRosMessageField *field, int position, uint64_t* val)
{
  if(field->type != CROS_STD_MSGS_UINT64)
    return -1;

  return arrayFieldValueAt(field, position, sizeof(uint64_t), val);
}

int cRosMessageFieldArrayAtFloat32(cRosMessageField *field, int position, float* val)
{
  if(field->type != CROS_STD_MSGS_FLOAT32)
    return -1;

  return arrayFieldValueAt(field, position, sizeof(float), val);
}

int cRosMessageFieldArrayAtFloat64(cRosMessageField *field, int position, double* val)
{
  if(field->type != CROS_STD_MSGS_FLOAT64)
    return -1;

  return arrayFieldValueAt(field, position, sizeof(double), val);
}

int cRosMessageFieldArrayAtMsg(cRosMessageField *field, int position, cRosMessage** val)
{
  cRosMessage* msgs =  field->data.as_msg_array[position];
  *val = msgs;
  return 0;
}

int cRosMessageFieldArrayAtString(cRosMessageField *field, int position, const char** val_ptr)
{
  if(field->type != CROS_STD_MSGS_STRING)
    return -1;

  char* str = (char*) field->data.as_string_array[position];
  *val_ptr = str;

  return 0;
}

int arrayFieldValueAt(cRosMessageField *field, int position, int element_size, void* data)
{
  if(!field->is_array)
    return -1;

  memcpy( data, field->data.opaque + position * element_size, element_size);

  return 0;
}

size_t cRosMessageFieldSize(cRosMessageField* field)
{
  size_t single_size = 0;

  if (is_builtin_type(field->type))
  {
    single_size = field->size;
  }
  else
  {
    single_size = cRosMessageSize(field->data.as_msg);
  }

  size_t ret = 0;
  if(field->is_array)
  {
    ret = single_size * field->array_size;
  }
  else
  {
    ret = single_size;
  }
  return ret;
}

size_t cRosMessageSize(cRosMessage* message)
{
    size_t ret = 0;
    cRosMessageField** fields_it = message->fields;
    int i;
    for( i = 0; i < message->n_fields; i++)
    {
      cRosMessageField* field = *fields_it;
      ret += cRosMessageFieldSize(field);
      fields_it++;
    }

    return ret;// + sizeof(uint32_t);
}

void cRosMessageSerialize(cRosMessage *message, DynBuffer* buffer)
{
  size_t it;
  for (it = 0; it < message->n_fields; it++)
  {
    cRosMessageField *field = message->fields[it];

    if(field->is_array && field->is_fixed_array)
      dynBufferPushBackInt32(buffer,field->array_size);

    switch (field->type)
    {
      case CROS_STD_MSGS_INT8:
      case CROS_STD_MSGS_UINT8:
      case CROS_STD_MSGS_INT16:
      case CROS_STD_MSGS_UINT16:
      case CROS_STD_MSGS_INT32:
      case CROS_STD_MSGS_UINT32:
      case CROS_STD_MSGS_INT64:
      case CROS_STD_MSGS_UINT64:
      case CROS_STD_MSGS_FLOAT32:
      case CROS_STD_MSGS_FLOAT64:
      case CROS_STD_MSGS_BOOL:
      case CROS_STD_MSGS_TIME:
      case CROS_STD_MSGS_DURATION:
      case CROS_STD_MSGS_HEADER:
      case CROS_STD_MSGS_CHAR:
      case CROS_STD_MSGS_BYTE:
      {
        size_t size = getMessageTypeSizeOf(field->type);
        if (field->is_array)
          size = size * field->array_size;
        dynBufferPushBackBuf(buffer, field->data.opaque, size);
        break;
      }
      case CROS_STD_MSGS_STRING:
      {
        if(field->is_array)
        {
          int i;
          for( i = 0; i < field->array_size; i++)
          {
            const char* val = NULL;
            cRosMessageFieldArrayAtString(field, i, &val);
            dynBufferPushBackInt32(buffer, strlen(val));
            dynBufferPushBackBuf(buffer, (unsigned char *)val, strlen(val));
          }
        }
        else
        {
          dynBufferPushBackBuf(buffer,(unsigned char*)field->data.as_string,field->size);
        }

        break;
      }
      default:
      {
        if(field->is_array)
        {
          int it2;
          for (it2 = 0; it2 < field->array_size; it2++)
          {
            cRosMessage* msg = NULL;
            cRosMessageFieldArrayAtMsg(field,it2,&msg);
            cRosMessageSerialize(msg, buffer);
          }
        }
        else
        {
          cRosMessageSerialize(field->data.as_msg, buffer);
        }
        break;
      }
    }
  }
}

void cRosMessageDeserialize(cRosMessage *message, DynBuffer* buffer)
{
  size_t it;
  for (it = 0; it < message->n_fields; it++)
  {
    cRosMessageField *field = message->fields[it];

    switch (field->type)
    {
      case CROS_STD_MSGS_INT8:
      case CROS_STD_MSGS_UINT8:
      case CROS_STD_MSGS_INT16:
      case CROS_STD_MSGS_UINT16:
      case CROS_STD_MSGS_INT32:
      case CROS_STD_MSGS_UINT32:
      case CROS_STD_MSGS_INT64:
      case CROS_STD_MSGS_UINT64:
      case CROS_STD_MSGS_FLOAT32:
      case CROS_STD_MSGS_FLOAT64:
      case CROS_STD_MSGS_BOOL:
      case CROS_STD_MSGS_TIME:
      case CROS_STD_MSGS_DURATION:
      case CROS_STD_MSGS_HEADER:
      case CROS_STD_MSGS_CHAR:
      case CROS_STD_MSGS_BYTE:
      {
        size_t size = getMessageTypeSizeOf(field->type);
        if (field->is_array)
        {
          size_t curr_data_size = field->array_size;
          if (field->is_fixed_array)
          {
            memcpy(field->data.opaque, dynBufferGetCurrentData(buffer), size * field->array_size);
            dynBufferMovePoseIndicator(buffer, size * field->array_size);
          }
          else
          {
            size_t array_size = *((uint32_t*)dynBufferGetCurrentData(buffer));
            dynBufferMovePoseIndicator(buffer, 4);
            int i;
            for(i = 0; i < array_size; i++)
            {
              arrayFieldValuePushBack(field, dynBufferGetCurrentData(buffer), size);
              dynBufferMovePoseIndicator(buffer, size);
            }
          }
        }
        else
        {
          memcpy(field->data.opaque, dynBufferGetCurrentData(buffer), size);
          dynBufferMovePoseIndicator(buffer, size);
        }

        break;
      }
      case CROS_STD_MSGS_STRING:
      {
        // CHECK-ME
        if (field->is_array)
        {
          size_t curr_data_size = field->array_size;
          if(!field->is_fixed_array)
          {
            curr_data_size = *((uint32_t*)dynBufferGetCurrentData(buffer));
            dynBufferMovePoseIndicator(buffer, 4);
          }

          int i;
          for(i = 0; i < curr_data_size; i++)
          {
            size_t element_size = *((uint32_t*)dynBufferGetCurrentData(buffer));
            dynBufferMovePoseIndicator(buffer, 4);
            cRosMessageFieldArrayPushBackString(field, (char *)dynBufferGetCurrentData(buffer));
            dynBufferMovePoseIndicator(buffer, element_size);
          }
        }
        else
        {
          size_t curr_data_size = *((uint32_t*)dynBufferGetCurrentData(buffer));
          dynBufferMovePoseIndicator(buffer, 4);
          field->data.as_string = (char*) calloc(curr_data_size + 1, sizeof(char));
          memcpy(field->data.as_string, dynBufferGetCurrentData(buffer), curr_data_size);
          dynBufferMovePoseIndicator(buffer, curr_data_size);
        }
        break;
      }
      default:
      {
        // CHECK-ME
        if (field->is_array)
        {
          size_t curr_data_size = field->array_size;
          if(!field->is_fixed_array)
          {
            curr_data_size = *((uint32_t*)dynBufferGetCurrentData(buffer));
            dynBufferMovePoseIndicator(buffer, 4);
          }

          int it2;
          for(it2 = 0; it2 < curr_data_size; it2++)
          {
            cRosMessage* msg = calloc(1,sizeof(cRosMessage));
            cRosMessageInit(msg);
            char* msgPath = calloc(
                            strlen(message->msgDef->root_dir) +
                            strlen("/") +
                            strlen(field->type_s) +
                            strlen(".msg") + 1,
                            sizeof(char));
            strcat(msgPath, message->msgDef->root_dir);
            strcat(msgPath, "/");
            strcat(msgPath, field->type_s);
            strcat(msgPath,".msg");
            cRosMessageBuild(msg,msgPath);
            cRosMessageDeserialize(msg, buffer);
            free(msgPath);
          }
        }
        else
        {
          cRosMessageDeserialize(field->data.as_msg, buffer);
        }
        break;
      }
    }
  }
}

int is_builtin_type(CrosMessageType type)
{
  switch(type)
  {
    case CROS_STD_MSGS_INT8:
    case CROS_STD_MSGS_UINT8:
    case CROS_STD_MSGS_INT16:
    case CROS_STD_MSGS_UINT16:
    case CROS_STD_MSGS_INT32:
    case CROS_STD_MSGS_UINT32:
    case CROS_STD_MSGS_INT64:
    case CROS_STD_MSGS_UINT64:
    case CROS_STD_MSGS_FLOAT32:
    case CROS_STD_MSGS_FLOAT64:
    case CROS_STD_MSGS_BOOL:
    case CROS_STD_MSGS_TIME:
    case CROS_STD_MSGS_DURATION:
    case CROS_STD_MSGS_CHAR:
    case CROS_STD_MSGS_BYTE:
    case CROS_STD_MSGS_STRING:
      return 1;
    default:
      return 0;
  }
}

size_t getMessageTypeSizeOf(CrosMessageType type)
{
  switch(type)
  {
    case CROS_STD_MSGS_INT8:
    case CROS_STD_MSGS_UINT8:
      return 1;
    case CROS_STD_MSGS_INT16:
    case CROS_STD_MSGS_UINT16:
      return 2;
    case CROS_STD_MSGS_INT32:
    case CROS_STD_MSGS_UINT32:
      return 4;
    case CROS_STD_MSGS_INT64:
    case CROS_STD_MSGS_UINT64:
      return 8;
    case CROS_STD_MSGS_FLOAT32:
      return 4;
    case CROS_STD_MSGS_FLOAT64:
      return 8;
    case CROS_STD_MSGS_BOOL:
      return 1;
    case CROS_STD_MSGS_TIME:
    case CROS_STD_MSGS_DURATION:
      return 8;
    // deprecated
    case CROS_STD_MSGS_CHAR:
    case CROS_STD_MSGS_BYTE:
      return 1;
    default:
      assert(0);
  }
}

CrosMessageType getMessageType(const char *type_s)
{
  if (strcmp(type_s, "int8") == 0 || strcmp(type_s, "std_msgs/Int8") == 0)
    return CROS_STD_MSGS_INT8;
  else if (strcmp(type_s, "uint8") == 0 || strcmp(type_s, "std_msgs/UInt8") == 0)
    return CROS_STD_MSGS_UINT8;
  else if (strcmp(type_s, "int16") == 0 || strcmp(type_s, "std_msgs/Int16") == 0)
    return CROS_STD_MSGS_INT16;
  else if (strcmp(type_s, "uint16") == 0 || strcmp(type_s, "std_msgs/UInt16") == 0)
    return CROS_STD_MSGS_UINT16;
  else if (strcmp(type_s, "int32") == 0 || strcmp(type_s, "std_msgs/Int32") == 0)
    return CROS_STD_MSGS_INT32;
  else if (strcmp(type_s, "uint32") == 0 || strcmp(type_s, "std_msgs/UInt32") == 0)
    return CROS_STD_MSGS_UINT32;
  else if (strcmp(type_s, "int64") == 0 || strcmp(type_s, "std_msgs/Int64") == 0)
    return CROS_STD_MSGS_INT64;
  else if (strcmp(type_s, "uint64") == 0 || strcmp(type_s, "std_msgs/UInt64") == 0)
    return CROS_STD_MSGS_UINT64;
  else if (strcmp(type_s, "float32") == 0 || strcmp(type_s, "std_msgs/Float32") == 0)
    return CROS_STD_MSGS_FLOAT32;
  else if (strcmp(type_s, "float64") == 0 || strcmp(type_s, "std_msgs/Float64") == 0)
    return CROS_STD_MSGS_FLOAT64;
  else if (strcmp(type_s, "string") == 0 || strcmp(type_s, "std_msgs/String") == 0)
    return CROS_STD_MSGS_STRING;
  else if (strcmp(type_s, "bool") == 0 || strcmp(type_s, "std_msgs/Bool") == 0)
    return CROS_STD_MSGS_BOOL;
  else if (strcmp(type_s, "time") == 0 || strcmp(type_s, "std_msgs/Time") == 0)
    return CROS_STD_MSGS_TIME;
  else if (strcmp(type_s, "duration") == 0 || strcmp(type_s, "std_msgs/Duration") == 0)
    return CROS_STD_MSGS_DURATION;
  else if (strcmp(type_s, "char") == 0 || strcmp(type_s, "std_msgs/Char") == 0)   // deprecated
    return CROS_STD_MSGS_CHAR;
  else if (strcmp(type_s, "byte") == 0 || strcmp(type_s, "std_msgs/Byte") == 0)   // deprecated
    return CROS_STD_MSGS_BYTE;
  else if (strcmp(type_s, "std_msgs/Header") == 0 || (strcmp(type_s, "Header") == 0)
      || (strcmp(type_s, "roslib/Header") == 0))
    return CROS_STD_MSGS_HEADER;
  else
    return CROS_CUSTOM_TYPE;
}

const char * getMessageTypeString(CrosMessageType type)
{
  switch(type)
  {
    case CROS_STD_MSGS_INT8:
      return "std_msgs/UInt8";
    case CROS_STD_MSGS_UINT8:
      return "std_msgs/UInt8";
    case CROS_STD_MSGS_INT16:
      return "std_msgs/Int16";
    case CROS_STD_MSGS_UINT16:
      return "std_msgs/UInt16";
    case CROS_STD_MSGS_INT32:
      return "std_msgs/Int32";
    case CROS_STD_MSGS_UINT32:
      return "std_msgs/UInt32";
    case CROS_STD_MSGS_INT64:
      return "std_msgs/Int64";
    case CROS_STD_MSGS_UINT64:
      return "std_msgs/UInt64";
    case CROS_STD_MSGS_FLOAT32:
      return "std_msgs/Float32";
    case CROS_STD_MSGS_FLOAT64:
      return "std_msgs/Float64";
    case CROS_STD_MSGS_STRING:
      return "std_msgs/String";
    case CROS_STD_MSGS_BOOL:
      return "std_msgs/Bool";
    case CROS_STD_MSGS_TIME:
      return "std_msgs/Time";
    case CROS_STD_MSGS_DURATION:
      return "std_msgs/Duration";
    case CROS_STD_MSGS_CHAR:        // deprecated
      return "std_msgs/Char";
    case CROS_STD_MSGS_BYTE:        // deprecated
      return "std_msgs/Byte";
    case CROS_STD_MSGS_HEADER:
      return "std_msgs/Header";
    default:
      assert(0);
  }
}

const char * getMessageTypeDeclaration(CrosMessageType type)
{
  switch(type)
  {
    case CROS_STD_MSGS_INT8:
      return "int8";
    case CROS_STD_MSGS_UINT8:
      return "uint8";
    case CROS_STD_MSGS_INT16:
      return "int16";
    case CROS_STD_MSGS_UINT16:
      return "uint16";
    case CROS_STD_MSGS_INT32:
      return "int32";
    case CROS_STD_MSGS_UINT32:
      return "uint32";
    case CROS_STD_MSGS_INT64:
      return "int64";
    case CROS_STD_MSGS_UINT64:
      return "uint64";
    case CROS_STD_MSGS_FLOAT32:
      return "float32";
    case CROS_STD_MSGS_FLOAT64:
      return "float64";
    case CROS_STD_MSGS_STRING:
      return "string";
    case CROS_STD_MSGS_BOOL:
      return "bool";
    case CROS_STD_MSGS_TIME:
      return "time";
    case CROS_STD_MSGS_DURATION:
      return "duration";
    case CROS_STD_MSGS_CHAR:    // deprecated
      return "char";
    case CROS_STD_MSGS_BYTE:    // deprecated
      return "byte";
    case CROS_STD_MSGS_HEADER:
      return "header";
    default:
      assert(0);
  }
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
  dynBufferPushBackUint32( packet, header_out_len );

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
  dynBufferPushBackUint32( packet, header_out_len );

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
  dynBufferPushBackUint32( packet, 0 ); // Placehoder for packet size

  void* data_context = n->pubs[pub_idx].context;
  n->pubs[pub_idx].callback( packet, data_context);

  uint32_t size = (uint32_t)dynBufferGetSize(packet) - sizeof(uint32_t);
  memcpy(packet->data, &size, sizeof(uint32_t));
}
