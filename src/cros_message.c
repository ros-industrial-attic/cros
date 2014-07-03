#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <ctype.h>

#include "cros_message.h"
#include "tcpros_tags.h"
#include "cros_defs.h"
#include "md5.h"


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

int is_builtin_type(char* type)
{
    int n_elements = sizeof(PRIMITIVE_TYPES)/sizeof(char*);
    int i;
    for(i = 0; i < n_elements; i++)
    {
        if(strcmp(type,PRIMITIVE_TYPES[i]) == 0)
            return 1;
    }
    return 0;
}

int is_header_type(char* type)
{
    return (strcmp(type, "std_msgs/Header") == 0) ||
            (strcmp(type, "Header") == 0) ||
            (strcmp(type, "roslib/Header") == 0);
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
        char* type = base_msg_type(currentField->type);

        if(!is_builtin_type(type))
        {
            if(containsDep(msgDeps,type))
            {
                fields = fields->next;
                continue;
            }

            currentDep->msg = (cRosMessageDef*) malloc(sizeof(cRosMessageDef));
            // special mapping for header
            if(is_header_type(type))
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

                if(strstr(type,"/") == NULL)
                {
                    char* trail = msg->package;
                    currentDep->msg->name = (char*) malloc(strlen(trail) + strlen(type) + 1 + 1);
                    memcpy(currentDep->msg->name, trail, strlen(trail) + 1);
                    strcat(currentDep->msg->name, "/");
                    strcat(currentDep->msg->name, type);
                }
                else
                {

                    char* dep = (char*) malloc(strlen(type)+1);
                    memcpy(dep,type, strlen(type)+1);
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
    msg->constants->type = NULL;
    msg->constants->name = NULL;
    msg->constants->value = NULL;
    msg->constants->prev = NULL;
    msg->constants->next = NULL;
    msg->first_const = msg->constants;
    msg->fields = (msgFieldDef*) calloc(1, sizeof(msgFieldDef));
    initFieldDef(msg->fields);
    msg->fields->type = NULL;
    msg->fields->name = NULL;
    msg->fields->prev = NULL;
    msg->fields->next = NULL;
    msg->first_field = msg->fields;
    msg->name = NULL;
    msg->package = NULL;
    msg->plain_text = NULL;
    msg->root_dir = NULL;
}

void initCrosDep(msgDep* dep)
{
    dep->msg = NULL;
    dep->prev = NULL;
    dep->next = NULL;
}

void initFieldDef(msgFieldDef* field)
{
  field->array_size = 0;
  field->is_array = 0;
  field->name = NULL;
  field->type = NULL;
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
        dynStringPushBackStr(&buffer,const_it->type);
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
        char* type = base_msg_type(fields_it->type);

        if(is_builtin_type(type))
        {
          if(fields_it->is_array)
          {
            dynStringPushBackStr(&buffer,fields_it->type);
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
            dynStringPushBackStr(&buffer,fields_it->type);
          }

          dynStringPushBackStr(&buffer," ");
          dynStringPushBackStr(&buffer,fields_it->name);
          dynStringPushBackStr(&buffer,"\n");
        }
        else if(is_header_type(type))
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
            dynStringPushBackStr(&filename_dep,base_msg_type(fields_it->type));
            dynStringPushBackStr(&filename_dep,".msg");

            cRosMessage msg;
            cRosMessageInit(&msg);
            cRosMessageBuild(&msg,filename_dep.data);
            char *md5sum = calloc(strlen(msg.md5sum)+1,sizeof(char));
            strcpy(md5sum,msg.md5sum);
            cRosMessageFree(&msg);

            dynStringPushBackStr(&buffer, md5sum);
            dynStringPushBackStr(&buffer," ");
            dynStringPushBackStr(&buffer,fields_it->name);
            dynStringPushBackStr(&buffer,"\n");
        }
        free(type);
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
        dynStringPushBackStr(buffer,const_it->type);
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
            dynStringPushBackStr(buffer,fields_it->type);
            dynStringPushBackStr(buffer," ");
            dynStringPushBackStr(buffer,fields_it->name);
            dynStringPushBackStr(buffer,"\n");
        }
        else if(is_header_type(fields_it->type))
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
            dynStringPushBackStr(&filename_dep,base_msg_type(fields_it->type));
            dynStringPushBackStr(&filename_dep,".msg");

            cRosMessage msg;
            cRosMessageInit(&msg);
            cRosMessageBuild(&msg,filename_dep.data);
            char *md5sum = calloc(strlen(msg.md5sum)+1,sizeof(char));
            strcpy(md5sum,msg.md5sum);
            cRosMessageFree(&msg);

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
    *(message->md5sum) = '\0';
}

void cRosMessageFieldInit(cRosMessageField *field)
{
  field->is_array = 0;
  field->is_const = 0;
  field->name = NULL;
  field->type = NULL;
  field->is_builtin = 0;
  field->array_capacity = 0;
  field->array_max_size = 0;
  field->array_size = 0;
  field->array_data = NULL;
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
            char* entry_type = NULL;
            char* entry_name = NULL;
            char* entry_const_val = NULL;
            memcpy(msg_entry, new_line, char_count);
            msg_entry[char_count] = '\0';

            char* msg_entry_itr = msg_entry;
            entry_type = msg_entry_itr;

            while(*(msg_entry_itr++) != ' ');
            *(msg_entry_itr - 1) = '\0';

            entry_name = msg_entry_itr;
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
                return 0;

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
                current->type = entry_type;
                current->value = entry_const_val;
                current->next = (msgConst*)malloc(sizeof(msgConst));
                msgConst* next = current->next;
                next->name = NULL;
                next->type = NULL;
                next->value = NULL;
                next->prev = current;
                next->next = NULL;
                msg->constants = next;
            }
            else
            {
                msgFieldDef* current = msg->fields;

                current->name = entry_name;
                if(is_header_type(entry_type))
                {
                    current->type = (char*) malloc(strlen(HEADER_DEFAULT_TYPE)+1);
                    memcpy(current->type,HEADER_DEFAULT_TYPE,strlen(HEADER_DEFAULT_TYPE)+1);
                }
                else if(strstr(entry_type,"/") == NULL && !is_builtin_type(entry_type))
                {
                    current->type = (char*) malloc(strlen(msg->package)+ 1 + strlen(entry_type) + 1);
                    memcpy(current->type, msg->package, strlen(msg->package)+ 1);
                    strcat(current->type,"/");
                    strcat(current->type,entry_type);
                }
                else
                {
                    current->type = entry_type;
                }
                int array_size = -1;
                if(is_array_type(msg_entry, &array_size))
                {
                  current->is_array = 1;
                  current->array_size = array_size;
                }

                current->next = (msgFieldDef*)malloc(sizeof(msgFieldDef));
                initFieldDef(current->next);
                msgFieldDef* next = current->next;
                next->name = NULL;
                next->type = NULL;
                next->prev = current;
                next->next = NULL;
                msg->fields = next;
            }

            //TODO: mem leak!!!
            //free(msg_entry);
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

  sec->is_builtin = 1;
  sec->name = "sec";
  sec->type = "int32";
  sec->size = 4;

  *fields_it = sec;
  fields_it++;

  //int32 nsec
  cRosMessageField* nsec = calloc(1, sizeof(cRosMessageField));
  cRosMessageFieldInit(nsec);

  nsec->is_builtin = 1;
  nsec->name = "nsec";
  nsec->type = "int32";
  nsec->size = 4;

  *fields_it = nsec;

  field->size = 8;
  field->as_msg = time;
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
  sequence_id->type = "uint32";
  sequence_id->is_builtin = 1;
  sequence_id->size = 4;

  *fields_it = sequence_id;
  fields_it++;

  // time stamp
  // Two-integer timestamp that is expressed as:
  // * stamp.secs: seconds (stamp_secs) since epoch
  // * stamp.nsecs: nanoseconds since stamp_secs
  cRosMessageField* time_stamp = calloc(1, sizeof(cRosMessageField));
  cRosMessageFieldInit(time_stamp);
  time_stamp->type = "time";
  time_stamp->is_builtin = 1;
  build_time_field(time_stamp);

  *fields_it = time_stamp;
  fields_it++;

  //string frame_id
  cRosMessageField* frame_id = calloc(1, sizeof(cRosMessageField));
  cRosMessageFieldInit(frame_id);
  frame_id->is_builtin = 1;
  frame_id->name = "frame_id";
  frame_id->type = "string";

  *fields_it = frame_id;

  field->as_msg = header;
}

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
      field->as_msg = malloc(sizeof(cRosMessage));
      cRosMessageInit((cRosMessage*)field->as_msg);
      char* path = calloc(strlen(msg_def->root_dir) +
                          strlen("/") +
                          strlen(field_def_itr->type) +
                          strlen(".msg") + 1, // '\0'
                          sizeof(char));
      strcat(path, msg_def->root_dir);
      strcat(path, "/");
      strcat(path, field_def_itr->type);
      strcat(path, ".msg");
      cRosMessageBuild((cRosMessage*) field->as_msg, path);
    }

    if(field_def_itr->is_array)
    {
      field->is_array = field_def_itr->is_array;
      field->array_max_size = field_def_itr->array_size;
      field->array_capacity = 1;
      field->array_data = calloc(1,field->size);
    }

    msg_field_itr++;
    field_def_itr = field_def_itr->next;
  }

}

void cRosMessageFree(cRosMessage *message)
{
    free(message->fields);
    message->fields = NULL;
    message->n_fields = 0;
    free(message->msgDef);
    message->msgDef = NULL;
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

int cRosMessageSetFieldValueBool(cRosMessageField* field, unsigned char value)
{
  if(strcmp(field->type, "bool") != 0)
    return 0;

  field->as_bool = value;

  return 1;
}

int cRosMessageSetFieldValueInt(cRosMessageField* field, int value)
{
  if(strcmp(field->type, "int8") != 0 && strcmp(field->type, "uint8") != 0 &&
     strcmp(field->type, "int16") != 0 && strcmp(field->type, "uint16") != 0 &&
     strcmp(field->type, "int32") != 0 && strcmp(field->type, "uint32") != 0 &&
     strcmp(field->type, "int64") != 0 && strcmp(field->type, "uint64") != 0 )
    return 0;

  field->as_int = value;

  return 1;
}

int cRosMessageSetFieldValueDouble(cRosMessageField* field, double value)
{
  if(strcmp(field->type, "float32") != 0 && strcmp(field->type, "float64") != 0)
    return 0;

  field->as_double = value;

  return 1;
}

int cRosMessageSetFieldValueString(cRosMessageField* field, const char* value)
{
  if(strcmp(field->type, "string") != 0)
    return 0;

  free(field->as_string);
  field->as_string = calloc(strlen(value) + 1, sizeof(char));
  strcpy(field->as_string,value);
  return 1;
}

int cRosMessageSetFieldValueMsg(cRosMessageField* field, cRosMessage* value)
{
  if(field->is_builtin)
    return 0;

  field->as_msg = value;

  return 1;
}

int arrayFieldValuePushBack(cRosMessageField *field, void* data, int element_size)
{
  if(field->array_size == field->array_max_size)
  {
    return 0;
  }

  if(field->array_capacity == field->array_size)
  {
    void* new_location;
    new_location = realloc(field->array_data, 2 * field->array_capacity * element_size);
    if(new_location != NULL)
    {
      field->array_data = new_location;
      field->array_capacity *= 2;
    }
    else
    {
      return 0;
    }
  }
  unsigned char* vector = field->array_data;
  memcpy(vector + field->array_size * element_size, data, element_size);
  field->array_size ++;
  return 1;
}

int arrayFieldValueAt(cRosMessageField *field, int position, int element_size, void* data)
{
  memcpy( data, field->array_data + position * element_size, element_size);
  return 1;
}

int cRosMessageFieldArrayPushBackInt8(cRosMessageField *field, int8_t val)
{
  if(strcmp(field->type, "int8") != 0)
    return 0;

  return arrayFieldValuePushBack(field, &val, sizeof(int8_t));
}

int cRosMessageFieldArrayPushBackInt16(cRosMessageField *field, int16_t val)
{
  if(strcmp(field->type, "int16") != 0)
    return 0;

  return arrayFieldValuePushBack(field, &val, sizeof(int16_t));
}

int cRosMessageFieldArrayPushBackInt32(cRosMessageField *field, int32_t val)
{
  if(strcmp(field->type, "int32") != 0)
    return 0;

  return arrayFieldValuePushBack(field, &val, sizeof(int32_t));
}

int cRosMessageFieldArrayPushBackInt64(cRosMessageField *field, int64_t val)
{
  if(strcmp(field->type, "int64") != 0)
    return 0;

  return arrayFieldValuePushBack(field, &val, sizeof(int64_t));

}

int cRosMessageFieldArrayPushBackUint8(cRosMessageField *field, uint8_t val)
{
  if(strcmp(field->type, "uint8") != 0)
    return 0;

  return arrayFieldValuePushBack(field, &val, sizeof(uint8_t));

}

int cRosMessageFieldArrayPushBackUint16(cRosMessageField *field, uint16_t val)
{
  if(strcmp(field->type, "uint16") != 0)
    return 0;

  return arrayFieldValuePushBack(field, &val, sizeof(uint16_t));
}

int cRosMessageFieldArrayPushBackUint32(cRosMessageField *field, uint32_t val)
{
  if(strcmp(field->type, "uint32") != 0)
    return 0;

  return arrayFieldValuePushBack(field, &val, sizeof(uint32_t));
}

int cRosMessageFieldArrayPushBackUint64(cRosMessageField *field, uint64_t val)
{
  if(strcmp(field->type, "uint64") != 0)
    return 0;

  return arrayFieldValuePushBack(field, &val, sizeof(uint64_t));
}

int cRosMessageFieldArrayPushBackSingle(cRosMessageField *field, float val)
{
  if(strcmp(field->type, "float32") != 0)
    return 0;

  return arrayFieldValuePushBack(field, &val, sizeof(float));
}

int cRosMessageFieldArrayPushBackDouble(cRosMessageField *field, double val)
{
  if(strcmp(field->type, "float64") != 0)
    return 0;

  return arrayFieldValuePushBack(field, &val, sizeof(double));
}

int cRosMessageFieldArrayPushBackString(cRosMessageField *field, char* val)
{
  if(strcmp(field->type, "string") != 0)
    return 0;

  if(field->array_size == field->array_max_size)
  {
    return 0;
  }

  if(field->array_capacity == field->array_size)
  {
    void* new_location;
    new_location = realloc(field->array_data, 2 * field->array_capacity * sizeof(char*));
    if(new_location != NULL)
    {
      field->array_data = new_location;
      field->array_capacity *= 2;
    }
    else
    {
      return 0;
    }
  }
  unsigned char* vector = field->array_data;
  char* string_val = (char*) calloc(strlen(val) + 1, sizeof(char));
  strcpy(string_val, val);
  memcpy(vector + field->array_size * sizeof(char*), string_val, sizeof(char*));
  field->array_size ++;
  return 1;
}

int cRosMessageFieldArrayPushBackMsg(cRosMessageField *field, cRosMessage* msg)
{
  if(field->array_size == field->array_max_size)
  {
    return 0;
  }

  size_t element_size = sizeof(cRosMessage*);

  if(field->array_capacity == field->array_size)
  {
    void* new_location;
    new_location = realloc(field->array_data, 2 * field->array_capacity * element_size);
    if(new_location != NULL)
    {
      field->array_data = new_location;
      field->array_capacity *= 2;
    }
    else
    {
      return 0;
    }
  }
  cRosMessage** msg_array = field->array_data;
  msg_array[field->array_size] = msg;
  field->array_size ++;
  return 1;
}

int cRosMessageFieldArrayAtInt8(cRosMessageField *field, int position, int8_t* val)
{
  if(strcmp(field->type, "int8") != 0)
    return 0;

  return arrayFieldValueAt(field, position, sizeof(int8_t), val);
}

int cRosMessageFieldArrayAtInt16(cRosMessageField *field, int position, int16_t* val)
{
  if(strcmp(field->type, "int16") != 0)
    return 0;

  return arrayFieldValueAt(field, position, sizeof(int16_t), val);
}

int cRosMessageFieldArrayAtInt32(cRosMessageField *field, int position, int32_t* val)
{
  if(strcmp(field->type, "int32") != 0)
    return 0;

  return arrayFieldValueAt(field, position, sizeof(int32_t), val);
}

int cRosMessageFieldArrayAtInt64(cRosMessageField *field, int position, int64_t* val)
{
  if(strcmp(field->type, "int64") != 0)
    return 0;

  return arrayFieldValueAt(field, position, sizeof(int64_t), val);

}

int cRosMessageFieldArrayAtUint8(cRosMessageField *field, int position, uint8_t* val)
{
  if(strcmp(field->type, "uint8") != 0)
    return 0;

  return arrayFieldValueAt(field, position, sizeof(uint8_t), val);

}

int cRosMessageFieldArrayAtUint16(cRosMessageField *field, int position, uint16_t* val)
{
  if(strcmp(field->type, "uint16") != 0)
    return 0;

  return arrayFieldValueAt(field, position, sizeof(uint16_t), val);

}

int cRosMessageFieldArrayAtUint32(cRosMessageField *field, int position, uint32_t* val)
{
  if(strcmp(field->type, "uint32") != 0)
    return 0;

  return arrayFieldValueAt(field, position, sizeof(uint32_t), val);
}

int cRosMessageFieldArrayAtUint64(cRosMessageField *field, int position, uint64_t* val)
{
  if(strcmp(field->type, "uint64") != 0)
    return 0;

  return arrayFieldValueAt(field, position, sizeof(uint64_t), val);
}

int cRosMessageFieldArrayAtSingle(cRosMessageField *field, int position, float* val)
{
  if(strcmp(field->type, "float32") != 0)
    return 0;

  return arrayFieldValueAt(field, position, sizeof(float), &val);
}

int cRosMessageFieldArrayAtDouble(cRosMessageField *field, int position, double* val)
{
  if(strcmp(field->type, "float64") != 0)
    return 0;

  return arrayFieldValueAt(field, position, sizeof(double), &val);
}

int cRosMessageFieldArrayAtMsg(cRosMessageField *field, int position, cRosMessage** val)
{
  cRosMessage** msgs_array = (cRosMessage**) field->array_data;
  cRosMessage* msgs = (cRosMessage*) msgs_array[position];
  *val = msgs;
  return 1;
}

int cRosMessageFieldArrayAtString(cRosMessageField *field, int position, char** val_ptr)
{
  if(strcmp(field->type, "string") != 0)
    return 0;

  int i;
  char* data_it = field->array_data;

  for(i = 0; i < position; i++)
  {
    while(*(data_it++) == '\0');
  }

  *val_ptr = (char*)calloc(strlen(data_it) + 1, sizeof(char));
  strcpy(*val_ptr,data_it);

  return 1;
}

size_t cRosMessageFieldSize(cRosMessageField* field)
{
  size_t single_size = 0;

  if(field->is_builtin)
  {
    if(strcmp(field->type, "string") == 0)
    {
      if(field->as_string != NULL)
      {
        single_size = strlen(field->as_string);
      }
    }
    else
    {
      single_size = field->size;
    }
  }
  else
  {
    single_size = cRosMessageSize((cRosMessage*)field->as_msg);
  }

  size_t ret = 0;
  if(field->is_array)
  {
    ret = single_size * field->is_array * field->array_size;
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
  //dynBufferPushBackInt32(buffer, cRosMessageSize(message));
  size_t it;

  for (it = 0; it < message->n_fields; it++)
  {
    cRosMessageField *field = message->fields[it];

    if(field->is_builtin)
    {
      if(field->is_array)
      {
        if(field->array_max_size == 0)
        {
          dynBufferPushBackInt32(buffer,field->array_size);
        }

        if(strcmp(field->type, "int8")==0 ||
           strcmp(field->type, "uint8")==0 ||
           strcmp(field->type, "bool")==0  )
        {
          int i;
          for( i = 0; i < field->array_size; i++)
          {
            int8_t val;
            cRosMessageFieldArrayAtInt8(field,i,&val);
            dynBufferPushBackInt8(buffer,val);
          }
        }
        else if(strcmp(field->type, "int16")==0 || strcmp(field->type, "uint16")==0 )
        {
          int i;
          for( i = 0; i < field->array_size; i++)
          {
            int16_t val;
            cRosMessageFieldArrayAtInt16(field,i,&val);
            dynBufferPushBackInt16(buffer,val);
          }
        }
        else if(strcmp(field->type, "int32")==0 || strcmp(field->type, "uint32")==0 )
        {
          int i;
          for( i = 0; i < field->array_size; i++)
          {
            int32_t val;
            cRosMessageFieldArrayAtInt32(field,i,&val);
            dynBufferPushBackInt32(buffer,val);
          }
        }
        else if(strcmp(field->type, "int64")==0 || strcmp(field->type, "uint64")==0 )
        {
          int i;
          for( i = 0; i < field->array_size; i++)
          {
            int64_t val;
            cRosMessageFieldArrayAtInt64(field,i,&val);
            dynBufferPushBackInt64(buffer,val);
          }
        }
        else if(strcmp(field->type, "float32")==0)
        {
          int i;
          for( i = 0; i < field->array_size; i++)
          {
            float val;
            cRosMessageFieldArrayAtSingle(field,i,&val);
            dynBufferPushBackSingle(buffer,val);
          }
        }
        else if(strcmp(field->type, "float64")==0)
        {
          int i;
          for( i = 0; i < field->array_size; i++)
          {
            double val;
            cRosMessageFieldArrayAtDouble(field,i,&val);
            dynBufferPushBackSingle(buffer,val);
          }
        }
        else if(strcmp(field->type, "string")==0)
        {
          int i;
          for( i = 0; i < field->array_size; i++)
          {
            char* val = NULL;
            cRosMessageFieldArrayAtString(field, i, &val);
            dynBufferPushBackBuf(buffer, (const unsigned char*) val,strlen(val));
            free(val);
          }
        }
        //else if(strcmp(field->type, "duration")==0)
           //TODO: missing
      }
      else
      {
        if(strcmp(field->type, "int8")==0 || strcmp(field->type, "uint8")==0 )
        {
          dynBufferPushBackInt8(buffer,field->as_int);
        }
        else if(strcmp(field->type, "int16")==0 || strcmp(field->type, "uint16")==0 )
        {
          dynBufferPushBackInt16(buffer,field->as_int);
        }
        else if(strcmp(field->type, "int32")==0 || strcmp(field->type, "uint32")==0 )
        {
          dynBufferPushBackInt32(buffer,field->as_int);
        }
        else if(strcmp(field->type, "int64")==0 || strcmp(field->type, "uint64")==0 )
        {
          dynBufferPushBackInt64(buffer,field->as_int64);
        }
        else if(strcmp(field->type, "float32")==0)
        {
          dynBufferPushBackSingle(buffer,field->as_double);
        }
        else if(strcmp(field->type, "float64")==0)
        {
          dynBufferPushBackDouble(buffer,field->as_double);
        }
        else if(strcmp(field->type, "bool")==0)
        {
          dynBufferPushBackInt8(buffer,field->as_bool);
        }
        else if(strcmp(field->type, "string")==0)
        {
          dynBufferPushBackBuf(buffer,(unsigned char*)field->as_string,field->size);
        }
      }
    }
    else
    {
      if(field->is_array)
      {
        if(field->array_max_size == 0)
        {
          dynBufferPushBackInt32(buffer,field->array_size);
        }
        int i;
        for( i = 0; i < field->array_size; i++)
        {
          cRosMessage* msg = NULL;
          cRosMessageFieldArrayAtMsg(field,i,&msg);
          cRosMessageSerialize(msg, buffer);
        }
      }
      else
      {
        cRosMessageSerialize((cRosMessage*) field->as_msg, buffer);
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

    if(field->is_builtin)
    {
      if(strcmp(field->type, "int8")==0)
      {
        if(field->is_array)
        {
          size_t curr_data_size = field->array_max_size;

          if(curr_data_size == 0)
          {
            size_t field_size = *((uint32_t*)dynBufferGetCurrentData(buffer));
            dynBufferMovePoseIndicator(buffer, 4);
            int i;
            for(i = 0; i < field_size; i++)
            {
              cRosMessageFieldArrayPushBackInt8(field,*((int8_t*)dynBufferGetCurrentData(buffer)));
              dynBufferMovePoseIndicator(buffer, 1);
            }
          }
        }
        else
        {
          field->as_int = *((int8_t*)dynBufferGetCurrentData(buffer));
          dynBufferMovePoseIndicator(buffer, 1);
        }
      }
      else if(strcmp(field->type, "uint8")==0 || strcmp(field->type, "bool")==0)
      {
        if(field->is_array)
        {
          size_t curr_data_size = field->array_max_size;
          if(curr_data_size == 0)
          {
            curr_data_size = *((uint32_t*)dynBufferGetCurrentData(buffer));
            dynBufferMovePoseIndicator(buffer, 4);
          }

          int i;
          for(i = 0; i < curr_data_size; i++)
          {
            cRosMessageFieldArrayPushBackInt8(field,*((int8_t*)dynBufferGetCurrentData(buffer)));
            dynBufferMovePoseIndicator(buffer, 1);
          }
        }
        else
        {
          if(strcmp(field->type, "uint8")==0)
          {
            field->as_int = *((int8_t*)dynBufferGetCurrentData(buffer));
          }
          else
          {
            field->as_bool = *((int8_t*)dynBufferGetCurrentData(buffer));
          }
          dynBufferMovePoseIndicator(buffer, 1);
        }

      }
      else if(strcmp(field->type, "int16")==0)
      {
        if(field->is_array)
        {
          size_t curr_data_size = field->array_max_size;

          if(curr_data_size == 0)
          {
            curr_data_size = *((uint32_t*)dynBufferGetCurrentData(buffer));
            dynBufferMovePoseIndicator(buffer, 4);
          }

          int i;
          for(i = 0; i < curr_data_size; i++)
          {
            cRosMessageFieldArrayPushBackInt16(field,*((int16_t*)dynBufferGetCurrentData(buffer)));
            dynBufferMovePoseIndicator(buffer, 2);
          }

        }
        else
        {
          field->as_int = *((int16_t*)dynBufferGetCurrentData(buffer));
          dynBufferMovePoseIndicator(buffer, 2);
        }
      }
      else if(strcmp(field->type, "uint16")==0 )
      {
        if(field->is_array)
        {
          size_t curr_data_size = field->array_max_size;

          if(curr_data_size == 0)
          {
            curr_data_size = *((uint32_t*)dynBufferGetCurrentData(buffer));
            dynBufferMovePoseIndicator(buffer, 4);
          }

          int i;
          for(i = 0; i < curr_data_size; i++)
          {
            cRosMessageFieldArrayPushBackUint16(field,*((uint16_t*)dynBufferGetCurrentData(buffer)));
            dynBufferMovePoseIndicator(buffer, 2);
          }

        }
        else
        {
          field->as_int = *((uint16_t*)dynBufferGetCurrentData(buffer));
          dynBufferMovePoseIndicator(buffer, 2);
        }
      }
      else if(strcmp(field->type, "int32")==0)
      {
        if(field->is_array)
        {
          size_t curr_data_size = field->array_max_size;

          if(curr_data_size == 0)
          {
            curr_data_size = *((uint32_t*)dynBufferGetCurrentData(buffer));
            dynBufferMovePoseIndicator(buffer, 4);
          }

          int i;
          for(i = 0; i < curr_data_size; i++)
          {
            cRosMessageFieldArrayPushBackInt32(field,*((int32_t*)dynBufferGetCurrentData(buffer)));
            dynBufferMovePoseIndicator(buffer, 4);
          }

        }
        else
        {
          field->as_int = *((int32_t*)dynBufferGetCurrentData(buffer));
          dynBufferMovePoseIndicator(buffer, 4);
        }
      }
      else if(strcmp(field->type, "uint32")==0 )
      {
        if(field->is_array)
        {
          size_t curr_data_size = field->array_max_size;

          if(curr_data_size == 0)
          {
            curr_data_size = *((uint32_t*)dynBufferGetCurrentData(buffer));
            dynBufferMovePoseIndicator(buffer, 4);
          }

          int i;
          for(i = 0; i < curr_data_size; i++)
          {
            cRosMessageFieldArrayPushBackUint32(field,*((uint32_t*)dynBufferGetCurrentData(buffer)));
            dynBufferMovePoseIndicator(buffer, 4);
          }

        }
        else
        {
          field->as_int = *((uint32_t*)dynBufferGetCurrentData(buffer));
          dynBufferMovePoseIndicator(buffer, 4);
        }
      }
      else if(strcmp(field->type, "int64")==0)
      {
        if(field->is_array)
        {
          size_t curr_data_size = field->array_max_size;

          if(curr_data_size == 0)
          {
            curr_data_size = *((uint32_t*)dynBufferGetCurrentData(buffer));
            dynBufferMovePoseIndicator(buffer, 4);
          }

          int i;
          for(i = 0; i < curr_data_size; i++)
          {
            cRosMessageFieldArrayPushBackInt64(field,*((int64_t*)dynBufferGetCurrentData(buffer)));
            dynBufferMovePoseIndicator(buffer, 8);
          }

        }
        else
        {
          field->as_int64 = *((int64_t*)dynBufferGetCurrentData(buffer));
          dynBufferMovePoseIndicator(buffer, 8);
        }
      }
      else if(strcmp(field->type, "uint64")==0 )
      {
        if(field->is_array)
        {
          size_t curr_data_size = field->array_max_size;

          if(curr_data_size == 0)
          {
            curr_data_size = *((uint32_t*)dynBufferGetCurrentData(buffer));
            dynBufferMovePoseIndicator(buffer, 4);
          }

          int i;
          for(i = 0; i < curr_data_size; i++)
          {
            cRosMessageFieldArrayPushBackUint64(field,*((uint64_t*)dynBufferGetCurrentData(buffer)));
            dynBufferMovePoseIndicator(buffer, 8);
          }
        }
        else
        {
          field->as_int64 = *((uint64_t*)dynBufferGetCurrentData(buffer));
          dynBufferMovePoseIndicator(buffer, 8);
        }
      }
      else if(strcmp(field->type, "float32")==0)
      {
        if(field->is_array)
        {
          size_t curr_data_size = field->array_max_size;

          if(curr_data_size == 0)
          {
            curr_data_size = *((uint32_t*)dynBufferGetCurrentData(buffer));
            dynBufferMovePoseIndicator(buffer, 4);
          }

          int i;
          for(i = 0; i < curr_data_size; i++)
          {
            cRosMessageFieldArrayPushBackSingle(field,*((float*)dynBufferGetCurrentData(buffer)));
            dynBufferMovePoseIndicator(buffer, 4);
          }

        }
        else
        {
          field->as_float = *((float*)dynBufferGetCurrentData(buffer));
          dynBufferMovePoseIndicator(buffer, 4);
        }
      }
      else if(strcmp(field->type, "float64")==0)
      {
        if(field->is_array)
        {
          size_t curr_data_size = field->array_max_size;

          if(curr_data_size == 0)
          {
            curr_data_size = *((uint32_t*)dynBufferGetCurrentData(buffer));
            dynBufferMovePoseIndicator(buffer, 4);
          }

          int i;
          for(i = 0; i < curr_data_size; i++)
          {
            cRosMessageFieldArrayPushBackDouble(field,*((double*)dynBufferGetCurrentData(buffer)));
            dynBufferMovePoseIndicator(buffer, 8);
          }
        }
        else
        {
          field->as_double = *((double*)dynBufferGetCurrentData(buffer));
          dynBufferMovePoseIndicator(buffer, 8);
        }
      }
      else if(strcmp(field->type, "string")==0)
      {
        if(field->is_array)
        {
          size_t curr_data_size = field->array_max_size;

          if(curr_data_size == 0)
          {
            curr_data_size = *((uint32_t*)dynBufferGetCurrentData(buffer));
            dynBufferMovePoseIndicator(buffer, 4);
          }

          int i;
          for(i = 0; i < curr_data_size; i++)
          {
            size_t element_size = *((uint32_t*)dynBufferGetCurrentData(buffer));
            dynBufferMovePoseIndicator(buffer, 4);
            char* element_val = (char*) calloc(element_size + 1,sizeof(char));
            memcpy(element_val,dynBufferGetCurrentData(buffer),element_size);
            dynBufferMovePoseIndicator(buffer, element_size);
            cRosMessageFieldArrayPushBackString(field, element_val);
            free(element_val);
          }
        }
        else
        {
          size_t curr_data_size = *((uint32_t*)dynBufferGetCurrentData(buffer));
          dynBufferMovePoseIndicator(buffer, 4);
          field->as_string = (char*) calloc(curr_data_size + 1,sizeof(char));
          memcpy(field->as_string,dynBufferGetCurrentData(buffer),curr_data_size);
          dynBufferMovePoseIndicator(buffer, curr_data_size);
        }
      }
    }
    else
    {
      if(field->is_array)
      {
        size_t curr_data_size = field->array_max_size;

        if(curr_data_size == 0)
        {
          curr_data_size = *((uint32_t*)dynBufferGetCurrentData(buffer));
          dynBufferMovePoseIndicator(buffer, 4);
        }

        int i;
        for(i = 0; i < curr_data_size; i++)
        {
          cRosMessage* msg = calloc(1,sizeof(cRosMessage));
          cRosMessageInit(msg);
          char* msgPath = calloc(
                          strlen(message->msgDef->root_dir) +
                          strlen("/") +
                          strlen(field->type) +
                          strlen(".msg") + 1,
                          sizeof(char));
          strcat(msgPath, message->msgDef->root_dir);
          strcat(msgPath, "/");
          strcat(msgPath, field->type);
          strcat(msgPath,".msg");
          cRosMessageBuild(msg,msgPath);
          cRosMessageDeserialize(msg, buffer);
          free(msgPath);
        }
      }
      else
      {
        cRosMessageDeserialize((cRosMessage*) field->as_msg, buffer);
      }
    }
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
