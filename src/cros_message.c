#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <ctype.h>
#include <assert.h>

#include "cros_message.h"
#include "cros_message_internal.h"
#include "tcpros_tags.h"
#include "cros_defs.h"
#include "md5.h"

static void * arrayFieldValueAt(cRosMessageField *field, int position, size_t element_size);
static const char * getMessageTypeDeclarationConst(msgConst *msgConst);
static const char * getMessageTypeDeclarationField(msgFieldDef *fieldDef);

char* base_msg_type(const char* type)
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
    const char* iterator = type;

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
  char* array_size;
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
  char *endptr;
  long int res = strtol(array_size, &endptr, 10);
  if (endptr == array_size)
    *size = -1;
  else
    *size = (int)res;

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

        if(!isBuiltinMessageType(currentField->type))
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

unsigned char* getMD5Msg(cRosMessageDef* msg)
{
    DynString buffer;
    unsigned char* result = (unsigned char*) malloc(16);
    int i;

    dynStringInit(&buffer);
    msgConst* const_it = msg->first_const;

    while(const_it->next != NULL)
    {
        const char *type_decl = getMessageTypeDeclarationConst(const_it);
        dynStringPushBackStr(&buffer,type_decl);
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
        const char *type_decl = getMessageTypeDeclarationField(fields_it);

        if(isBuiltinMessageType(fields_it->type))
        {
          if(fields_it->is_array)
          {
            dynStringPushBackStr(&buffer, type_decl);
            dynStringPushBackStr(&buffer,"[");
            if(fields_it->array_size != -1)
            {
              char num[15];
              sprintf(num, "%d",fields_it->array_size);
              dynStringPushBackStr(&buffer,num);
            }
            dynStringPushBackStr(&buffer,"]");
          }
          else
          {
            dynStringPushBackStr(&buffer, type_decl);
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
            dynStringPushBackStr(&filename_dep, type_decl);
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
        const char *type_decl = getMessageTypeDeclarationConst(const_it);
        dynStringPushBackStr(buffer,type_decl);
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
        const char *type_decl = getMessageTypeDeclarationField(fields_it);
        if(isBuiltinMessageType(fields_it->type))
        {
            dynStringPushBackStr(buffer, type_decl);
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
            dynStringPushBackStr(&filename_dep,base_msg_type(type_decl));
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

    message->md5sum = (char*) calloc(33,1); // 32 chars + '\0';
}

cRosMessage * cRosMessageNew()
{
  cRosMessage *ret = (cRosMessage *)calloc(1, sizeof(cRosMessage));

  if (ret == NULL)
    return NULL;

  cRosMessageInit(ret);

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
  memset(field->data.opaque, 0, sizeof(field->data.opaque));
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
            int comment_char_found = 0;
            int char_count = 0;

            //strip comments
            while(((char)*iterator) != '\0')
            {
                if(((char)*iterator) == *CHAR_COMMENT)
                {
                    comment_char_found = 1;
                    break;
                }
                char_count++;
                iterator++;
            }

            // ignore empty lines
            if(comment_char_found)
            {
              if(char_count == 0)
              {
                new_line = strtok_r(NULL, delimiter, &new_line_saveptr);
                continue;
              }
              else
              {
                //remove spaces among the message entry and the comment
                while(*(new_line + char_count - 1) == ' ')
                {
                 char_count--;
                }
              }
            }

            char* msg_entry = (char*) calloc(char_count + 1, sizeof(char)); // type/name
            strncpy(msg_entry, new_line,char_count);
            char* entry_type = NULL;
            char* entry_name = NULL;
            char* entry_const_val = NULL;

            char* msg_entry_itr = msg_entry;
            entry_type = msg_entry;

            while(*(msg_entry_itr++) != ' ');
            *(msg_entry_itr - 1) = '\0';

            entry_name = (char*) calloc(strlen(msg_entry_itr) + 1, sizeof(char));
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
                      current->type_s = (char*) calloc(strlen(msg->package)+ 1 + strlen(entry_type) + 1, sizeof(char));
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

                int array_size;
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

        msg->root_dir = (char*) calloc (strlen(file_tokenized)+1, sizeof(char)); msg->root_dir[0] = '\0';
        strcpy(msg->root_dir,file_tokenized);

        msg->package = (char*) calloc (strlen(token_pack)+1, sizeof(char)); msg->package[0] = '\0';
        strcpy(msg->package,token_pack);

        msg->name = (char*) calloc (strlen(token_name)+1, sizeof(char)); msg->name[0] = '\0';
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

  //int32 sec
  cRosMessageField* sec = cRosMessageFieldNew();

  sec->name = calloc(strlen("secs")+1, sizeof(char));
  strncpy(sec->name,"secs", strlen("secs"));
  sec->type = CROS_STD_MSGS_INT32;
  sec->size = 4;

  time->fields[0] = sec;

  //int32 nsec
  cRosMessageField* nsec = cRosMessageFieldNew();

  nsec->name = calloc(strlen("nsecs")+1, sizeof(char));
  strncpy(nsec->name,"nsecs", strlen("nsecs"));
  nsec->type = CROS_STD_MSGS_INT32;
  nsec->size = 4;

  time->fields[1] = nsec;

  field->size = 8;
  field->data.as_msg = time;
}

void build_header_field(cRosMessageField* field)
{
  cRosMessage* header = cRosMessageNew();
  header->fields = (cRosMessageField**) calloc(3,sizeof(cRosMessageField*));
  header->n_fields = 3;

  //uint32 seq
  cRosMessageField* sequence_id = cRosMessageFieldNew();

  sequence_id->name = calloc(strlen("seq")+1, sizeof(char));
  strncpy(sequence_id->name,"seq", strlen("seq"));
  sequence_id->type = CROS_STD_MSGS_UINT32;
  sequence_id->size = 4;

  header->fields[0] = sequence_id;

  // time stamp
  // Two-integer timestamp that is expressed as:
  // * stamp.secs: seconds (stamp_secs) since epoch
  // * stamp.nsecs: nanoseconds since stamp_secs
  cRosMessageField* time_stamp = cRosMessageFieldNew();
  time_stamp->name = calloc(strlen("stamp")+1, sizeof(char));
  strncpy(time_stamp->name,"stamp", strlen("stamp"));
  time_stamp->type = CROS_CUSTOM_TYPE;
  build_time_field(time_stamp);

  header->fields[1] = time_stamp;

  //string frame_id
  cRosMessageField* frame_id = cRosMessageFieldNew();
  frame_id->name = calloc(strlen("frame_id")+1, sizeof(char));
  strncpy(frame_id->name,"frame_id", strlen("frame_id"));
  frame_id->type = CROS_STD_MSGS_STRING;

  header->fields[2] = frame_id;

  field->data.as_msg = header;
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
      case CROS_STD_MSGS_STRING:
      {
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
      case CROS_CUSTOM_TYPE:
      {
        if(!field_def_itr->is_array)
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
        }
        break;
      }
    }


    if(field_def_itr->is_array)
    {
      field->is_array = field_def_itr->is_array;
      if (field_def_itr->array_size == -1)
      {
        if(field->type != CROS_CUSTOM_TYPE)
        {
          if(field->type == CROS_STD_MSGS_STRING)
          {
            field->data.as_array = calloc(1,sizeof(char*));
          }
          else
          {
            field->data.as_array = calloc(1,field->size);
          }
        }
        else
        {
          field->data.as_msg_array = calloc(1,sizeof(cRosMessage*));
        }
        field->array_size = 0;
        field->array_capacity = 1;
      }
      else
      {
        if(field->type != CROS_CUSTOM_TYPE)
        {
        field->data.as_array = calloc(field_def_itr->array_size,field->size);
        }
        else
        {
          field->data.as_msg_array = calloc(field_def_itr->array_size,sizeof(cRosMessage*));
        }
        field->array_size = field_def_itr->array_size;
        field->array_capacity = field_def_itr->array_size;
        field->is_fixed_array = 1;
      }
    }

    msg_field_itr++;
    field_def_itr = field_def_itr->next;
  }
}

void cRosMessageConstDefFree(msgConst* msg_const)
{
  free(msg_const->name);
  msg_const->name = NULL;
  free(msg_const->type_s);
  msg_const->type_s = NULL;
  free(msg_const->value);
  msg_const->value = NULL;
}

void cRosMessageFieldDefFree(msgFieldDef* msg_field)
{
  free(msg_field->name);
  msg_field->name = NULL;
  free(msg_field->type_s);
  msg_field->type_s = NULL;
}

void cRosMessageDefFree(cRosMessageDef *msgDef)
{
  free(msgDef->name);
  msgDef->name = NULL;
  free(msgDef->package);
  msgDef->package = NULL;
  free(msgDef->root_dir);
  msgDef->root_dir = NULL;
  free(msgDef->plain_text);
  msgDef->plain_text = NULL;

  msgConst* it_const = msgDef->first_const;
  while(it_const->next != NULL)
  {
    cRosMessageConstDefFree(it_const);
    it_const = it_const->next;
  }
  free(msgDef->first_const);
  msgDef->first_const = NULL;
  msgDef->constants = NULL;

  msgFieldDef* it_field = msgDef->first_field;
  while(it_field->next != NULL)
  {
    cRosMessageFieldDefFree(it_field);
    it_field = it_field->next;
  }
  free(msgDef->first_field);
  msgDef->first_field = NULL;
  msgDef->fields = NULL;
}

void cRosMessageRelease(cRosMessage *message)
{
  int i;

  for(i = 0; i < message->n_fields; i++)
  {
    cRosMessageFieldFree(message->fields[i]);
  }
  free(message->fields);
  message->fields = NULL;
  message->n_fields = 0;

  cRosMessageDefFree(message->msgDef);
  message->msgDef = NULL;

  free(message->md5sum);
  message->md5sum = NULL;
}

void cRosMessageFree(cRosMessage *message)
{
  if (message == NULL)
    return;

  cRosMessageRelease(message);
  free(message);
}

cRosMessageField * cRosMessageFieldNew()
{
  cRosMessageField *ret = (cRosMessageField *)calloc(1, sizeof(cRosMessageField));
  cRosMessageFieldInit(ret);
  return ret;
}

void cRosMessageFieldRelease(cRosMessageField *field)
{
  free(field->name);
  field->name = NULL;

  free(field->type_s);
  field->type_s = NULL;

  if(field->is_array)
  {
    if(field->type != CROS_CUSTOM_TYPE ||
    field->type != CROS_STD_MSGS_STRING ||
    field->type != CROS_STD_MSGS_TIME ||
    field->type != CROS_STD_MSGS_DURATION ||
    field->type != CROS_STD_MSGS_HEADER)
    {
      free(field->data.as_array);
      field->data.as_array = NULL;
    }
    else
    {
        int i;
        uint8_t* it;
        size_t data_size;
        for(i = 0; i < field->array_size; i++)
        {
          if(field->data.as_string_array)
          {
            free(field->data.as_string_array[i]);
          }
          else
          {
            free(field->data.as_msg_array[i]);
          }
        }
        free(field->data.as_array);
        field->data.as_array = NULL;
    }
   }
}

void cRosMessageFieldFree(cRosMessageField *field)
{
  if (field == NULL)
    return;

  cRosMessageFieldRelease(field);
  free(field);
}

cRosMessageField* cRosMessageGetField(cRosMessage *message, char *field_name)
{
  cRosMessageField* matching_field = NULL;

  int i;
  for(i = 0; i < message->n_fields; i++)
  {
    cRosMessageField* curr_field = message->fields[i];
    if(strcmp(curr_field->name, field_name) == 0)
    {
      matching_field = curr_field;
      break;
    }
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
    new_location = realloc(field->data.as_array, 2 * field->array_capacity * element_size);
    if(new_location != NULL)
    {
      field->data.as_array = new_location;
      field->array_capacity *= 2;
    }
    else
    {
      return -1;
    }
  }

  memcpy(field->data.as_array + field->array_size * element_size, data, element_size);
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
    new_location = realloc(field->data.as_string_array, 2 * field->array_capacity * sizeof(char*));
    if(new_location != NULL)
    {
      field->data.as_array = new_location;
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
    new_location = realloc(field->data.as_array, 2 * field->array_capacity * element_size);
    if(new_location != NULL)
    {
      field->data.as_array = new_location;
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

int8_t * cRosMessageFieldArrayAtInt8(cRosMessageField *field, int position)
{
  if(field->type != CROS_STD_MSGS_INT8)
    return NULL;

  return (int8_t *)arrayFieldValueAt(field, position, sizeof(int8_t));
}

int16_t * cRosMessageFieldArrayAtInt16(cRosMessageField *field, int position)
{
  if(field->type != CROS_STD_MSGS_INT16)
    return NULL;

  return (int16_t *)arrayFieldValueAt(field, position, sizeof(int16_t));
}

int32_t * cRosMessageFieldArrayAtInt32(cRosMessageField *field, int position)
{
  if(field->type != CROS_STD_MSGS_INT32)
    return NULL;

  return (int32_t *)arrayFieldValueAt(field, position, sizeof(int32_t));
}

int64_t * cRosMessageFieldArrayAtInt64(cRosMessageField *field, int position)
{
  if(field->type != CROS_STD_MSGS_INT64)
    return NULL;

  return (int64_t *)arrayFieldValueAt(field, position, sizeof(int64_t));
}

uint8_t * cRosMessageFieldArrayAtUInt8(cRosMessageField *field, int position)
{
  if(field->type != CROS_STD_MSGS_UINT8)
    return NULL;

  return (uint8_t *)arrayFieldValueAt(field, position, sizeof(uint8_t));
}

uint16_t * cRosMessageFieldArrayAtUInt16(cRosMessageField *field, int position)
{
  if(field->type != CROS_STD_MSGS_UINT16)
    return NULL;

  return (uint16_t *)arrayFieldValueAt(field, position, sizeof(uint16_t));
}

uint32_t * cRosMessageFieldArrayAtUInt32(cRosMessageField *field, int position)
{
  if(field->type != CROS_STD_MSGS_UINT32)
    return NULL;

  return (uint32_t *)arrayFieldValueAt(field, position, sizeof(uint32_t));
}

uint64_t * cRosMessageFieldArrayAtUInt64(cRosMessageField *field, int position)
{
  if(field->type != CROS_STD_MSGS_UINT64)
    return NULL;

  return (uint64_t *)arrayFieldValueAt(field, position, sizeof(uint64_t));
}

float * cRosMessageFieldArrayAtFloat32(cRosMessageField *field, int position)
{
  if(field->type != CROS_STD_MSGS_FLOAT32)
    return NULL;

  return (float *)arrayFieldValueAt(field, position, sizeof(float));
}

double * cRosMessageFieldArrayAtFloat64(cRosMessageField *field, int position)
{
  if(field->type != CROS_STD_MSGS_FLOAT64)
    return NULL;

  return (double *)arrayFieldValueAt(field, position, sizeof(double));
}

int cRosMessageFieldArrayAtMsgGet(cRosMessageField *field, int position, cRosMessage** ptr)
{
  if(field->type != CROS_CUSTOM_TYPE || !field->is_array)
    return -1;

  cRosMessage* msg =  field->data.as_msg_array[position];
  *ptr = msg;
  return 0;
}

int cRosMessageFieldArrayAtMsgSet(cRosMessageField *field, int position, cRosMessage* val)
{
  if(field->type != CROS_CUSTOM_TYPE || !field->is_array)
    return -1;

  field->data.as_msg_array[position] = val;
  return 0;
}

int cRosMessageFieldArrayAtStringGet(cRosMessageField *field, int position, const char** ptr)
{
  if(field->type != CROS_STD_MSGS_STRING || !field->is_array)
    return -1;

  char* str = (char*) field->data.as_string_array[position];
  *ptr = str;

  return 0;
}

int cRosMessageFieldArrayAtStringSet(cRosMessageField *field, int position, const char* val)
{
  if(field->type != CROS_STD_MSGS_STRING || !field->is_array)
    return -1;

  char *current = field->data.as_string_array[position];
  if (current != NULL)
    free(current);

  current = (char *)malloc(strlen(val) + 1);
  strcpy(current, val);

  field->data.as_string_array[position] = current;

  return 0;
}

int cRosMessageFieldArrayClear(cRosMessageField *field)
{
  if(!field->is_array || field->is_fixed_array)
    return -1;

  field->array_size = 0;

  return 0;
}

void * arrayFieldValueAt(cRosMessageField *field, int position, size_t size)
{
  if(!field->is_array)
    return NULL;

  if (position < 0 || position > field->array_size)
    return NULL;

  return field->data.as_array + (position * size);
}

size_t cRosMessageFieldSize(cRosMessageField* field)
{
  size_t single_size = 0;

  if (isBuiltinMessageType(field->type))
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

    if(field->is_array && !field->is_fixed_array)
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
      case CROS_STD_MSGS_CHAR:
      case CROS_STD_MSGS_BYTE:
      {
        size_t size = getMessageTypeSizeOf(field->type);
        if (field->is_array)
          dynBufferPushBackBuf(buffer, field->data.as_array, size * field->array_size);
        else
          dynBufferPushBackBuf(buffer, field->data.opaque, size);
        break;
      }
      case CROS_STD_MSGS_STRING:
      {
        // CHECK-ME
        if(field->is_array)
        {
          int i;
          for( i = 0; i < field->array_size; i++)
          {
            const char* val = NULL;
            cRosMessageFieldArrayAtStringGet(field, i, &val);
            dynBufferPushBackInt32(buffer, strlen(val));
            dynBufferPushBackBuf(buffer, (unsigned char *)val, strlen(val));
          }
        }
        else
        {
          if(field->size > 0)
          {
            dynBufferPushBackInt32(buffer, strlen(field->data.as_string));
            dynBufferPushBackBuf(buffer,(unsigned char*)field->data.as_string,strlen(field->data.as_string));
          }
          else
          {
            dynBufferPushBackInt32(buffer, 0);
          }
        }

        break;
      }
      default:
      {
        // CHECK-ME
        if(field->is_array)
        {
          int it2;
          for (it2 = 0; it2 < field->array_size; it2++)
          {
            cRosMessage* msg = NULL;
            cRosMessageFieldArrayAtMsgGet(field,it2,&msg);
            cRosMessageSerialize(msg, buffer);
          }
        }
        else
        {
          //uint32_t size = dynBufferGetSize(buffer);
          //dynBufferPushBackInt32(buffer,0);
          cRosMessageSerialize(field->data.as_msg, buffer);
          //uint32_t new_size = dynBufferGetSize(buffer);
          //uint32_t msg_size = new_size - size - sizeof(uint32_t);
          //*((uint32_t*) (buffer->data + size)) = msg_size;
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
            memcpy(field->data.as_array, dynBufferGetCurrentData(buffer), size * field->array_size);
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

const char * getMessageTypeDeclarationConst(msgConst *msgConst)
{
  if (msgConst->type_s == NULL)
    return getMessageTypeDeclaration(msgConst->type);
   else
     return msgConst->type_s;
}

const char * getMessageTypeDeclarationField(msgFieldDef *fieldDef)
{
  if (fieldDef->type_s == NULL)
    return getMessageTypeDeclaration(fieldDef->type);
   else
     return fieldDef->type_s;
}

int isBuiltinMessageType(CrosMessageType type)
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
