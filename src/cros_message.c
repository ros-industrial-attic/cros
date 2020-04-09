#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "cros_message.h"
#include "cros_message_internal.h"
#include "cros_defs.h"
#include "md5.h"

#ifdef _WIN32
#  define SPLIT_REGEX "\\."
#  define DIR_SEPARATOR_CHAR '\\'
#  define DIR_SEPARATOR_STR "\\"
// strtok_s is the Windows implementation of strtok_r
#  define strtok_r strtok_s
#else
#  define SPLIT_REGEX "/."
#  define DIR_SEPARATOR_CHAR '/'
#  define DIR_SEPARATOR_STR "/"
#endif


static void *arrayFieldValueAt(cRosMessageField *field, int position, size_t element_size);
static const char *getMessageTypeDeclarationConst(msgConst *msgConst);
static const char *getMessageTypeDeclarationField(msgFieldDef *fieldDef);

char *base_msg_type(const char* type)
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

    base = (char *)malloc(char_count + 1);
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
            (*base_itr == DIR_SEPARATOR_CHAR && slash_found))
        {
            free(base);
            return 0;
        }
        if(*base_itr == DIR_SEPARATOR_CHAR)
            slash_found = 1;
        base_itr++;
    }

    // TODO Check that the array index is a number

    free(base);
    return 1;
}

int is_array_type(char* type_statement, int* size)
{
    //@return: True if the name is a syntactically legal message type name
    //    @rtype: bool
  //if not x or len(x) != len(x.strip()):
  //    return False
  char* type_it = type_statement;
  char* array_size;
  char* num_start = NULL;
  int count = 0;

  while(*type_it != '\0' && *type_it != ']' && *type_it!= ' ')
  {
    if(num_start != NULL)
      count++;

    if(*type_it == '[')
      num_start = type_it;

    type_it++;
  }

  if(num_start == NULL)
    return 0;

  array_size = (char *)calloc(count + 1, sizeof(char));
  memcpy(array_size, num_start + 1, count);
  char *endptr;
  long int res = strtol(array_size, &endptr, 10);
  if (endptr == array_size)
    *size = -1;
  else
    *size = (int)res;
  free(array_size);

  return 1;
}

int containsDep(msgDep* iterator, char* depName)
{
    char* dep = (char *)malloc(strlen(depName)+1);
    memcpy(dep,depName, strlen(depName)+1);
    char* pack = dep;
    char* name = dep;
    while(*name != DIR_SEPARATOR_CHAR) name++;
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
cRosErrCodePack getDependenciesMsg(cRosMessageDef* msg, msgDep* msgDeps)
{
    cRosErrCodePack ret_err = CROS_SUCCESS_ERR_PACK; // Default return value

    //move on until you reach the head
    while(msgDeps->next != NULL) msgDeps = msgDeps->next;
    msgDep* currentDep = msgDeps ;

    msgFieldDef* fields = msg->first_field;

    while(fields->next != NULL && ret_err == CROS_SUCCESS_ERR_PACK)
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

            currentDep->msg = (cRosMessageDef *)malloc(sizeof(cRosMessageDef));
            // special mapping for header
            if(currentField->type == CROS_STD_MSGS_HEADER)
            {
                //have to re-names Header
                currentDep->msg->package = (char *)malloc(strlen(HEADER_DEFAULT_PACK) + 1);
                currentDep->msg->package[0] = '\0';
                strcpy(currentDep->msg->package,HEADER_DEFAULT_PACK);
                currentDep->msg->name = (char *)malloc(strlen(HEADER_DEFAULT_NAME) + 1);
                currentDep->msg->name[0] = '\0';
                strcpy(currentDep->msg->name,HEADER_DEFAULT_NAME);

                currentDep->msg->plain_text = (char *)malloc(strlen(HEADER_DEFAULT_TYPEDEF) + 1);
                memcpy(currentDep->msg->plain_text,"\0",1);
                strcpy(currentDep->msg->plain_text, HEADER_DEFAULT_TYPEDEF);
                msgDep *next = (msgDep *)malloc(sizeof(msgDep));
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
                    currentDep->msg->name = (char *)malloc(strlen(trail) + strlen(base_type) + 1 + 1);
                    memcpy(currentDep->msg->name, trail, strlen(trail) + 1);
                    strcat(currentDep->msg->name, "/");
                    strcat(currentDep->msg->name, base_type);
                }
                else
                {

                    char* dep = (char *)malloc(strlen(base_type)+1);
                    memcpy(dep,base_type, strlen(base_type)+1);
                    char* pack = dep;
                    char* name = dep;
                    while(*name != DIR_SEPARATOR_CHAR) name++;
                    *(name++) = '\0';
                    currentDep->msg->name = name;
                    currentDep->msg->package = pack;
                }

                currentDep->msg->fields = (msgFieldDef *)calloc(1, sizeof(msgFieldDef));
                initFieldDef(currentDep->msg->fields);
                currentDep->msg->first_field = currentDep->msg->fields;
                currentDep->msg->constants = (msgConst *)malloc(sizeof(msgConst));
                currentDep->msg->first_const = currentDep->msg->constants;

                char* path = (char *)malloc(strlen(msg->root_dir) + 1 +
                                            strlen(currentDep->msg->package) + 1 +
                                            strlen(currentDep->msg->name) + 4 + 1);
                memcpy(path, msg->root_dir, strlen(msg->root_dir) + 1);
                strcat(path, DIR_SEPARATOR_STR);
                strcat(path, currentDep->msg->package);
                strcat(path, DIR_SEPARATOR_STR);
                strcat(path,currentDep->msg->name);
                strcat(path,".msg");

                ret_err = loadFromFileMsg(path,currentDep->msg);
                if(ret_err == CROS_SUCCESS_ERR_PACK)
                {
                    msgDep *next = (msgDep *)malloc(sizeof(msgDep));
                    next->msg = NULL;
                    next->prev = currentDep;
                    currentDep->next = next;
                    currentDep = next;
                    ret_err = getDependenciesMsg(currentDep->prev->msg, currentDep);
                }
            }

            free(base_type);
        }
        fields = fields->next;
    }
    return ret_err;
}

//  Compute dependencies of the specified message file
cRosErrCodePack getFileDependenciesMsg(char* filename, cRosMessageDef* msg, msgDep* deps)
{
    cRosErrCodePack ret_err;
    ret_err = loadFromFileMsg(filename, msg);
    if(ret_err == CROS_SUCCESS_ERR_PACK)
        ret_err = getDependenciesMsg(msg,deps);
    return ret_err;
}

//  Compute full text of message, including text of embedded
//  types.  The text of the main msg is listed first. Embedded
//  msg files are denoted first by an 80-character '=' separator,
//  followed by a type declaration line,'MSG: pkg/type', followed by
//  the text of the embedded type.
char *computeFullTextMsg(cRosMessageDef* msg, msgDep* deps)
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
        full_size = strlen(deps->msg->plain_text) + strlen(separator) + strlen(msg_tag) + 3/*New lines*/;
        deps = deps->next;
    }

    //rollback
    while(deps->prev != NULL) deps = deps->prev;
    full_text = (char *)malloc(full_size + 1);
    memcpy(full_text,msg->plain_text,strlen(msg->plain_text) + 1);
    while(deps->next != NULL)
    {
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

cRosErrCodePack initCrosMsg(cRosMessageDef* msg)
{
  cRosErrCodePack ret_err;
  if(msg != NULL)
  {
    msg->constants = (msgConst *)malloc(sizeof(msgConst));
    msg->fields = (msgFieldDef *)calloc(1, sizeof(msgFieldDef));
    if(msg->constants != NULL && msg->fields != NULL)
    {
      initMsgConst(msg->constants);
      msg->first_const = msg->constants;
      initFieldDef(msg->fields);
      msg->first_field = msg->fields;
      msg->name = NULL;
      msg->package = NULL;
      msg->plain_text = NULL;
      msg->root_dir = NULL;
      ret_err = CROS_SUCCESS_ERR_PACK;
    }
    else
    {
      free(msg->constants);
      free(msg->fields);
      ret_err = CROS_MEM_ALLOC_ERR;
    }
  }
  else
    ret_err = CROS_BAD_PARAM_ERR;
  return ret_err;
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
  field->child_msg_def = NULL;
}

unsigned char *getMD5Msg(cRosMessageDef* msg)
{
    DynString buffer;
    cRosErrCodePack ret_err;
    unsigned char *result;

    result = (unsigned char *)malloc(16);
    if(result == NULL)
        return(NULL);

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

    ret_err = CROS_SUCCESS_ERR_PACK;
    while(ret_err == CROS_SUCCESS_ERR_PACK && fields_it->next != NULL)
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
            cRosMessageDef *msg_header;
            char *header_text;
            unsigned char *res;

            msg_header = (cRosMessageDef *)malloc(sizeof(cRosMessageDef));
            if(msg_header != NULL)
            {
                initCrosMsg(msg_header);
                header_text = (char *)malloc(strlen(HEADER_DEFAULT_TYPEDEF) + 1);
                if(header_text != NULL)
                {
                    memcpy(header_text,HEADER_DEFAULT_TYPEDEF,strlen(HEADER_DEFAULT_TYPEDEF) + 1);
                    loadFromStringMsg(header_text, msg_header);
                    free(header_text);
                    res =  getMD5Msg(msg_header);
                    cRosMessageDefFree(msg_header);
                    if(res != NULL)
                    {
                        cRosMD5Readable(res, &buffer);
                        free(res);
                        dynStringPushBackStr(&buffer," ");
                        dynStringPushBackStr(&buffer,fields_it->name);
                        dynStringPushBackStr(&buffer,"\n");
                    }
                    else
                    {
                        dynStringRelease(&buffer);
                        return NULL;
                    }
                }
                else
                {
                    cRosMessageDefFree(msg_header);
                    dynStringRelease(&buffer);
                    return NULL;
                }
            }
            else
            {
                dynStringRelease(&buffer);
                return NULL;
            }
        }
        else
        {
            cRosMessage *msg_fn;

            ret_err = cRosMessageBuildFromDef(&msg_fn, fields_it->child_msg_def); // it is faster than ret_err = cRosMessageNewBuild(msg->root_dir, type_decl, &msg_fn)
            if(ret_err == CROS_SUCCESS_ERR_PACK)
            {
                char *md5sum = strdup(msg_fn->md5sum);
                cRosMessageFree(msg_fn);
                if(md5sum != NULL)
                {
                    dynStringPushBackStr(&buffer, md5sum);
                    dynStringPushBackStr(&buffer," ");
                    dynStringPushBackStr(&buffer,fields_it->name);
                    dynStringPushBackStr(&buffer,"\n");
                    free(md5sum);
                }
                else
                    ret_err = CROS_MEM_ALLOC_ERR;
            }

        }
        fields_it = fields_it->next;
    }

    if(dynStringGetLen(&buffer) == 0)
        dynStringPushBackStr(&buffer,"\n");

    MD5_CTX md5_t;
    MD5_Init(&md5_t);
    MD5_Update(&md5_t, dynStringGetData(&buffer), dynStringGetLen(&buffer) - 1);
    MD5_Final(result, &md5_t);
    dynStringRelease(&buffer);

    return (ret_err == CROS_SUCCESS_ERR_PACK)? result : NULL;
}

cRosErrCodePack getMD5Txt(cRosMessageDef* msg_def, DynString* buffer)
{
    cRosErrCodePack ret_err;

    msgConst* const_it = msg_def->first_const;

    ret_err = CROS_SUCCESS_ERR_PACK;
    while(ret_err == CROS_SUCCESS_ERR_PACK && const_it->next != NULL)
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

    msgFieldDef* fields_it = msg_def->first_field;

    while(ret_err == CROS_SUCCESS_ERR_PACK && fields_it->next != NULL)
    {
        const char *type_decl = getMessageTypeDeclarationField(fields_it);

        if(isBuiltinMessageType(fields_it->type))
        {
            dynStringPushBackStr(buffer, type_decl);
            if(fields_it->is_array)
            {
              dynStringPushBackStr(buffer,"[");
              if(fields_it->array_size != -1)
              {
                char num[15];
                sprintf(num, "%d",fields_it->array_size);
                dynStringPushBackStr(buffer,num);
              }
              dynStringPushBackStr(buffer,"]");
            }

            dynStringPushBackStr(buffer," ");
            dynStringPushBackStr(buffer,fields_it->name);
            dynStringPushBackStr(buffer,"\n");
        }
        else if(fields_it->type == CROS_STD_MSGS_HEADER)
        {
            char *header_text;
            unsigned char *res;
            cRosMessageDef *msg_header;

            msg_header = (cRosMessageDef *)malloc(sizeof(cRosMessageDef));
            if(msg_header != NULL)
            {
                initCrosMsg(msg_header);
                header_text = (char *)malloc(strlen(HEADER_DEFAULT_TYPEDEF) + 1);
                if(header_text != NULL)
                {
                    memcpy(header_text,HEADER_DEFAULT_TYPEDEF,strlen(HEADER_DEFAULT_TYPEDEF) + 1);
                    loadFromStringMsg(header_text, msg_header);
                    free(header_text);
                    res =  getMD5Msg(msg_header);
                    if(res != NULL)
                    {
                        cRosMD5Readable(res, buffer);
                        free(res);
                        dynStringPushBackStr(buffer," ");
                        dynStringPushBackStr(buffer,fields_it->name);
                        dynStringPushBackStr(buffer,"\n");
                    }
                    else
                        ret_err = CROS_MEM_ALLOC_ERR;
                }
                else
                    ret_err = CROS_MEM_ALLOC_ERR;
                cRosMessageDefFree(msg_header);
            }
            else
                ret_err = CROS_MEM_ALLOC_ERR;
        }
        else
        {
            char *msg_name_str;
            cRosMessage *msg_fn;

            msg_name_str = base_msg_type(type_decl);
            ret_err = cRosMessageBuildFromDef(&msg_fn, fields_it->child_msg_def); // it is faster than ret_err = cRosMessageNewBuild(msg_def->root_dir, msg_name_str, &msg_fn)
            free(msg_name_str);
            if(ret_err == CROS_SUCCESS_ERR_PACK)
            {
                dynStringPushBackStr(buffer, msg_fn->md5sum);
                cRosMessageFree(msg_fn);

                dynStringPushBackStr(buffer," ");
                dynStringPushBackStr(buffer,fields_it->name);
                dynStringPushBackStr(buffer,"\n");
            }

        }
        fields_it = fields_it->next;
    }

    if(dynStringGetLen(buffer) == 0)
        dynStringPushBackStr(buffer,""); // Ensure that the returned DynString has allocated memory
    else
        dynStringReduce (buffer, 0, 1); // Remove the ending \n character which must not be processed by the MD5 algorithm

    return ret_err;
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

    message->md5sum = (char *)calloc(33, sizeof(char)); // 32 chars + '\0';
}

cRosMessage * cRosMessageNew(void)
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
    if(field != NULL)
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
}

//  Load message specification from a string:
//  types, names, constants
cRosErrCodePack loadFromStringMsg(char* text, cRosMessageDef* msg)
{
    char* new_line = NULL;
    char* new_line_saveptr = NULL;
    const char* delimiter = "\n";

    msg->plain_text = strdup(text);
    if(msg->plain_text == NULL)
        return CROS_MEM_ALLOC_ERR;

    new_line = strtok_r(text, delimiter, &new_line_saveptr);
    while(new_line != NULL)
    {
        char* iterator = new_line;
        int comment_char_found = 0;
        int char_count = 0; // Number of characters in the line before the comment

        //strip comments and new line
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
            //remove spaces between the message entry and the comment
            while(*(new_line + char_count - 1) == ' ')
            {
                char_count--;
            }
          }
        }

        char* msg_entry = (char *)calloc(char_count + 1, sizeof(char)); // type/name
        strncpy(msg_entry, new_line, char_count);
        char* entry_type;
        char* entry_name;

        char* msg_entry_itr = msg_entry;

        while(*msg_entry_itr != '\0' && *(msg_entry_itr++) != ' '); // Skip character at the beginning of the line until a space is found
        if(*(msg_entry_itr - 1) != ' ') // Error no space found
        {
            free(msg_entry);
            return CROS_FILE_ENTRY_NO_SEP_ERR;
        }
        *(msg_entry_itr - 1) = '\0'; // Convert the found space into a \0

        entry_name = (char *)calloc(strlen(msg_entry_itr) + 1, sizeof(char));
        strcpy(entry_name, msg_entry_itr);
        entry_type = base_msg_type(msg_entry); // Return the type of the entry in a new allocated memory string

        char* filled_count = msg_entry_itr;

        while(*filled_count != '\0') // ??? Any purpose here
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
            free(entry_name);
            free(entry_type);
            free(msg_entry);
            return CROS_FILE_ENTRY_TYPE_ERR;
        }

        char* const_char_ptr = strpbrk(entry_name, CHAR_CONST);

        if( const_char_ptr != NULL) // The entry is a constant definition
        {
            char* entry_const_val;
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
            current->value = strdup(entry_const_val); // strdup() will allocate a memory buffer that is independent of entry_name memory buffer so that it can freed independently as well
            current->next = (msgConst *)malloc(sizeof(msgConst));
            msgConst* next = current->next;
            initMsgConst(next);
            next->prev = current;
            msg->constants = next;
        }
        else // The entry is a data type declaration
        {
            msgFieldDef* current = msg->fields;

            current->name = entry_name;
            current->type = getMessageType(entry_type);
            if (current->type == CROS_CUSTOM_TYPE)
            {
              cRosErrCodePack ret_err;

              if(strstr(entry_type,"/") == NULL)
              {
                  current->type_s = (char *)calloc(strlen(msg->package)+ 1 + strlen(entry_type) + 1, sizeof(char));
                  memcpy(current->type_s, msg->package, strlen(msg->package)+ 1);
                  strcat(current->type_s,"/");
                  strcat(current->type_s,entry_type);
              }
              else
              {
                current->type_s = entry_type;
                entry_type = NULL; // Indicate that this buffer must not be freed later since it is used for the current msg def field
              }
              // Recursively load the msg definition of the custom type and store it into current->child_msg_def
              ret_err = cRosMessageDefBuild(&current->child_msg_def, msg->root_dir, current->type_s);
              if(ret_err != CROS_SUCCESS_ERR_PACK)
              {
                  free(current->type_s);
                  current->type_s = NULL;
                  free(entry_name);
                  current->name = NULL;
                  free(entry_type);
                  free(msg_entry);
                  return ret_err;
              }
            }

            int array_size;
            if(is_array_type(msg_entry, &array_size))
            {
              current->is_array = 1;
              current->array_size = array_size;
            }

            current->next = (msgFieldDef *)malloc(sizeof(msgFieldDef));
            msgFieldDef* next = current->next;
            initFieldDef(next);
            next->prev = current;
            msg->fields = next;
        }

        if (entry_type != NULL)
          free(entry_type);

        free(msg_entry);

        new_line = strtok_r(NULL, delimiter, &new_line_saveptr);
    }

    return CROS_SUCCESS_ERR_PACK;
}

cRosErrCodePack loadFromFileMsg(char* filename, cRosMessageDef* msg)
{
    cRosErrCodePack ret_err;
    size_t f_read_bytes;
    char* file_tokenized;
    FILE *f;
    char* token_pack = NULL;
    char* token_root = NULL;
    char* token_name = NULL;

    f = fopen(filename, "rb");
    if (f == NULL)
      return CROS_OPEN_MSG_FILE_ERR;

    // Load the entire message definition file in a buffer
    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    fseek(f, 0, SEEK_SET);
    char *msg_text = (char *)malloc(fsize + 1);
    if(msg_text == NULL)
    {
      fclose(f);
      return CROS_MEM_ALLOC_ERR;
    }

    f_read_bytes = fread(msg_text, 1, fsize, f);
    fclose(f);

    if(f_read_bytes != (size_t)fsize)
    {
      free(msg_text);
      return CROS_READ_MSG_FILE_ERR;
    }

    msg_text[fsize] = '\0';

    file_tokenized = (char *)calloc(strlen(filename)+1, sizeof(char));
    if(file_tokenized == NULL)
    {
      free(msg_text);
      return CROS_MEM_ALLOC_ERR;
    }
    strcpy(file_tokenized, filename);
    char* tok = strtok(file_tokenized,SPLIT_REGEX);

    while(tok != NULL)
    {
      if(strcmp(tok, "msg") != 0)
      {
        token_root = token_pack;
        token_pack = token_name;
        token_name = tok ;
      }
      tok = strtok(NULL,SPLIT_REGEX);
    }

    //build up the root path
    char* it = file_tokenized;
    while(it != token_root)
    {
      if(*it == '\0')
      *it=DIR_SEPARATOR_CHAR;
      it++;
    }

    msg->root_dir = (char*) malloc ((strlen(file_tokenized)+1) * sizeof(char)); // msg->root_dir[0] = '\0';
    msg->package = (char*) malloc ((strlen(token_pack)+1) * sizeof(char)); // msg->package[0] = '\0';
    msg->name = (char*) malloc ((strlen(token_name)+1) * sizeof(char)); // msg->name[0] = '\0';
    if(msg->root_dir != NULL && msg->package != NULL && msg->name != NULL)
    {
      strcpy(msg->root_dir,file_tokenized);
      strcpy(msg->package,token_pack);
      strcpy(msg->name,token_name);
      ret_err = loadFromStringMsg(msg_text, msg);
      ret_err = cRosAddErrCodeIfErr(ret_err, CROS_LOAD_MSG_FILE_ERR); // If an error occurred (the file could not be loaded), add more info (CROS_LOAD_MSG_FILE_ERR error code)
    }
    else
    {
      free(msg->root_dir);
      free(msg->package);
      free(msg->name);
      ret_err = CROS_MEM_ALLOC_ERR;
    }

    free(msg_text);
    free(file_tokenized);

    return ret_err;
}

cRosMessage *build_time_field(void)
{
  cRosMessage* time_msg = cRosMessageNew();
  if(time_msg != NULL)
  {
    time_msg->fields = (cRosMessageField **)calloc(2,sizeof(cRosMessageField*));
    if(time_msg->fields != NULL)
    {
      time_msg->n_fields = 2;

      cRosMessageField* sec = cRosMessageFieldNew();
      cRosMessageField* nsec = cRosMessageFieldNew();
      if(sec != NULL && nsec != NULL)
      {
        //int32 sec
        sec->name = strdup("secs");
        sec->type = CROS_STD_MSGS_UINT32;
        sec->size = getMessageTypeSizeOf(sec->type);
        time_msg->fields[0] = sec;

        //int32 nsec
        nsec->name = strdup("nsecs");
        nsec->type = CROS_STD_MSGS_UINT32;
        nsec->size = getMessageTypeSizeOf(nsec->type);
        time_msg->fields[1] = nsec;

        if(sec->name == NULL || nsec->name == NULL)
        {
          cRosMessageFree(time_msg);
          time_msg = NULL;
        }
      }
      else
      {
        cRosMessageFieldFree(sec);
        cRosMessageFieldFree(nsec);
        cRosMessageFree(time_msg);
        time_msg = NULL;
      }
    }
    else
    {
      cRosMessageFree(time_msg);
      time_msg = NULL;
    }
  }

  return time_msg;
}

cRosMessage *build_duration_field(void)
{
  cRosMessage* durat_msg = cRosMessageNew();
  if(durat_msg != NULL)
  {
    durat_msg->fields = (cRosMessageField **)calloc(2,sizeof(cRosMessageField*));
    if(durat_msg->fields != NULL)
    {
      durat_msg->n_fields = 2;

      cRosMessageField* sec = cRosMessageFieldNew();
      cRosMessageField* nsec = cRosMessageFieldNew();
      if(sec != NULL && nsec != NULL)
      {
        //int32 sec
        sec->name = strdup("secs");
        sec->type = CROS_STD_MSGS_INT32;
        sec->size = getMessageTypeSizeOf(sec->type);
        durat_msg->fields[0] = sec;

        //int32 nsec
        nsec->name = strdup("nsecs");
        nsec->type = CROS_STD_MSGS_INT32;
        nsec->size = getMessageTypeSizeOf(nsec->type);
        durat_msg->fields[1] = nsec;

        if(sec->name == NULL || nsec->name == NULL)
        {
          cRosMessageFree(durat_msg);
          durat_msg = NULL;
        }
      }
      else
      {
        cRosMessageFieldFree(sec);
        cRosMessageFieldFree(nsec);
        cRosMessageFree(durat_msg);
        durat_msg = NULL;
      }
    }
    else
    {
      cRosMessageFree(durat_msg);
      durat_msg = NULL;
    }
  }

  return durat_msg;
}

cRosMessage *build_header_field(void)
{
  cRosMessage* header_msg = cRosMessageNew();
  if(header_msg != NULL)
  {
    header_msg->fields = (cRosMessageField **)calloc(3,sizeof(cRosMessageField*));
    if(header_msg->fields != NULL)
    {
      header_msg->n_fields = 3;

      cRosMessageField* sequence_id = cRosMessageFieldNew();
      cRosMessageField* time_stamp = cRosMessageFieldNew();
      cRosMessageField* frame_id = cRosMessageFieldNew();
      if(sequence_id != NULL && time_stamp != NULL && frame_id != NULL)
      {
        //uint32 seq
        sequence_id->name = strdup("seq");
        sequence_id->type = CROS_STD_MSGS_UINT32;
        sequence_id->size = getMessageTypeSizeOf(sequence_id->type);
        header_msg->fields[0] = sequence_id;

        // time stamp
        // Two-integer timestamp that is expressed as:
        // * stamp.secs: seconds (stamp_secs) since epoch
        // * stamp.nsecs: nanoseconds since stamp_secs
        time_stamp->name = strdup("stamp");
        time_stamp->type = CROS_STD_MSGS_TIME;
        time_stamp->data.as_msg = build_time_field();
        header_msg->fields[1] = time_stamp;

        //string frame_id
        frame_id->name = strdup("frame_id");
        frame_id->type = CROS_STD_MSGS_STRING;
        header_msg->fields[2] = frame_id;
        if(sequence_id->name == NULL || time_stamp->name == NULL || frame_id->name == NULL)
        {
          cRosMessageFree(header_msg);
          header_msg = NULL;
        }
      }
      else
      {
        cRosMessageFieldFree(frame_id);
        cRosMessageFieldFree(time_stamp);
        cRosMessageFieldFree(sequence_id);
        cRosMessageFree(header_msg);
        header_msg = NULL;
      }
    }
    else
    {
      cRosMessageFree(header_msg);
      header_msg = NULL;
    }
  }

  return header_msg;
}

cRosErrCodePack cRosMessageDefBuild(cRosMessageDef **msg_def_ptr, const char *msg_root_dir, const char *msg_type)
{
  cRosErrCodePack ret;
  cRosMessageDef* msg_def = (cRosMessageDef *)malloc(sizeof(cRosMessageDef));
  if(msg_def == NULL)
    return CROS_MEM_ALLOC_ERR;

  ret = initCrosMsg(msg_def);
  if(ret == CROS_SUCCESS_ERR_PACK)
  {
    char* msg_file_path;
    if(msg_root_dir != NULL) // If msg_root_dir parameter is not NULL, the file path is composed, otherwise, msg_type is used directly as file path
    {
      msg_file_path = (char *)malloc(strlen(msg_root_dir) + strlen(DIR_SEPARATOR_STR) + strlen(msg_type) + strlen(FILEEXT_MSG) + 2);
      if(msg_file_path != NULL)
      {
        strcpy(msg_file_path, msg_root_dir);
        strcat(msg_file_path, DIR_SEPARATOR_STR);
        strcat(msg_file_path, msg_type);
        strcat(msg_file_path, ".");
        strcat(msg_file_path, FILEEXT_MSG);
      }
      else
        ret = CROS_MEM_ALLOC_ERR;
    }
    else
      msg_file_path = (char *)msg_type;
    if(ret == CROS_SUCCESS_ERR_PACK)
    {
      ret = loadFromFileMsg(msg_file_path, msg_def);
      if(msg_root_dir != NULL)
        free(msg_file_path);

      if (ret == CROS_SUCCESS_ERR_PACK)
        *msg_def_ptr = msg_def;
      else
        cRosMessageDefFree(msg_def);
    }
  }
  else
    free(msg_def);

  return ret;
}

int cRosMessageFieldCopy(cRosMessageField* new_field, cRosMessageField* orig_field)
{
  int ret;
  if(new_field != NULL && orig_field != NULL)
  {
    new_field->size = orig_field->size;
    new_field->is_const = orig_field->is_const;
    new_field->is_array = orig_field->is_array;
    new_field->is_fixed_array = orig_field->is_fixed_array;
    new_field->array_size = orig_field->array_size;
    new_field->array_capacity = orig_field->array_capacity;
    new_field->type = orig_field->type;
    new_field->type_s = (orig_field->type_s != NULL)? strdup(orig_field->type_s):NULL;
    new_field->name = (orig_field->name != NULL)? strdup(orig_field->name):NULL;
    if((new_field->type_s == NULL && orig_field->type_s != NULL) || (new_field->name == NULL && orig_field->name != NULL))
    {
      free(new_field->type_s);
      free(new_field->name);
      ret=-1;
    }
    else
    {
      ret=0; // Default return value
      if(!orig_field->is_array)
      {
        switch (orig_field->type)
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
            new_field->data = orig_field->data;
            break;
          }
          case CROS_STD_MSGS_STRING:
          {
            new_field->data.as_string = (orig_field->data.as_string != NULL)? strdup(orig_field->data.as_string) : NULL;
            if(orig_field->data.as_string != NULL && new_field->data.as_string == NULL)
              ret=-1; // Error allocating memory for string
            break;
          }
          case CROS_STD_MSGS_TIME:
          case CROS_STD_MSGS_DURATION:
          case CROS_STD_MSGS_HEADER:
          case CROS_CUSTOM_TYPE:
          {
            if(!orig_field->is_array)
            {
              new_field->data.as_msg = cRosMessageCopyWithoutDef(orig_field->data.as_msg);
              if(new_field->data.as_msg == NULL)
                ret=-1;
            }
            break;
          }
        }
      }
      else
      {
        // Allocate the array
        if(orig_field->type == CROS_STD_MSGS_STRING) // The array is encoded as a pointer to pointers
          new_field->data.as_array = calloc(orig_field->array_capacity, sizeof(char *)); // String pointers
        else if(orig_field->type == CROS_STD_MSGS_TIME || orig_field->type == CROS_STD_MSGS_DURATION ||
                orig_field->type == CROS_STD_MSGS_HEADER || orig_field->type == CROS_CUSTOM_TYPE)
          new_field->data.as_array = calloc(orig_field->array_capacity, sizeof(cRosMessage *)); // Message pointers
        else // The array is encoded as a pointer to elements
          new_field->data.as_array = calloc(orig_field->array_capacity, orig_field->size); // Element pointers
        if(new_field->data.as_array != NULL) // If the array was allocated, copy the elements
        {
          switch (orig_field->type)
          {
            case CROS_STD_MSGS_STRING:
            {
              int n_str;
              for(n_str=0;n_str<orig_field->array_size && ret==0;n_str++)
              {
                new_field->data.as_string_array[n_str] = (orig_field->data.as_string_array[n_str] != NULL)?strdup(orig_field->data.as_string_array[n_str]):NULL;
                if(new_field->data.as_string_array[n_str] == NULL && orig_field->data.as_string_array[n_str] != NULL)
                  ret=-1;
              }
              if(ret != 0) // Error allocating memory for strings: free previously allocated string memory
                for(n_str=0;n_str<orig_field->array_size && ret==0;n_str++)
                  free(new_field->data.as_string_array[n_str]);
              break;
            }
            case CROS_STD_MSGS_TIME:
            case CROS_STD_MSGS_DURATION:
            case CROS_STD_MSGS_HEADER:
            case CROS_CUSTOM_TYPE:
            {
              int n_msg;
              for(n_msg=0;n_msg<orig_field->array_size && ret==0;n_msg++)
              {
                new_field->data.as_msg_array[n_msg] = cRosMessageCopyWithoutDef(orig_field->data.as_msg_array[n_msg]);
                if(new_field->data.as_msg_array[n_msg] == NULL && orig_field->data.as_msg_array[n_msg] != NULL)
                  ret=-1;
              }
              if(ret != 0) // Error allocating memory for messages: free previously allocated message memory
                for(n_msg=0;n_msg<orig_field->array_size && ret==0;n_msg++)
                  cRosMessageFree(new_field->data.as_msg_array[n_msg]);
              break;
            }
            default:
            {
              memcpy(new_field->data.as_array, orig_field->data.as_array, orig_field->size * orig_field->array_size);
              break;
            }
          }
          if(ret != 0)
          {
            free(new_field->data.as_array);
          }
        }
        else
          ret=-1; // Error allocating the array
      }
      if(ret != 0) // Free memory to leave the message in a determined state
      {
        free(new_field->type_s);
        free(new_field->name);
      }
    }
  }
  else
    ret=-1;
  return ret;
}

void printNSpaces(int n_spaces)
{
  int spc_ind;
  for(spc_ind=0;spc_ind<n_spaces;spc_ind++)
    fprintf(cRosOutStreamGet()," ");
}

#define MAX_NUM_MSG_ELEM_PRINT 5 //! Maximum number of array elements that will be printed by cRosMessageFieldPrint()

// This function prints a field of one message (msg_field) to the output stream.
void cRosMessageFieldPrint(cRosMessageField *msg_field, int n_indent)
{
  FILE *msg_out = cRosOutStreamGet();
  printNSpaces(n_indent);
  if(msg_field != NULL)
  {
    int elem_ind;
    fprintf(msg_out,"nam:'%s' siz:%i cons:%i arr(is:%i isFix:%i siz:%i cap:%i) typ:%s (%s):", \
           (msg_field->name!=NULL)?msg_field->name:"NULL", msg_field->size, msg_field->is_const, \
           msg_field->is_array, msg_field->is_fixed_array, msg_field->array_size, msg_field->array_capacity, \
           (msg_field->type_s!=NULL)?msg_field->type_s:"NULL", getMessageTypeDeclaration(msg_field->type));
    switch(msg_field->type)
    {
      case CROS_STD_MSGS_INT8:
      {
        if(!msg_field->is_array)
          fprintf(msg_out,"%hi", (short)msg_field->data.as_int8);
        else
        {
          for(elem_ind=0;elem_ind < msg_field->array_size && elem_ind < MAX_NUM_MSG_ELEM_PRINT;elem_ind++)
            fprintf(msg_out,"%hi,", (short)msg_field->data.as_int8_array[elem_ind]);
          if(msg_field->array_size > MAX_NUM_MSG_ELEM_PRINT)
            fprintf(msg_out,"...");
        }
        break;
      }
      case CROS_STD_MSGS_UINT8:
      {
        if(!msg_field->is_array)
          fprintf(msg_out,"%hu", (unsigned short)msg_field->data.as_uint8);
        else
        {
          for(elem_ind=0;elem_ind < msg_field->array_size && elem_ind < MAX_NUM_MSG_ELEM_PRINT;elem_ind++)
            fprintf(msg_out,"%hu,", (unsigned short)msg_field->data.as_uint8_array[elem_ind]);
          if(msg_field->array_size > MAX_NUM_MSG_ELEM_PRINT)
            fprintf(msg_out,"...");
        }
        break;
      }
      case CROS_STD_MSGS_INT16:
      {
        if(!msg_field->is_array)
          fprintf(msg_out,"%hi", msg_field->data.as_int16);
        else
        {
          for(elem_ind=0;elem_ind < msg_field->array_size && elem_ind < MAX_NUM_MSG_ELEM_PRINT;elem_ind++)
            fprintf(msg_out,"%hi,", msg_field->data.as_int16_array[elem_ind]);
          if(msg_field->array_size > MAX_NUM_MSG_ELEM_PRINT)
            fprintf(msg_out,"...");
        }
        break;
      }
      case CROS_STD_MSGS_UINT16:
      {
        if(!msg_field->is_array)
          fprintf(msg_out,"%hu", msg_field->data.as_uint16);
        else
        {
          for(elem_ind=0;elem_ind < msg_field->array_size && elem_ind < MAX_NUM_MSG_ELEM_PRINT;elem_ind++)
            fprintf(msg_out,"%hu,", msg_field->data.as_uint16_array[elem_ind]);
          if(msg_field->array_size > MAX_NUM_MSG_ELEM_PRINT)
            fprintf(msg_out,"...");
        }
        break;
      }
      case CROS_STD_MSGS_INT32:
      {
        if(!msg_field->is_array)
          fprintf(msg_out,"%li", (long)msg_field->data.as_int32);
        else
        {
          for(elem_ind=0;elem_ind < msg_field->array_size && elem_ind < MAX_NUM_MSG_ELEM_PRINT;elem_ind++)
            fprintf(msg_out,"%li,", (long)msg_field->data.as_int32_array[elem_ind]);
          if(msg_field->array_size > MAX_NUM_MSG_ELEM_PRINT)
            fprintf(msg_out,"...");
        }
        break;
      }
      case CROS_STD_MSGS_UINT32:
      {
        if(!msg_field->is_array)
          fprintf(msg_out,"%lu", (long unsigned)msg_field->data.as_uint32);
        else
        {
          for(elem_ind=0;elem_ind < msg_field->array_size && elem_ind < MAX_NUM_MSG_ELEM_PRINT;elem_ind++)
            fprintf(msg_out,"%lu,", (long unsigned)msg_field->data.as_uint32_array[elem_ind]);
          if(msg_field->array_size > MAX_NUM_MSG_ELEM_PRINT)
            fprintf(msg_out,"...");
        }
        break;
      }
      case CROS_STD_MSGS_INT64:
      {
        if(!msg_field->is_array)
          fprintf(msg_out,"%lli", (long long)msg_field->data.as_int64);
        else
        {
          for(elem_ind=0;elem_ind < msg_field->array_size && elem_ind < MAX_NUM_MSG_ELEM_PRINT;elem_ind++)
            fprintf(msg_out,"%lli,", (long long)msg_field->data.as_int64_array[elem_ind]);
          if(msg_field->array_size > MAX_NUM_MSG_ELEM_PRINT)
            fprintf(msg_out,"...");
        }
        break;
      }
      case CROS_STD_MSGS_UINT64:
      {
        if(!msg_field->is_array)
          fprintf(msg_out,"%llu", (long long unsigned)msg_field->data.as_uint64);
        else
        {
          for(elem_ind=0;elem_ind < msg_field->array_size && elem_ind < MAX_NUM_MSG_ELEM_PRINT;elem_ind++)
            fprintf(msg_out,"%llu,", (long long unsigned)msg_field->data.as_uint64_array[elem_ind]);
          if(msg_field->array_size > MAX_NUM_MSG_ELEM_PRINT)
            fprintf(msg_out,"...");
        }
        break;
      }
      case CROS_STD_MSGS_FLOAT32:
      {
        if(!msg_field->is_array)
          fprintf(msg_out,"%f", msg_field->data.as_float32);
        else
        {
          for(elem_ind=0;elem_ind < msg_field->array_size && elem_ind < MAX_NUM_MSG_ELEM_PRINT;elem_ind++)
            fprintf(msg_out,"%f,", msg_field->data.as_float32_array[elem_ind]);
          if(msg_field->array_size > MAX_NUM_MSG_ELEM_PRINT)
            fprintf(msg_out,"...");
        }
        break;
      }
      case CROS_STD_MSGS_FLOAT64:
      {
        if(!msg_field->is_array)
          fprintf(msg_out,"%f", msg_field->data.as_float64);
        else
        {
          for(elem_ind=0;elem_ind < msg_field->array_size && elem_ind < MAX_NUM_MSG_ELEM_PRINT;elem_ind++)
            fprintf(msg_out,"%f,", msg_field->data.as_float64_array[elem_ind]);
          if(msg_field->array_size > MAX_NUM_MSG_ELEM_PRINT)
            fprintf(msg_out,"...");
        }
        break;
      }
      case CROS_STD_MSGS_BOOL:
      {
        if(!msg_field->is_array)
          fprintf(msg_out,"%hu", (unsigned short)msg_field->data.as_uint8);
        else
        {
          for(elem_ind=0;elem_ind < msg_field->array_size && elem_ind < MAX_NUM_MSG_ELEM_PRINT;elem_ind++)
            fprintf(msg_out,"%hu,", (unsigned short)msg_field->data.as_uint8_array[elem_ind]);
          if(msg_field->array_size > MAX_NUM_MSG_ELEM_PRINT)
            fprintf(msg_out,"...");
        }
        break;
      }
      case CROS_STD_MSGS_CHAR:
      {
        if(!msg_field->is_array)
          fprintf(msg_out,"%hu", (unsigned short)msg_field->data.as_uint8);
        else
        {
          for(elem_ind=0;elem_ind < msg_field->array_size && elem_ind < MAX_NUM_MSG_ELEM_PRINT;elem_ind++)
            fprintf(msg_out,"%hu,", (unsigned short)msg_field->data.as_uint8_array[elem_ind]);
          if(msg_field->array_size > MAX_NUM_MSG_ELEM_PRINT)
            fprintf(msg_out,"...");
        }
        break;
      }
      case CROS_STD_MSGS_BYTE:
      {
        if(!msg_field->is_array)
          fprintf(msg_out,"%hi", (short)msg_field->data.as_int8);
        else
        {
          for(elem_ind=0;elem_ind < msg_field->array_size && elem_ind < MAX_NUM_MSG_ELEM_PRINT;elem_ind++)
            fprintf(msg_out,"%hi,", (short)msg_field->data.as_int8_array[elem_ind]);
          if(msg_field->array_size > MAX_NUM_MSG_ELEM_PRINT)
            fprintf(msg_out,"...");
        }
        break;
      }
      case CROS_STD_MSGS_STRING:
      {
        if(!msg_field->is_array)
          fprintf(msg_out,"'%s'", (msg_field->data.as_string!=NULL)?msg_field->data.as_string:"NULL");
        else
        {
          if(msg_field->data.as_string_array != NULL)
          {
            for(elem_ind=0;elem_ind < msg_field->array_size && elem_ind < MAX_NUM_MSG_ELEM_PRINT;elem_ind++)
              fprintf(msg_out,"'%s',", (msg_field->data.as_string_array[elem_ind]!=NULL)?msg_field->data.as_string_array[elem_ind]:"NULL");
            if(msg_field->array_size > MAX_NUM_MSG_ELEM_PRINT)
              fprintf(msg_out,"...");
          }
          else
            fprintf(msg_out,"NULL");
        }
          break;
      }
      default:
      {
        break;
      }
    }
    // The value of these last types are printed after the switch statement because they share the same code
    if(msg_field->type == CROS_STD_MSGS_TIME || msg_field->type == CROS_STD_MSGS_DURATION ||
       msg_field->type == CROS_STD_MSGS_HEADER || msg_field->type == CROS_CUSTOM_TYPE)
    {
      if(!msg_field->is_array)
        cRosMessageFieldsPrint(msg_field->data.as_msg, n_indent);
      else
      {
        fprintf(msg_out,"\n");
        for(elem_ind=0;elem_ind < msg_field->array_size;elem_ind++)
          cRosMessageFieldsPrint(msg_field->data.as_msg_array[elem_ind], n_indent);
        if(msg_field->array_size > MAX_NUM_MSG_ELEM_PRINT)
            fprintf(msg_out,"...");
      }
    }
    else
      fprintf(msg_out,"\n");
  }
  else
    fprintf(msg_out,"Field NULL\n");
}

// This function prints the MD5 and all the fields (fields field struct) of one message (msg) to the output file stream.
void cRosMessageFieldsPrint(cRosMessage *msg, int n_indent)
{
  printNSpaces(n_indent);
  if(msg != NULL)
  {
    fprintf(cRosOutStreamGet(), "MsgAt 0x%p: MD5:'%s' N.Flds:%i Flds:%s\n", msg, (msg->md5sum != NULL)? msg->md5sum: "NULL", msg->n_fields, (msg->fields != NULL)?"":"NULL");
    if(msg->fields != NULL)
    {
      int field_ind;
      // Print all the filed in source message while no error occurs
      for(field_ind=0;field_ind<msg->n_fields;field_ind++)
        cRosMessageFieldPrint(msg->fields[field_ind], n_indent+1);
    }
  }
  else
    fprintf(cRosOutStreamGet(), "MsgAt NULL\n");
}

// This function copies the MD5 and all the fields (fields field struct) from one message (m_src) to another (m_dst).
// It if expected than the fields field of m_dst is NULL.
int cRosMessageFieldsCopy(cRosMessage *m_dst, cRosMessage *m_src)
{
  int ret;

  if(m_src != NULL && m_src != NULL)
  {
    if(m_src->md5sum != NULL) // If source message has a valid MD5 field, copy it
    {
      if(m_dst->md5sum != NULL)
      {
        strcpy(m_dst->md5sum, m_src->md5sum);
        ret=0;
      }
      else
      {
        m_dst->md5sum = strdup(m_src->md5sum);
        ret = (m_dst->md5sum != NULL)?0:-1;
      }
    }
    else
    {
      free(m_dst->md5sum);
      m_dst->md5sum=NULL;
      ret=0;
    }
  }
  else
    ret=-1;
  if(ret == 0) // If no error copying MD5 field, continue
  {
    // Remove previous fields from destination message
    cRosMessageFieldsFree(m_dst);
    // Create a new field array in destination message
    m_dst->fields = (cRosMessageField **)calloc(m_src->n_fields, sizeof(cRosMessageField*));
    if(m_dst->fields != NULL)
    {
      int field_ind;

      m_dst->n_fields = m_src->n_fields;
      // Copy all the filed in source message while no error occurs
      for(field_ind=0;field_ind<m_src->n_fields && ret==0;field_ind++)
      {
        cRosMessageField *new_field = cRosMessageFieldNew();
        if(new_field != NULL)
        {
          ret = cRosMessageFieldCopy(new_field, m_src->fields[field_ind]);
          if(ret == 0)
            m_dst->fields[field_ind] = new_field;
          else
            free(new_field);
        }
        else
          ret=-1;
      }
      if(ret != 0) // Error copying fields
      {
        // Destroy already created fields
        cRosMessageFieldsFree(m_dst);
      }
    }
    else
      ret=-1;
  }
  return ret;
}

cRosMessage *cRosMessageCopyWithoutDef(cRosMessage *m_src)
{
  cRosMessage *m_dst;
  if(m_src != NULL)
  {
    m_dst = cRosMessageNew();
    if(m_dst != NULL)
    {
      if(cRosMessageFieldsCopy(m_dst, m_src) != 0)
      {
        // Error copying fields: free the new message
        cRosMessageFree(m_dst);
        m_dst=NULL; // Return NULL (error indicator)
      }
    }
  }
  else
    m_dst = NULL;
  return m_dst;
}

cRosMessage *cRosMessageCopy(cRosMessage *m_src)
{
  cRosMessage *m_dst;
  if(m_src != NULL)
  {
    m_dst = cRosMessageNew();
    if(m_dst != NULL)
    {
      if(cRosMessageFieldsCopy(m_dst, m_src) == 0)
      {
        cRosErrCodePack ret_err;
        ret_err = cRosMessageDefCopy(&m_dst->msgDef, m_src->msgDef );
        if(ret_err != CROS_SUCCESS_ERR_PACK)
        {
          cRosMessageFree(m_dst);
          m_dst=NULL; // Return NULL (error indicator)
        }
      }
      else
      {
        // Error copying fields: free the new message
        cRosMessageFree(m_dst);
        m_dst=NULL; // Return NULL (error indicator)
      }
    }
  }
  else
    m_dst = NULL;
  return m_dst;
}

cRosErrCodePack cRosFieldDefCopy(msgFieldDef* new_field_def, msgFieldDef* orig_field_def )
{
  cRosErrCodePack ret;
  if(new_field_def != NULL && orig_field_def != NULL)
  {
    new_field_def->type = orig_field_def->type;
    new_field_def->is_array = orig_field_def->is_array;
    new_field_def->array_size = orig_field_def->array_size;
    new_field_def->next = NULL;
    new_field_def->prev = NULL;
    new_field_def->type_s = (orig_field_def->type_s != NULL)? strdup(orig_field_def->type_s):NULL;
    new_field_def->name = (orig_field_def->name != NULL)? strdup(orig_field_def->name):NULL;
    if((new_field_def->type_s == NULL && orig_field_def->type_s != NULL) || (new_field_def->name == NULL && orig_field_def->name != NULL))
      ret=CROS_MEM_ALLOC_ERR;
    else
      ret = cRosMessageDefCopy(&new_field_def->child_msg_def, orig_field_def->child_msg_def );

    if(ret != CROS_SUCCESS_ERR_PACK)
    {
      free(new_field_def->type_s);
      free(new_field_def->name);
      new_field_def->type_s = NULL;
      new_field_def->name = NULL;
    }
  }
  else
    ret=CROS_BAD_PARAM_ERR;
  return ret;
}

cRosErrCodePack cRosConstDefCopy(msgConst* new_const_def, msgConst* orig_const_def )
{
  cRosErrCodePack ret;
  if(new_const_def != NULL && orig_const_def != NULL)
  {
    new_const_def->type = orig_const_def->type;
    new_const_def->next = NULL;
    new_const_def->prev = NULL;
    new_const_def->type_s = (orig_const_def->type_s != NULL)? strdup(orig_const_def->type_s):NULL;
    new_const_def->name = (orig_const_def->name != NULL)? strdup(orig_const_def->name):NULL;
    new_const_def->value = (orig_const_def->value != NULL)? strdup(orig_const_def->value):NULL;
    if((new_const_def->type_s == NULL && orig_const_def->type_s != NULL) ||
       (new_const_def->name == NULL && orig_const_def->name != NULL) ||
       (new_const_def->value == NULL && orig_const_def->value != NULL))
    {
      free(new_const_def->type_s);
      free(new_const_def->name);
      free(new_const_def->value);
      ret=CROS_MEM_ALLOC_ERR;
    }
    else
      ret=CROS_SUCCESS_ERR_PACK;
  }
  else
    ret=CROS_BAD_PARAM_ERR;
  return ret;
}

// Make a copy of a message definition struct allocating new memory for it, so that all fields can be freed independently from the originals
cRosErrCodePack cRosMessageDefCopy(cRosMessageDef** ptr_new_msg_def, cRosMessageDef* orig_msg_def )
{
  cRosErrCodePack ret;

  if(orig_msg_def == NULL)
  {
    *ptr_new_msg_def = NULL;
    return CROS_SUCCESS_ERR_PACK;
  }

  *ptr_new_msg_def = (cRosMessageDef *)malloc(sizeof(cRosMessageDef));
  if(*ptr_new_msg_def != NULL)
  {
    initCrosMsg(*ptr_new_msg_def);
    (*ptr_new_msg_def)->name = (orig_msg_def->name != NULL)? strdup(orig_msg_def->name):NULL; // NULL should not be passed to strdup()
    (*ptr_new_msg_def)->package = (orig_msg_def->package != NULL)? strdup(orig_msg_def->package):NULL;
    (*ptr_new_msg_def)->root_dir = (orig_msg_def->root_dir != NULL)? strdup(orig_msg_def->root_dir):NULL;
    (*ptr_new_msg_def)->plain_text = (orig_msg_def->plain_text != NULL)? strdup(orig_msg_def->plain_text):NULL;

    ret=CROS_SUCCESS_ERR_PACK; // Default return value: no error
    // Copy the first field
    if(orig_msg_def->first_field != NULL)
    {
      msgFieldDef *new_field_itr, *orig_field_itr;

      orig_field_itr =  orig_msg_def->first_field;
      new_field_itr = (*ptr_new_msg_def)->first_field;

      ret=cRosFieldDefCopy(new_field_itr, orig_field_itr); // Copy the first msg def field

      // Copy the rest of message definition fields
      while(orig_field_itr->next != NULL && ret == CROS_SUCCESS_ERR_PACK)
      {
        new_field_itr->next = (msgFieldDef *)malloc(sizeof(msgFieldDef));
        if(new_field_itr->next != NULL)
        {
          initFieldDef(new_field_itr->next);
          ret=cRosFieldDefCopy(new_field_itr->next, orig_field_itr->next);
          if(ret == CROS_SUCCESS_ERR_PACK)
          {
            new_field_itr->next->prev = new_field_itr;
            new_field_itr = new_field_itr->next;
            orig_field_itr = orig_field_itr->next;
          }
          else  // Error allocating memory: discard last filed and exit the loop
          {
            free(new_field_itr->next);
            new_field_itr->next = NULL;
          }
        }
        else
          ret=CROS_MEM_ALLOC_ERR; // Error allocating memory: exit the loop
      }
      (*ptr_new_msg_def)->fields=new_field_itr; // Last valid field
    }
    else
    {
      free((*ptr_new_msg_def)->first_field);
      (*ptr_new_msg_def)->first_field=NULL;
      (*ptr_new_msg_def)->fields=NULL;
    }

     // Copy the first constant
    if(orig_msg_def->first_const != NULL)
    {
      msgConst *new_const_itr, *orig_const_itr;

      orig_const_itr =  orig_msg_def->first_const;
      new_const_itr = (*ptr_new_msg_def)->first_const;

      ret=cRosConstDefCopy(new_const_itr, orig_const_itr);

      // Copy message definition constants
      while(orig_const_itr->next != NULL && ret == CROS_SUCCESS_ERR_PACK)
      {
        new_const_itr->next = (msgConst *)malloc(sizeof(msgConst));
        if(new_const_itr->next != NULL)
        {
          initMsgConst(new_const_itr->next);
          ret=cRosConstDefCopy(new_const_itr->next, orig_const_itr->next);
          if(ret == CROS_SUCCESS_ERR_PACK)
          {
            new_const_itr->next->prev = new_const_itr;
            new_const_itr = new_const_itr->next;
            orig_const_itr = orig_const_itr->next;
          }
          else  // Error allocating memory: discard last filed and exit the loop
          {
            free(new_const_itr->next);
            new_const_itr->next = NULL;
          }
        }
        else
          ret=CROS_MEM_ALLOC_ERR; // Error allocating memory: exit the loop
      }
      (*ptr_new_msg_def)->constants = new_const_itr; // Last valid constant

    }
    else
    {
      free((*ptr_new_msg_def)->first_const);
      (*ptr_new_msg_def)->first_const=NULL;
      (*ptr_new_msg_def)->constants=NULL;
    }
    if(ret != CROS_SUCCESS_ERR_PACK) // Error making the msg. def. copy: free all the memory that we have allocated
    {
      cRosMessageDefFree(*ptr_new_msg_def);
      *ptr_new_msg_def = NULL;
    }
  }
  else
    ret=CROS_MEM_ALLOC_ERR;
  return(ret);
}

cRosErrCodePack cRosMessageNewBuild(const char *msg_root_dir, const char *msg_type, cRosMessage **new_msg_ptr)
{
  cRosErrCodePack ret_err;

  if(new_msg_ptr != NULL)
  {
    cRosMessageDef *msg_def;
    ret_err = cRosMessageDefBuild(&msg_def, msg_root_dir, msg_type);
    if(ret_err == CROS_SUCCESS_ERR_PACK) // Check if we got any error while building message definition
    {
      ret_err = cRosMessageBuildFromDef(new_msg_ptr, msg_def); // Copy msg_def into message->msgDef and build the message (original msg_def can be freed after the copy)
      cRosMessageDefFree(msg_def);
    }
  }
  else
    ret_err = CROS_BAD_PARAM_ERR;
  return ret_err;
}

cRosErrCodePack cRosMessageBuildFromDef(cRosMessage** message_ptr, cRosMessageDef* msg_def )
{
  cRosErrCodePack ret;
  cRosMessage* message;

  ret = CROS_SUCCESS_ERR_PACK; // Default return value: success

  message = cRosMessageNew();
  if(message == NULL)
    return CROS_MEM_ALLOC_ERR;

  DynString output;
  dynStringInit(&output);

  cRosMessageDefFree(message->msgDef); // Just in case there was a previous message definition in the message
  cRosMessageDefCopy(&message->msgDef, msg_def );

  unsigned char* res = getMD5Msg(msg_def);
  cRosMD5Readable(res, &output);
  free(res);
  strcpy(message->md5sum, dynStringGetData(&output));
  dynStringRelease(&output);

  msgFieldDef* field_def_itr =  msg_def->first_field;
  while(field_def_itr->next != NULL)
  {
    message->n_fields++;
    field_def_itr = field_def_itr->next;
  }

  message->fields = (cRosMessageField **)calloc(message->n_fields, sizeof(cRosMessageField*));

  field_def_itr =  msg_def->first_field;
  cRosMessageField** msg_field_itr = message->fields;

  while(field_def_itr->next != NULL && ret == CROS_SUCCESS_ERR_PACK)
  {
    *msg_field_itr = (cRosMessageField *)malloc(sizeof(cRosMessageField));
    cRosMessageField* field = *msg_field_itr;
    cRosMessageFieldInit(field);

    field->type = field_def_itr->type;
    field->name = (char *)calloc(strlen(field_def_itr->name)+1,sizeof(char));
    strcpy(field->name, field_def_itr->name);
    if (field_def_itr->type_s != NULL)
    {
      field->type_s = (char *)calloc(strlen(field_def_itr->type_s)+1,sizeof(char));
      strcpy(field->type_s, field_def_itr->type_s);
    }

    field->is_array = field_def_itr->is_array;
    if(field_def_itr->is_array)
    {
      if (field_def_itr->array_size == -1) // Variable-length array
      {
        field->array_size = 0;
        field->array_capacity = 1; // If we don't now the array length, allocate memory for at least one element
      }
      else // Fixed-length array
      {
        field->array_size = field_def_itr->array_size;
        field->array_capacity = field_def_itr->array_size;
        field->is_fixed_array = 1;
      }
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
        if(field_def_itr->is_array)
        {
          field->data.as_array = calloc(field->array_capacity,field->size);
          if(field->data.as_array == NULL)
            ret=CROS_MEM_ALLOC_ERR;
        }
        break;
      }
      case CROS_STD_MSGS_STRING:
      {
        if(field_def_itr->is_array)
        {
          field->data.as_string_array = (char **)calloc(field->array_capacity,sizeof(char *));
          if(field->data.as_string_array == NULL)
            ret=CROS_MEM_ALLOC_ERR;
        }
        break;
      }
      case CROS_STD_MSGS_TIME:
      case CROS_STD_MSGS_DURATION:
      case CROS_STD_MSGS_HEADER:
      case CROS_CUSTOM_TYPE:
      {
        cRosMessage *new_msg;
        if(field_def_itr->is_array)
        {
          field->data.as_msg_array = (cRosMessage **)calloc(field->array_capacity,sizeof(cRosMessage *));
          if(field->data.as_msg_array != NULL)
          {
            int msg_ind;

            for(msg_ind=0;msg_ind<field->array_size && ret == CROS_SUCCESS_ERR_PACK;msg_ind++)
            {
              new_msg = NULL;
              if(field->type == CROS_STD_MSGS_TIME)
                new_msg = build_time_field();
              else if(field->type == CROS_STD_MSGS_DURATION)
                new_msg = build_duration_field();
              else if(field->type == CROS_STD_MSGS_HEADER)
                new_msg = build_header_field();
              else if(field->type == CROS_CUSTOM_TYPE)
              {
                ret = cRosMessageBuildFromDef(&new_msg, field_def_itr->child_msg_def); // faster alternative to ret = cRosMessageNewBuild(msg_def->root_dir, field_def_itr->type_s, &new_msg)
                ret = cRosAddErrCodeIfErr(ret, CROS_CREATE_CUSTOM_MSG_ERR); // If the message could not be created, add more info to the error code pack
              }

              if(new_msg != NULL) // New message created: insert it into the array
                cRosMessageFieldArrayAtMsgSet(field, msg_ind, new_msg);
              else // Error creating the new message
              {
                if(field->type == CROS_STD_MSGS_TIME || field->type == CROS_STD_MSGS_DURATION || field->type == CROS_STD_MSGS_HEADER)
                  ret = CROS_MEM_ALLOC_ERR;
              }
            }
          }
          else
            ret = CROS_MEM_ALLOC_ERR;
        }
        else
        {
          if(field->type == CROS_STD_MSGS_TIME)
            field->data.as_msg = build_time_field();
          else if(field->type == CROS_STD_MSGS_DURATION)
            field->data.as_msg = build_duration_field();
          else if(field->type == CROS_STD_MSGS_HEADER)
            field->data.as_msg = build_header_field();
          else if(field->type == CROS_CUSTOM_TYPE)
          {
            ret = cRosMessageBuildFromDef(&field->data.as_msg, field_def_itr->child_msg_def);
            ret = cRosAddErrCodeIfErr(ret, CROS_CREATE_CUSTOM_MSG_ERR);
          }

          if(field->data.as_msg == NULL && (field->type == CROS_STD_MSGS_TIME || field->type == CROS_STD_MSGS_DURATION || field->type == CROS_STD_MSGS_HEADER))
            ret = CROS_MEM_ALLOC_ERR;
        }
        break;
      }
    }
    msg_field_itr++;
    field_def_itr = field_def_itr->next;
  }

  if(ret == CROS_SUCCESS_ERR_PACK)
    *message_ptr = message;
  else
    cRosMessageFree(message);

  return ret;
}

void cRosMessageConstDefFree(msgConst* msg_const)
{
  if(msg_const != NULL)
  {
    free(msg_const->name);
    msg_const->name = NULL;
    free(msg_const->type_s);
    msg_const->type_s = NULL;
    free(msg_const->value);
    msg_const->value = NULL;
    free(msg_const); // Assignment to NULL not needed
  }
}

void cRosMessageFieldDefFree(msgFieldDef* msg_field)
{
  if(msg_field != NULL)
  {
    cRosMessageDefFree(msg_field->child_msg_def);
    free(msg_field->name);
    msg_field->name = NULL;
    free(msg_field->type_s);
    msg_field->type_s = NULL; // Assignment to NULL not needed
    free(msg_field);
  }
}

void cRosMessageDefFree(cRosMessageDef *msgDef)
{
  if (msgDef == NULL)
    return;

  free(msgDef->name);
  msgDef->name = NULL;
  free(msgDef->package);
  msgDef->package = NULL;
  free(msgDef->root_dir);
  msgDef->root_dir = NULL;
  free(msgDef->plain_text);
  msgDef->plain_text = NULL;

  msgConst* it_const = msgDef->first_const;
  while(it_const != NULL)
  {
    msgConst* next_const = it_const->next;
    cRosMessageConstDefFree(it_const);
    it_const = next_const;
  }
  msgDef->first_const = NULL;
  msgDef->constants = NULL;

  msgFieldDef* it_field = msgDef->first_field;
  while(it_field != NULL)
  {
    msgFieldDef* next_field=it_field->next;
    cRosMessageFieldDefFree(it_field);
    it_field = next_field;
  }
  msgDef->first_field = NULL;
  msgDef->fields = NULL;
  free(msgDef); // All the previous pointer assignments to NULL make no sense if we free the cRosMessageDef struct
}

void cRosMessageFieldsFree(cRosMessage *message)
{
  int i;

  for(i = 0; i < message->n_fields; i++)
    cRosMessageFieldFree(message->fields[i]);
  free(message->fields);
  message->fields = NULL;
  message->n_fields = 0;
}

void cRosMessageRelease(cRosMessage *message)
{
  cRosMessageFieldsFree(message);

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

cRosMessageField * cRosMessageFieldNew(void)
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
    if(field->type != CROS_CUSTOM_TYPE && field->type != CROS_STD_MSGS_STRING &&
    field->type != CROS_STD_MSGS_TIME && field->type != CROS_STD_MSGS_DURATION &&
    field->type != CROS_STD_MSGS_HEADER)
    {
      free(field->data.as_array);
      field->data.as_array = NULL;
    }
    else
    {
        int elem_ind;
        for(elem_ind = 0; elem_ind < field->array_size; elem_ind++)
        {
          if(field->type == CROS_STD_MSGS_STRING)
            free(field->data.as_string_array[elem_ind]);
          else
            cRosMessageFree(field->data.as_msg_array[elem_ind]);
        }
        free(field->data.as_array);
        field->data.as_array = NULL;
    }
  }
  else
  {
    if(field->type == CROS_STD_MSGS_STRING)
    {
      free(field->data.as_string);
      field->data.as_string=NULL;
    }
    else if(field->type == CROS_CUSTOM_TYPE || field->type == CROS_STD_MSGS_TIME ||
            field->type == CROS_STD_MSGS_DURATION || field->type == CROS_STD_MSGS_HEADER)
    {
      cRosMessageFree(field->data.as_msg);
      field->data.as_msg=NULL;
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

cRosMessageField *cRosMessageGetField(cRosMessage *message, const char *field_name)
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
  int ret;
  if (field->type != CROS_STD_MSGS_STRING)
    return -1;

  size_t str_len = strlen(value);
  field->data.as_string = (char *)realloc(field->data.as_string, sizeof(char)*(str_len+1));
  if(field->data.as_string != NULL)
  {
    strcpy(field->data.as_string, value);
    field->size = (int)str_len;
    ret=0; // Success
  }
  else
    ret=-1;
  return ret;
}

int arrayFieldValuesPushBack(cRosMessageField *field, const void* data, int element_size, int n_new_elements)
{
  if(field == NULL || !field->is_array || field->is_fixed_array)
    return -1;

  if(field->array_capacity < field->array_size + n_new_elements)
  {
    void *new_location;
    size_t new_arr_cap;

    new_arr_cap = (field->array_size + n_new_elements) * 3 / 2;
    new_location = realloc(field->data.as_array, new_arr_cap * element_size); // If field->data.as_array is NULL, realloc() behaves as malloc()
    if(new_location != NULL)
    {
      field->data.as_array = new_location;
      field->array_capacity = (int)new_arr_cap;
    }
    else
    {
      return -1;
    }
  }

  memcpy((void *)((char *)field->data.as_array + field->array_size*element_size), data, element_size * n_new_elements);
  field->array_size += n_new_elements;
  return 0;
}

int arrayFieldValuePushBack(cRosMessageField *field, const void* data, int element_size)
{
  return arrayFieldValuesPushBack(field, data, element_size, 1);
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
    char** new_location;
    new_location = (char **)realloc(field->data.as_string_array, 2 * field->array_capacity * sizeof(char*));
    if(new_location != NULL)
    {
      field->data.as_string_array = new_location;
      field->array_capacity *= 2;
    }
    else
    {
      return -1;
    }
  }
  int ret;
  char* element_val;
  ret=0;
  if(val != NULL)
  {
    element_val = (char *)calloc(strlen(val) + sizeof(char), 1);
    if(element_val != NULL)
      strcpy(element_val, val);
    else
      ret=-1;
  }
  else
    element_val = NULL;
  if(ret == 0)
  {
    field->data.as_string_array[field->array_size] = element_val;
    field->array_size ++;
  }
  return ret;
}

// Add a new blank element at the end of the array in field n_field of message msg
int cRosMessageFieldArrayPushBackZero(cRosMessage* msg, int n_field)
{
  cRosMessageField *field;
  CrosMessageType elem_type;
  size_t elem_size;

  if(n_field >= msg->n_fields || n_field < 0)
    return -1;

  field = msg->fields[n_field];

  if(!field->is_array || field->is_fixed_array)
    return -1;

  elem_type = field->type;
  elem_size = getMessageTypeSizeOf(elem_type);
  if(field->array_capacity == field->array_size)
  {
    void *new_location;
    new_location = realloc(field->data.as_array, 2 * field->array_capacity * elem_size);
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

  switch(elem_type)
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
    case CROS_STD_MSGS_STRING:
    {
      char *elem_array = field->data.as_array;
      memset(elem_array + elem_size*field->array_size, 0, elem_size);
      break;
    }
    case CROS_STD_MSGS_TIME:
    case CROS_STD_MSGS_DURATION:
    case CROS_STD_MSGS_HEADER:
    case CROS_CUSTOM_TYPE:
    {
      if(msg->msgDef != NULL)
      {
        msgFieldDef* field_def_itr;
        int field_ind;
        cRosMessage **msg_array;
        cRosErrCodePack ret_err;

         // Look for the definition corresponding to the specified field
        field_def_itr = msg->msgDef->first_field;
        for(field_ind=0;field_ind < n_field && field_def_itr != NULL;field_ind++)
          field_def_itr = field_def_itr->next;

        if(field_def_itr == NULL)
          return -1; // msg definition not found

        msg_array = field->data.as_msg_array;
        ret_err = cRosMessageBuildFromDef(&msg_array[field->array_size], field_def_itr->child_msg_def); // instead of cRosMessageNewBuild(msg->msgDef->root_dir, field->type_s, &msg_array[field->array_size]);
        if(ret_err != CROS_SUCCESS_ERR_PACK)
          return -1; // Error creating the message
      }
      else
        return -1; // msgDef not available in current message, we don't know the type of the message to build
    }
    default:
      break;
  }

  field->array_size ++;
  return 0;
}

int cRosMessageFieldArrayPushBackMsg(cRosMessageField *field, cRosMessage* msg)
{
  if(field->type != CROS_CUSTOM_TYPE && field->type != CROS_STD_MSGS_TIME &&
     field->type != CROS_STD_MSGS_DURATION && field->type != CROS_STD_MSGS_HEADER)
    return -1;

  if(!field->is_array || field->is_fixed_array)
    return -1;

  if(field->array_capacity == field->array_size)
  {
    cRosMessage **new_location;
    new_location = (cRosMessage **)realloc(field->data.as_msg_array, 2 * field->array_capacity * sizeof(cRosMessage*));
    if(new_location != NULL)
    {
      field->data.as_msg_array = new_location;
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

cRosMessage *cRosMessageFieldArrayRemoveLastMsg(cRosMessageField *field)
{
  if(field->type != CROS_CUSTOM_TYPE && field->type != CROS_STD_MSGS_TIME &&
     field->type != CROS_STD_MSGS_DURATION && field->type != CROS_STD_MSGS_HEADER)
    return NULL;

  if(!field->is_array || field->is_fixed_array)
    return NULL;

  cRosMessage *msg;

  if(field->array_size > 0)
  {
    msg = field->data.as_msg_array[field->array_size-1];
    field->array_size--;
  }
  else
    msg=NULL; // Array empty

  return msg;
}

int8_t *cRosMessageFieldArrayAtInt8(cRosMessageField *field, int position)
{
  if(field->type != CROS_STD_MSGS_INT8)
    return NULL;

  return (int8_t *)arrayFieldValueAt(field, position, sizeof(int8_t));
}

int16_t *cRosMessageFieldArrayAtInt16(cRosMessageField *field, int position)
{
  if(field->type != CROS_STD_MSGS_INT16)
    return NULL;

  return (int16_t *)arrayFieldValueAt(field, position, sizeof(int16_t));
}

int32_t *cRosMessageFieldArrayAtInt32(cRosMessageField *field, int position)
{
  if(field->type != CROS_STD_MSGS_INT32)
    return NULL;

  return (int32_t *)arrayFieldValueAt(field, position, sizeof(int32_t));
}

int64_t *cRosMessageFieldArrayAtInt64(cRosMessageField *field, int position)
{
  if(field->type != CROS_STD_MSGS_INT64)
    return NULL;

  return (int64_t *)arrayFieldValueAt(field, position, sizeof(int64_t));
}

uint8_t *cRosMessageFieldArrayAtUInt8(cRosMessageField *field, int position)
{
  if(field->type != CROS_STD_MSGS_UINT8)
    return NULL;

  return (uint8_t *)arrayFieldValueAt(field, position, sizeof(uint8_t));
}

uint16_t *cRosMessageFieldArrayAtUInt16(cRosMessageField *field, int position)
{
  if(field->type != CROS_STD_MSGS_UINT16)
    return NULL;

  return (uint16_t *)arrayFieldValueAt(field, position, sizeof(uint16_t));
}

uint32_t *cRosMessageFieldArrayAtUInt32(cRosMessageField *field, int position)
{
  if(field->type != CROS_STD_MSGS_UINT32)
    return NULL;

  return (uint32_t *)arrayFieldValueAt(field, position, sizeof(uint32_t));
}

uint64_t *cRosMessageFieldArrayAtUInt64(cRosMessageField *field, int position)
{
  if(field->type != CROS_STD_MSGS_UINT64)
    return NULL;

  return (uint64_t *)arrayFieldValueAt(field, position, sizeof(uint64_t));
}

float *cRosMessageFieldArrayAtFloat32(cRosMessageField *field, int position)
{
  if(field->type != CROS_STD_MSGS_FLOAT32)
    return NULL;

  return (float *)arrayFieldValueAt(field, position, sizeof(float));
}

double *cRosMessageFieldArrayAtFloat64(cRosMessageField *field, int position)
{
  if(field->type != CROS_STD_MSGS_FLOAT64)
    return NULL;

  return (double *)arrayFieldValueAt(field, position, sizeof(double));
}

cRosMessage *cRosMessageFieldArrayAtMsgGet(cRosMessageField *field, int position)
{
  if(field == NULL || (field->type != CROS_CUSTOM_TYPE && field->type != CROS_STD_MSGS_TIME &&
      field->type != CROS_STD_MSGS_DURATION && field->type != CROS_STD_MSGS_HEADER) || !field->is_array)
    return NULL;

  if(position >= field->array_size)
    return NULL;

  cRosMessage* msg =  field->data.as_msg_array[position];
  return msg;
}

int cRosMessageFieldArrayAtMsgSet(cRosMessageField *field, int position, cRosMessage* msg)
{
  int ret;
  if((field->type != CROS_CUSTOM_TYPE && field->type != CROS_STD_MSGS_TIME &&
      field->type != CROS_STD_MSGS_DURATION && field->type != CROS_STD_MSGS_HEADER) || !field->is_array)
    return -1;

  if(position > field->array_size)
    return -1;

  if(position == field->array_size)
    ret = cRosMessageFieldArrayPushBackMsg(field, msg);
  else
  {
    cRosMessageFree(field->data.as_msg_array[position]);
    field->data.as_msg_array[position] = msg;
    ret = 0;
  }

  return ret;
}

char *cRosMessageFieldArrayAtStringGet(cRosMessageField *field, int position)
{
  if(field->type != CROS_STD_MSGS_STRING || !field->is_array)
    return NULL;

  char* str = (char*) field->data.as_string_array[position];
  return str;
}

int cRosMessageFieldArrayAtStringSet(cRosMessageField *field, int position, const char* val)
{
  int ret;
  if(field->type != CROS_STD_MSGS_STRING || !field->is_array)
    return -1;

  char *current = field->data.as_string_array[position];
  ret = 0; // Default return value: success
  if(val != NULL)
  {
    current = (char *)realloc(current, strlen(val) + sizeof(char)); // Realloc buffer for the string
    if(current != NULL)
      strcpy(current, val);
    else
      ret=-1;
  }
  else
  {
    if(current != NULL)
    {
      free(current);
      current = NULL;
    }
  }

  field->data.as_string_array[position] = current;

  return ret;
}

int cRosMessageFieldArrayClear(cRosMessageField *field)
{
  int n_elem;
  if(!field->is_array || field->is_fixed_array)
    return -1;

  for(n_elem=0;n_elem<field->array_size;n_elem++)
  {
    switch(field->type)
    {
      case CROS_STD_MSGS_STRING:
      {
        free(field->data.as_string_array[n_elem]);
        break;
      }
      case CROS_STD_MSGS_TIME:
      case CROS_STD_MSGS_DURATION:
      case CROS_STD_MSGS_HEADER:
      case CROS_CUSTOM_TYPE:
      {
        cRosMessageFree(field->data.as_msg_array[n_elem]);
        break;
      }
      default:
        break;
    }
  }

  field->array_size = 0;

  return 0;
}

void *arrayFieldValueAt(cRosMessageField *field, int position, size_t size)
{
  if(!field->is_array)
    return NULL;

  if (position < 0 || position > field->array_size)
    return NULL;

  return((void *)((char *)field->data.as_array + (position * size))); // sizeof(char) is 1 (byte), so we cast to char * before calculating the array field position
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

cRosErrCodePack cRosMessageSerialize(cRosMessage *message, DynBuffer* buffer)
{
  cRosErrCodePack ret_err;
  int field_ind;

  ret_err = CROS_SUCCESS_ERR_PACK; // default error value: success
  for (field_ind = 0; field_ind < message->n_fields && ret_err == CROS_SUCCESS_ERR_PACK; field_ind++)
  {
    cRosMessageField *field = message->fields[field_ind];

    if(field->is_array && !field->is_fixed_array)
      ret_err = (dynBufferPushBackInt32(buffer, field->array_size) >= 0)?CROS_SUCCESS_ERR_PACK:CROS_MEM_ALLOC_ERR;

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
        size_t size = getMessageTypeSizeOf(field->type);
        if (field->is_array)
          ret_err = (dynBufferPushBackBuf(buffer, field->data.as_array, size * field->array_size) >= 0)?CROS_SUCCESS_ERR_PACK:CROS_MEM_ALLOC_ERR;
        else
          ret_err = (dynBufferPushBackBuf(buffer, field->data.opaque, size) >= 0)?CROS_SUCCESS_ERR_PACK:CROS_MEM_ALLOC_ERR;
        break;
      }
      case CROS_STD_MSGS_TIME:
      case CROS_STD_MSGS_DURATION:
      case CROS_STD_MSGS_HEADER:
      case CROS_CUSTOM_TYPE:
      {
        if(field->is_array)
        {
          int elem_ind;
          for(elem_ind = 0; elem_ind < field->array_size && ret_err == CROS_SUCCESS_ERR_PACK; elem_ind++)
          {
            cRosMessage *arr_elem_msg;
            arr_elem_msg = cRosMessageFieldArrayAtMsgGet(field, elem_ind);
            if(arr_elem_msg != NULL)
              ret_err = cRosMessageSerialize(arr_elem_msg, buffer);
          }
        }
        else
        {
          if(field->data.as_msg != NULL)
            ret_err = cRosMessageSerialize(field->data.as_msg, buffer);
        }
        break;
      }
      case CROS_STD_MSGS_STRING:
      {
        if(field->is_array)
        {
          int elem_ind;
          for(elem_ind = 0; elem_ind < field->array_size && ret_err == CROS_SUCCESS_ERR_PACK; elem_ind++)
          {
            const char* arr_elem_str;
            size_t arr_elem_str_len;
            arr_elem_str = cRosMessageFieldArrayAtStringGet(field, elem_ind);
            if(arr_elem_str != NULL)
              arr_elem_str_len = strlen(arr_elem_str);
            else
              arr_elem_str_len = 0;
            dynBufferPushBackInt32(buffer, arr_elem_str_len);
            ret_err = (dynBufferPushBackBuf(buffer, (unsigned char *)arr_elem_str, arr_elem_str_len) >= 0)?CROS_SUCCESS_ERR_PACK:CROS_MEM_ALLOC_ERR;
          }
        }
        else
        {
          if(field->data.as_string != NULL)
          {
            size_t str_len = strlen(field->data.as_string);
            dynBufferPushBackInt32(buffer, str_len);
            ret_err = (dynBufferPushBackBuf(buffer,(unsigned char*)field->data.as_string, str_len) >= 0)?CROS_SUCCESS_ERR_PACK:CROS_MEM_ALLOC_ERR;
          }
          else
            ret_err = (dynBufferPushBackInt32(buffer, 0) >= 0)?CROS_SUCCESS_ERR_PACK:CROS_MEM_ALLOC_ERR;
        }
        break;
      }
      default:
      {
        ret_err = CROS_BAD_PARAM_ERR;
        break;
      }
    }
  }
  return ret_err;
}

// In this function we assume that the message is already build according to its definition.
// Only when receiving a variable-length array, new elements if the message field may need to be created
cRosErrCodePack cRosMessageDeserialize(cRosMessage *message, DynBuffer* buffer)
{
  int field_ind;
  cRosErrCodePack ret_err;

  ret_err = CROS_SUCCESS_ERR_PACK; // default error value: no error

  msgFieldDef* field_def_itr =  (message->msgDef != NULL)? message->msgDef->first_field : NULL; // Keep track of the message definition (if available) corresponding to the current message field in case we need to build a msg of type custom

  for (field_ind = 0; field_ind < message->n_fields && ret_err == CROS_SUCCESS_ERR_PACK; field_ind++)
  {
    cRosMessageField *field = message->fields[field_ind];

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
        size_t elem_size = getMessageTypeSizeOf(field->type);
        if (field->is_array)
        {
          if (field->is_fixed_array)
          {
            ret_err = (dynBufferGetCurrentContent( field->data.as_array, buffer, elem_size * field->array_size ) >= 0)?CROS_SUCCESS_ERR_PACK:CROS_MEM_ALLOC_ERR; // equiv. to: memcpy(field->data.as_array, dynBufferGetCurrentData(buffer), size * field->array_size);
            dynBufferMovePoseIndicator(buffer, elem_size * field->array_size);
          }
          else
          {
            uint32_t array_n_elems;
            cRosMessageFieldArrayClear(field);
            ret_err = (dynBufferGetCurrentContent( (unsigned char *)&array_n_elems, buffer, sizeof(uint32_t) ) >= 0)?CROS_SUCCESS_ERR_PACK:CROS_MEM_ALLOC_ERR; // equiv. to: array_n_elems = *((uint32_t*)dynBufferGetCurrentData(buffer));
            dynBufferMovePoseIndicator(buffer, sizeof(uint32_t));
            if(ret_err == CROS_SUCCESS_ERR_PACK)
            {
              if(dynBufferGetRemainingDataSize(buffer) >= array_n_elems*elem_size)
              {
                ret_err = (arrayFieldValuesPushBack(field, (const void *)dynBufferGetCurrentData(buffer), elem_size, array_n_elems) >= 0)?CROS_SUCCESS_ERR_PACK:CROS_MEM_ALLOC_ERR;
                dynBufferMovePoseIndicator(buffer, array_n_elems*elem_size);
              }
              else
                ret_err = CROS_DEPACK_INSUFF_DAT_ERR; // Not enough data available in the packet buffer
            }
          }
        }
        else
        {
          ret_err = (dynBufferGetCurrentContent( field->data.opaque, buffer, elem_size ) >= 0)?CROS_SUCCESS_ERR_PACK:CROS_MEM_ALLOC_ERR; // equiv. to: memcpy(field->data.opaque, dynBufferGetCurrentData(buffer), elem_size);
          dynBufferMovePoseIndicator(buffer, elem_size);
        }

        break;
      }
      case CROS_STD_MSGS_STRING:
      {
        if (field->is_array)
        {
          uint32_t curr_array_size;
          if(field->is_fixed_array) // If it is a fixed array, use the fixed array size
            curr_array_size = field->array_size;
          else // Otherwise, obtain the number of elements of the received array
          {
            cRosMessageFieldArrayClear(field); // Remove previous strings from the array to start added the new ones one by one
            ret_err = (dynBufferGetCurrentContent( (unsigned char *)&curr_array_size, buffer, sizeof(uint32_t) ) >= 0)?CROS_SUCCESS_ERR_PACK:CROS_MEM_ALLOC_ERR; // equiv. to: curr_array_size = *((uint32_t*)dynBufferGetCurrentData(buffer));
            dynBufferMovePoseIndicator(buffer, sizeof(uint32_t));
          }

          uint32_t elem_ind;
          for(elem_ind = 0; elem_ind < curr_array_size && ret_err == CROS_SUCCESS_ERR_PACK; elem_ind++)
          {
            uint32_t element_size;
            ret_err = (dynBufferGetCurrentContent( (unsigned char *)&element_size, buffer, sizeof(uint32_t) ) >= 0)?CROS_SUCCESS_ERR_PACK:CROS_MEM_ALLOC_ERR; // equiv. to: element_size = *((uint32_t*)dynBufferGetCurrentData(buffer));
            dynBufferMovePoseIndicator(buffer, sizeof(uint32_t));
            if(ret_err == CROS_SUCCESS_ERR_PACK)
            {
              char* tmp_string = (char *)calloc(element_size + 1, sizeof(char));
              if(tmp_string != NULL)
              {
                ret_err = (dynBufferGetCurrentContent( (unsigned char *)tmp_string, buffer, element_size ) >= 0)?CROS_SUCCESS_ERR_PACK:CROS_MEM_ALLOC_ERR; // equiv. to: memcpy(tmp_string, dynBufferGetCurrentData(buffer), element_size);
                if(field->is_fixed_array)
                  ret_err = (cRosMessageFieldArrayAtStringSet(field, elem_ind, tmp_string) >= 0)?CROS_SUCCESS_ERR_PACK:CROS_MEM_ALLOC_ERR;
                else
                  ret_err = (cRosMessageFieldArrayPushBackString(field, tmp_string) >= 0)?CROS_SUCCESS_ERR_PACK:CROS_MEM_ALLOC_ERR;
                dynBufferMovePoseIndicator(buffer, element_size);
                free(tmp_string);
              }
              else
                ret_err=CROS_MEM_ALLOC_ERR;
            }
          }
        }
        else
        {
          uint32_t curr_data_size;
          ret_err = (dynBufferGetCurrentContent( (unsigned char *)&curr_data_size, buffer, sizeof(uint32_t) ) >= 0)?CROS_SUCCESS_ERR_PACK:CROS_MEM_ALLOC_ERR; // equiv. to: curr_data_size = *((uint32_t*)dynBufferGetCurrentData(buffer));
          dynBufferMovePoseIndicator(buffer, sizeof(uint32_t));
          field->data.as_string = (char *)realloc(field->data.as_string, (curr_data_size + 1) * sizeof(char)); // If field->data.as_string was NULL previously, realloc behaves as malloc
          if(field->data.as_string != NULL)
          {
            dynBufferGetCurrentContent( (unsigned char *)field->data.as_string, buffer, curr_data_size ); // equiv. to: memcpy(field->data.as_string, dynBufferGetCurrentData(buffer), curr_data_size);
            field->data.as_string[curr_data_size] = '\0';
            dynBufferMovePoseIndicator(buffer, curr_data_size);
          }
          else
            ret_err=CROS_MEM_ALLOC_ERR;
        }
        break;
      }
      case CROS_STD_MSGS_TIME:
      case CROS_STD_MSGS_DURATION:
      case CROS_STD_MSGS_HEADER:
      case CROS_CUSTOM_TYPE:
      {
        if (field->is_array)
        {
          uint32_t received_arr_siz;
          uint32_t msg_ind;

          if(field->is_fixed_array) // If it is a fixed array, we use the fixed array size
            received_arr_siz = field->array_size;
          else // Otherwise, obtain the number of array elements from the received packet
          {
            ret_err = (dynBufferGetCurrentContent( (unsigned char *)&received_arr_siz, buffer, sizeof(uint32_t) ) >= 0)?CROS_SUCCESS_ERR_PACK:CROS_MEM_ALLOC_ERR;
            dynBufferMovePoseIndicator(buffer, sizeof(uint32_t));

            // Adapt the array size of the message field to the received array length
            for(msg_ind = field->array_size;msg_ind < received_arr_siz && ret_err == CROS_SUCCESS_ERR_PACK;msg_ind++) // received more elements than the available messages: create more
            {
              cRosMessage *new_msg;
              new_msg = NULL;
              if(field->type == CROS_STD_MSGS_TIME)
                new_msg = build_time_field();
              else if(field->type == CROS_STD_MSGS_DURATION)
                new_msg = build_duration_field();
              else if(field->type == CROS_STD_MSGS_HEADER)
                new_msg = build_header_field();
              else if(field->type == CROS_CUSTOM_TYPE)
              {
                if(field_def_itr != NULL)
                  ret_err = cRosMessageBuildFromDef(&new_msg, field_def_itr->child_msg_def); // instead of cRosMessageNewBuild(message->msgDef->root_dir, field->type_s, &new_msg);
                else
                  ret_err =CROS_DEPACK_NO_MSG_DEF_ERR;
              }

              if(new_msg != NULL)
                cRosMessageFieldArrayPushBackMsg(field, new_msg); // new_msg is now used in the array so it cannot be freed independently
              else
              {
                if(ret_err == CROS_SUCCESS_ERR_PACK)
                  ret_err=CROS_MEM_ALLOC_ERR;
              }
            }

            while(field->array_size > (int)received_arr_siz) // received less elements than the available messages: delete
              cRosMessageFree(cRosMessageFieldArrayRemoveLastMsg(field));
          }

          for(msg_ind = 0;(int)msg_ind < field->array_size && ret_err == CROS_SUCCESS_ERR_PACK;msg_ind++)
          {
            cRosMessage *curr_msg;
            curr_msg = cRosMessageFieldArrayAtMsgGet(field, msg_ind);
            ret_err = cRosMessageDeserialize(curr_msg, buffer);
          }
        }
        else
          ret_err = cRosMessageDeserialize(field->data.as_msg, buffer);
        break;
      }
      default:
      {
        ret_err=CROS_BAD_PARAM_ERR;
        break;
      }
    }
    field_def_itr = (field_def_itr != NULL)? field_def_itr->next : NULL;
  }
  return ret_err;
}

const char *getMessageTypeDeclarationConst(msgConst *msgConst)
{
  if (msgConst->type_s == NULL)
    return getMessageTypeDeclaration(msgConst->type);
   else
     return msgConst->type_s;
}

const char *getMessageTypeDeclarationField(msgFieldDef *fieldDef)
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
      return sizeof(int8_t);
    case CROS_STD_MSGS_INT16:
    case CROS_STD_MSGS_UINT16:
      return sizeof(int16_t);
    case CROS_STD_MSGS_INT32:
    case CROS_STD_MSGS_UINT32:
      return sizeof(int32_t);
    case CROS_STD_MSGS_INT64:
    case CROS_STD_MSGS_UINT64:
      return sizeof(int64_t);
    case CROS_STD_MSGS_FLOAT32:
      return sizeof(float);
    case CROS_STD_MSGS_FLOAT64:
      return sizeof(double);
    case CROS_STD_MSGS_BOOL:
      return sizeof(int8_t);
    case CROS_STD_MSGS_STRING:
      return sizeof(char *);
    case CROS_STD_MSGS_TIME:
    case CROS_STD_MSGS_DURATION:
    case CROS_STD_MSGS_HEADER:
    case CROS_CUSTOM_TYPE:
      return sizeof(cRosMessage *);
    // deprecated
    case CROS_STD_MSGS_CHAR:
    case CROS_STD_MSGS_BYTE:
      return sizeof(char);
    default:
      PRINT_ERROR ( "getMessageTypeSizeOf() : Invalid CrosMessageType specified\n" );
      return 0;
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

const char *getMessageTypeString(CrosMessageType type)
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
      PRINT_ERROR ( "getMessageTypeString() : Invalid CrosMessageType specified\n" );
      return NULL;
  }
}

const char *getMessageTypeDeclaration(CrosMessageType type)
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
    case CROS_CUSTOM_TYPE:
      return "custom";
    default:
      PRINT_ERROR ( "getMessageTypeString() : Invalid CrosMessageType specified\n" );
      return NULL;
  }
}
