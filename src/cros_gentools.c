#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "cros_gentools.h"
#include "md5.h"

char* base_msg_type(char* type)
{
	//	"""
	//	Compute the base data type, e.g. for arrays, get the underlying array item type
	//	@param type_: ROS msg type (e.g. 'std_msgs/String')
	//	@type  type_: str
	//	@return: base type
	//	@rtype: str
	//	"""
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

//	base = base_msg_type(x)
	char* base = base_msg_type(type_statement);
	//	if not roslib.names.is_legal_resource_name(base):
	//			return False
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

int loadFromStringMsg(char* text, cRosMsg* msg)
{
//	Load message specification from a string:
//	types, names, constants

		char* new_line = NULL;
		char* new_line_saveptr = NULL;
		const char* delimiter = "\n";
		//	for orig_line in text.split('\n'):
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

			//	l = orig_line.split(COMMENTCHAR)[0].strip() #strip comments
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

			//	if not l:
			//		continue #ignore empty lines
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

			//	splits = [s for s in [x.strip() for x in l.split(" ")] if s] #split type/name, filter out empties
			//	type_ = splits[0]
			char* msg_entry_itr = msg_entry;
			entry_type = msg_entry_itr;

			while(*(msg_entry_itr++) != ' ');
			*(msg_entry_itr - 1) = '\0';

			entry_name = msg_entry_itr;

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
			//	if not is_valid_msg_type(type_):
			//		raise MsgSpecException("%s is not a legal message type"%type_)
			if(!is_valid_msg_type(entry_type))
				return 0;

			//	if CONSTCHAR in l:
			//		if not is_valid_constant_type(type_):
			//			raise MsgSpecException("%s is not a legal constant type"%type_)
			char* const_char_ptr = strpbrk(entry_name, CHAR_CONST);

			if( const_char_ptr != NULL)
			{
				if(strcmp(entry_type, "string") == 0)
				{
					//	String constants
					//	Strings contain anything to the right of the equals sign,
					//	there are no comments allowed

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
				msgField* current = msg->fields;

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

				current->next = (msgField*)malloc(sizeof(msgField));
				msgField* next = current->next;
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

void initCrosMsg(cRosMsg* msg)
{
	msg->constants = (msgConst*) malloc(sizeof(msgConst));
	msg->constants->type = NULL;
	msg->constants->name = NULL;
	msg->constants->value = NULL;
	msg->constants->prev = NULL;
	msg->constants->next = NULL;
	msg->first_const = msg->constants;
	msg->fields = (msgField*) malloc(sizeof(msgField));
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

void initCrosSrv(cRosSrv* srv)
{
	srv->request = (cRosMsg*) malloc(sizeof(cRosMsg));
	initCrosMsg(srv->request);
	srv->response = (cRosMsg*) malloc(sizeof(cRosMsg));
	initCrosMsg(srv->response);
	srv->name = NULL;
	srv->package = NULL;
	srv->plain_text = NULL;
	srv->root_dir = NULL;
}

void initCrosDep(msgDep* dep)
{
	dep->msg = NULL;
	dep->prev = NULL;
	dep->next = NULL;
}

int loadFromFileMsg(char* filename, cRosMsg* msg)
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

int loadFromFileSrv(char* filename, cRosSrv* srv)
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

		//split before the first delim char
		*(srv_res - 1) = '\0';

		//move over the delimiter and the new line char
		srv_res += strlen(SRV_DELIMITER) + 1;
		srv_req = srv_text;

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

		srv->request->package = srv->package;
		srv->request->root_dir = srv->root_dir;
		srv->response->package = srv->package;
		srv->response->root_dir = srv->root_dir;
		loadFromStringMsg(srv_req, srv->request);
		loadFromStringMsg(srv_res, srv->response);
		free(srv_text);
	}

	free(file_tokenized);
	return EXIT_SUCCESS;
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
int getDependenciesMsg(cRosMsg* msg, msgDep* msgDeps)
{
	//move on until you reach the head
	while(msgDeps->next != NULL) msgDeps = msgDeps->next;
	msgDep* currentDep = msgDeps ;

	msgField* fields = msg->first_field;

	while(fields->next != NULL)
	{
		msgField* currentField = fields;
		char* type = base_msg_type(currentField->type);

		if(!is_builtin_type(type))
		{
			if(containsDep(msgDeps,type))
			{
				fields = fields->next;
				continue;
			}

			currentDep->msg = (cRosMsg*) malloc(sizeof(cRosMsg));
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

				currentDep->msg->fields = (msgField*) malloc(sizeof(msgField));
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

//	Compute dependencies of the specified message file
int getFileDependenciesMsg(char* filename, cRosMsg* msg, msgDep* deps)
{
	loadFromFileMsg(filename, msg);
	getDependenciesMsg(msg,deps);
	return EXIT_SUCCESS;
}

//	Compute dependencies of the specified service file
int getFileDependenciesSrv(char* filename, cRosSrv* srv, msgDep* deps)
{
	loadFromFileSrv(filename, srv);
	getDependenciesMsg(srv->request,deps);
	getDependenciesMsg(srv->response,deps);
	return EXIT_SUCCESS;
}

//	Compute full text of message, including text of embedded
//	types.  The text of the main msg is listed first. Embedded
//	msg files are denoted first by an 80-character '=' separator,
//	followed by a type declaration line,'MSG: pkg/type', followed by
//	the text of the embedded type.
char* computeFullTextMsg(cRosMsg* msg, msgDep* deps)
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

//	Compute full text of service, including text of embedded
//	types.  The text of the main srv is listed first. Embedded
//	srv files are denoted first by an 80-character '=' separator,
//	followed by a type declaration line,'MSG: pkg/type', followed by
//	the text of the embedded type.
char* computeFullTextSrv(cRosSrv* srv, msgDep* deps)
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

unsigned char* getMD5Msg(cRosMsg* msg)
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

	msgField* fields_it = msg->first_field;

	while(fields_it->next != NULL)
	{
		if(is_builtin_type(fields_it->type))
		{
			dynStringPushBackStr(&buffer,fields_it->type);
			dynStringPushBackStr(&buffer," ");
			dynStringPushBackStr(&buffer,fields_it->name);
			dynStringPushBackStr(&buffer,"\n");
		}
		else if(is_header_type(fields_it->type))
		{
			cRosMsg* msg = (cRosMsg*) malloc(sizeof(cRosMsg));
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
			unsigned char* res = cRosGentoolsMD5(filename_dep.data);
			cRosMD5Readable(res, &buffer);
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


void getMD5Txt(cRosMsg* msg, DynString* buffer)
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

	msgField* fields_it = msg->first_field;

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
			cRosMsg* msg = (cRosMsg*) malloc(sizeof(cRosMsg));
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
			unsigned char* res = cRosGentoolsMD5(filename_dep.data);
			cRosMD5Readable(res, buffer);
			dynStringPushBackStr(buffer," ");
			dynStringPushBackStr(buffer,fields_it->name);
			dynStringPushBackStr(buffer,"\n");
		}
		fields_it = fields_it->next;
	}
}

unsigned char* cRosGentoolsMD5(char* filename)
{

	char* filename_tokenized = (char*)malloc(strlen(filename)+1);
	strcpy(filename_tokenized, filename);
	strtok(filename_tokenized, ".");
	char* file_ext = strtok(NULL,".");

	if(strcmp(file_ext,FILEEXT_MSG) == 0)
	{
		cRosMsg* msg = (cRosMsg*) malloc(sizeof(cRosMsg));
		initCrosMsg(msg);
		loadFromFileMsg(filename,msg);
		return getMD5Msg(msg);
	}

	if(strcmp(file_ext,FILEEXT_SRV) == 0)
	{
		/*
		cRosSrv* srv = (cRosSrv*) malloc(sizeof(cRosSrv));
		initCrosSrv(srv);
		loadFromFileSrv(filename,srv);

		unsigned char* res = NULL;
		DynString buffer;
		dynStringInit(&buffer);

		res = getMD5Msg(srv->request);
		cRosMD5Readable(res, &buffer);
		res = getMD5Msg(srv->response);
		cRosMD5Readable(res, &buffer);
		unsigned char* result = (unsigned char*) malloc(16);
		MD5_CTX md5_t;
		MD5_Init(&md5_t);
		MD5_Update(&md5_t,buffer.data,buffer.len - 1);
		MD5_Final(result, &md5_t);
		return result;
		*/

		cRosSrv* srv = (cRosSrv*) malloc(sizeof(cRosSrv));
		initCrosSrv(srv);
		loadFromFileSrv(filename,srv);

		unsigned char* res = NULL;
		DynString buffer;
		dynStringInit(&buffer);

		MD5_CTX md5_t;
		MD5_Init(&md5_t);

		getMD5Txt(srv->request, &buffer);

		if(buffer.len != 0)
		{
			MD5_Update(&md5_t,buffer.data,buffer.len - 1);
			dynStringClear(&buffer);
		}

		getMD5Txt(srv->response, &buffer);
		MD5_Update(&md5_t,buffer.data,buffer.len - 1);

		unsigned char* result = (unsigned char*) malloc(16);
		MD5_Final(result, &md5_t);

		return result;
	}

	free(filename_tokenized);
	return NULL;
}

int cRosGentoolsSHA1(char* filename)
{
	//FILE * fp = fopen(filename, "r");
	//fclose(fp);
	return 0;
}

int cRosGentoolsFulltext(char* filename)
{
	char* full_text = NULL;;

	char* filename_tokenized = (char*)malloc(strlen(filename)+1);
	strcpy(filename_tokenized, filename);
	strtok(filename_tokenized, ".");
	char* file_ext = strtok(NULL,".");

	if(strcmp(file_ext,FILEEXT_MSG) == 0)
	{
		msgDep messageDependencies;
		cRosMsg msg;
		initCrosMsg(&msg);
		initCrosDep(&messageDependencies);
		getFileDependenciesMsg(filename, &msg, &messageDependencies);
		full_text = computeFullTextMsg(&msg, &messageDependencies);
	}

	if(strcmp(file_ext,FILEEXT_SRV) == 0)
	{
		msgDep messageDependencies;
		cRosSrv srv;
		initCrosSrv(&srv);
		initCrosDep(&messageDependencies);
		getFileDependenciesSrv(filename, &srv, &messageDependencies);
		full_text = computeFullTextSrv(&srv, &messageDependencies);
	}

	printf("%s",full_text);

	free(filename_tokenized);
	//free(full_text);
  return 1;
}
