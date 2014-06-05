#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "cros_gentools.h"
#include "md5.h"
#include "dyn_string.h"

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

int load_from_string_msg(char* text, cRosMsg* msg)
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
				current->type = entry_type;
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

		return 1;
//	return MsgSpec(types, names, constants, text, full_name, short_name, package_context)
}

void load_from_string_srv(char* filename)
{

}

int load_from_file(char* filename, cRosMsg* msg)
{
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

		char* filename_tokenized = (char*)malloc(strlen(filename)+1);
		strcpy(filename_tokenized, filename);
		strtok(filename_tokenized, ".");
		char* file_ext = strtok(NULL,".");

		if(strcmp(file_ext,FILEEXT_MSG) == 0)
		{
			load_from_string_msg(msg_text, msg);
		}

		if(strcmp(file_ext,FILEEXT_SRV) == 0)
		{
			load_from_string_srv(msg_text);
		}
		free(filename_tokenized);
		free(msg_text);
	}

	return EXIT_SUCCESS;
}

int is_header_type(char* type)
{
	return (strcmp(type, "std_msgs/Header") == 0) ||
			(strcmp(type, "Header") == 0) ||
			(strcmp(type, "roslib/Header") == 0);
}

int containsDep(msgDep* iterator, char* depName)
{
	while(iterator->next != NULL && iterator->msg != NULL)
	{
		if(strcmp(iterator->msg->name, depName) == 0)
			return 1;
		iterator = iterator->next;
	}
	return 0;
}

int add_msgs_depends(msgDep* msgDeps, cRosMsg* msg)//rospack, spec, deps, package_context)
{
/*
    Add the list of message types that spec depends on to depends.
    @param spec: message to compute dependencies for
    @type  spec: roslib.msgs.MsgSpec/roslib.srvs.SrvSpec
    @param deps [str]: list of dependencies. This list will be updated
    with the dependencies of spec when the method completes
    @type  deps: [str]
    @raise KeyError for invalid dependent types due to missing package dependencies.
    """
*/

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
				continue;
			}

			currentDep->msg = (cRosMsg*) malloc(sizeof(cRosMsg));
			// special mapping for header
			if(is_header_type(type))
			{
				//have to re-names Header
				currentDep->msg->name = (char*) malloc(strlen(HEADER_DEFAULT_TYPE) + 1);
				memcpy(currentDep->msg->name,"\0",1);
				strcpy(currentDep->msg->name,HEADER_DEFAULT_TYPE);

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
					char* trail = "geometry_msgs";
					currentDep->msg->name = (char*) malloc(strlen(trail) + strlen(type) + 1 + 1);
					memcpy(currentDep->msg->name, trail, strlen(trail) + 1);
					strcat(currentDep->msg->name, "/");
					strcat(currentDep->msg->name, type);
				}

				currentDep->msg->fields = (msgField*) malloc(sizeof(msgField));
				currentDep->msg->first_field = currentDep->msg->fields;
				currentDep->msg->constants = (msgConst*) malloc(sizeof(msgConst));
				currentDep->msg->first_const = currentDep->msg->constants;

				char* rootpath = "/home/nico/Desktop/test_msgs";
				char* path = (char*) malloc(strlen(rootpath) + strlen(currentDep->msg->name) + 1 + 1 + 4);
				memcpy(path, rootpath, strlen(rootpath) + 1);
				strcat(path, "/");
				strcat(path,currentDep->msg->name);
				strcat(path,".msg");

				load_from_file(path,currentDep->msg);
				msgDep* next = malloc(sizeof(msgDep));
				next->msg = NULL;
				next->prev = currentDep;
				currentDep->next = next;
				currentDep = next;
				add_msgs_depends(currentDep, currentDep->prev->msg);
			}
		}
		fields = fields->next;
	}

	//	for t in spec.types:
	//			t = roslib.msgs.base_msg_type(t)
	//			if not roslib.msgs.is_builtin(t):
	//					t_package, t_base = roslib.names.package_resource_name(t)
	//
	//					# special mapping for header
	//					if t == roslib.msgs.HEADER:
	//							# have to re-names Header
	//							deps.append(_header_type_name)
	//
	//					if roslib.msgs.is_registered(t):
	//							depspec = roslib.msgs.get_registered(t)
	//							if t != roslib.msgs.HEADER:
	//									if '/' in t:
	//											deps.append(t)
	//									else:
	//											deps.append(package_context+'/'+t)
	//					else:
	//							if valid_packages is None:
	//									valid_packages = _get_valid_packages(package_context, rospack)
	//							if t_package in valid_packages:
	//									# if we are allowed to load the message, load it.
	//									key, depspec = roslib.msgs.load_by_type(t, package_context)
	//									if t != roslib.msgs.HEADER:
	//										deps.append(key)
	//									roslib.msgs.register(key, depspec)
	//							else:
	//									# not allowed to load the message, so error.
	//									raise KeyError(t)
	//					_add_msgs_depends(rospack, depspec, deps, package_context)
	return EXIT_SUCCESS;
}

void get_dependencies(cRosMsg* msg, msgDep* deps)/*spec, package, compute_files=True, stdout=sys.stdout, stderr=sys.stderr, rospack=None)*/
{
//	Compute dependencies of the specified Msgs/Srvs
//	@param spec: message or service instance
//	@type  spec: L{roslib.msgs.MsgSpec}/L{roslib.srvs.SrvSpec}
//	@param package: package name
//	@type  package: str
//	@param stdout: (optional) stdout pipe
//	@type  stdout: file
//	@param stderr: (optional) stderr pipe
//	@type  stderr: file
//	@param compute_files: (optional, default=True) compute file
//	dependencies of message ('files' key in return value)
//	@type  compute_files: bool
//	@return: dict:
//		* 'files': list of files that \a file depends on
//		* 'deps': list of dependencies by type
//		* 'spec': Msgs/Srvs instance.
//		* 'uniquedeps': list of dependencies with duplicates removed,
//		* 'package': package that dependencies were generated relative to.
//	@rtype: dict
//
//
//	deps = []
//	try:
//			if not rospack:
//					rospack = rospkg.RosPack()

	/*char* filename_tokenized = (char*) malloc(strlen(filename) + 1);
	strcpy(filename_tokenized,filename);
	strtok(filename_tokenized, ".");
	char* file_ext = strtok(NULL,".");*/

	//if(strcmp(file_ext,FILEEXT_MSG) == 0)
	//{
		add_msgs_depends(deps, msg);
	//}

	//if(strcmp(file_ext,FILEEXT_SRV) == 0)
	//{
	//	add_msgs_depends(rospack, spec.request, deps, package)
	//	add_msgs_depends(rospack, spec.response, deps, package
	//}

//
//	# convert from type names to file names
//
//	if compute_files:
//			files = {}
//			for d in set(deps):
//					d_pkg, t = roslib.names.package_resource_name(d)
//					d_pkg = d_pkg or package # convert '' -> local package
//					files[d] = roslib.msgs.msg_file(d_pkg, t)
//	else:
//			files = None
//
//	# create unique dependency list
//	uniquedeps = []
//	for d in deps:
//			if not d in uniquedeps:
//					uniquedeps.append(d)
//
//	if compute_files:
//			return { 'files': files, 'deps': deps, 'spec': spec, 'package': package, 'uniquedeps': uniquedeps }
//	else:
//			return { 'deps': deps, 'spec': spec, 'package': package, 'uniquedeps': uniquedeps }
	//free(filename_tokenized);
}


//	Compute dependencies of the specified message/service file
cRosMsg* get_file_dependencies(char* filename, msgDep* deps)
{
	cRosMsg* msg = (cRosMsg*) malloc(sizeof(cRosMsg));
	msg->constants = (msgConst*) malloc(sizeof(msgConst));
	msg->first_const = msg->constants;
	msg->fields = (msgField*) malloc(sizeof(msgField));
	msg->first_field = msg->fields;

	load_from_file(filename, msg);
	get_dependencies(msg,deps);
	return msg;
}

//	Compute full text of message/service, including text of embedded
//	types.  The text of the main msg/srv is listed first. Embedded
//	msg/srv files are denoted first by an 80-character '=' separator,
//	followed by a type declaration line,'MSG: pkg/type', followed by
//	the text of the embedded type.
char* compute_full_text(cRosMsg* msg, msgDep* deps)
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

	full_size += strlen(msg->plain_text) + 1/*New line*/;

	while(deps->next != NULL)
	{
		//printf("%s\nMSG: %s\n", separator, deps->msg->name);
		full_size = strlen(deps->msg->plain_text) + strlen(separator) + strlen(msg_tag) + 2/*New lines*/;
		deps = deps->next;
	}

	//rollback
	while(deps->prev != NULL) deps = deps->prev;
	full_text = (char*) malloc(full_size + 1);
	memcpy(full_text,msg->plain_text,strlen(msg->plain_text) + 1);
	strcat(full_text,"\n");
	while(deps->next != NULL)
	{
		//printf("%s\nMSG: %s\n", separator, deps->msg->name);
		strcat(full_text, separator);
		strcat(full_text, "\n");
		strcat(full_text, msg_tag);
		strcat(full_text, deps->msg->plain_text);
		strcat(full_text, "\n");
		deps = deps->next;
	}
	return full_text;
}

char* cRosGentoolsMD5(char* filename)
{
	DynString buffer;
	dynStringInit(&buffer);
	unsigned char* result = (unsigned char*) malloc(16);
	int i;

	msgDep* messageDependencies = (msgDep*)malloc(sizeof(msgDep));
	messageDependencies->msg = NULL;
	messageDependencies->prev = NULL;
	messageDependencies->next = NULL;
	cRosMsg* msg = get_file_dependencies(filename, messageDependencies);
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

		}
		else
		{
			DynString filename_dep;
			dynStringInit(&filename_dep);
			dynStringPushBackStr(&filename_dep,"/home/nico/Desktop/test_msgs/geometry_msgs/");
			dynStringPushBackStr(&filename_dep,fields_it->type);
			dynStringPushBackStr(&filename_dep,".msg");
			char* res = cRosGentoolsMD5(filename_dep.data);
			char val[5];
			for(i = 0; i < 16; i++)
			{
			  snprintf(val, 4, "%02x", (unsigned char) res[i]);
				dynStringPushBackStr(&buffer,val);
			}
			dynStringPushBackStr(&buffer," ");
			dynStringPushBackStr(&buffer,fields_it->name);
			dynStringPushBackStr(&buffer,"\n");
		}
		fields_it = fields_it->next;
	}

	MD5_CTX md5_t;
	MD5_Init(&md5_t);
	MD5_Update(&md5_t,buffer.data,buffer.len - 1);
	MD5_Final(result, &md5_t);

	printf("\n");
	for(i = 0; i < 16; i++)
	{
	  printf("%02x",result[i]);
	}
	printf("\n");

	return (char*)result;
}

int cRosGentoolsSHA1(char* filename)
{
	//FILE * fp = fopen(filename, "r");
	//fclose(fp);
	return 0;
}

int cRosGentoolsFulltext(char* filename)
{
	// Try with /opt/ros/groovy/share/nav_msgs/srv/GetPlan.srv

	msgDep* messageDependencies = (msgDep*)malloc(sizeof(msgDep));
	messageDependencies->msg = NULL;
	messageDependencies->prev = NULL;
	messageDependencies->next = NULL;
	cRosMsg* msg = get_file_dependencies(filename, messageDependencies);
	char* full_text = compute_full_text(msg, messageDependencies);
	printf("%s",full_text);
  return 1;
}
