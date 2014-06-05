#ifndef _CROS_GENTOOLS_H_
#define _CROS_GENTOOLS_H_

/*! \defgroup cros_gentools cROS message generation tool
 * 
 * Implemenation of the  ROS message generation tool and
 * dependency resolutor (gentools and gendeps)
 */

/*! \addtogroup cros_gentool
 *  @{
 */

static const char* FILEEXT_SRV = "srv";
static const char* FILEEXT_MSG = "msg";

// e.g. std_msgs/String
static const char* CHAR_SEP = "/";

// character that designates a constant assignment rather than a field
static const char* CHAR_CONST = "=";
static const char* CHAR_COMMENT = "#";

static const char* HEADER_DEFAULT_TYPE = "std_msgs/Header";

static const char* HEADER_DEFAULT_TYPEDEF = "\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data\n\
# in a particular coordinate frame.\n\
#\n\
# sequence ID: consecutively increasing ID\n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
";

struct t_msgField{
	char* type;
	char* name;
	struct t_msgField* prev;
	struct t_msgField* next;
};

typedef struct t_msgField msgField;

struct t_msgConst{
	char* type;
	char* name;
	char* value;
	struct t_msgConst* prev;
	struct t_msgConst* next;
};

typedef struct t_msgConst msgConst;

struct t_msg{
	char* name;
	char* plain_text;
	msgField* fields;
	msgField* first_field;
	msgConst* constants;
	msgConst* first_const;
};

typedef struct t_msg cRosMsg;

struct t_msgDep{
	cRosMsg* msg;
	struct t_msgDep* prev;
	struct t_msgDep* next;
};

typedef struct t_msgDep msgDep;

static const char* PRIMITIVE_TYPES[] = {
		"int8","uint8","int16","uint16","int32","uint32","int64","uint64","float32","float64",
    "string", "bool",
    //deprecated:
    "char","byte",
		//time and duration
    "time","duration"};

/*! \brief Generate md5 hash of files
 * 
 *  \param filename Full path of the message/service file
 */
char* cRosGentoolsMD5(char* filename);


/*! \brief Generate SHA1 hash of files
 * 
 *  \param filename Full path of the message/service file
 * 
 *  \return Returns 1 on success, 0 on failure
 */
int cRosGentoolsSHA1(char* filename);

/*! \brief Generate concatenated list of files
 * 
 *  \param filename Full path of the message/service file
 *
 */
int cRosGentoolsFulltext(char* filename);

/*! @}*/

#endif
