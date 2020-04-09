#ifndef _CROS_SERVICE_INTERNAL_H_
#define _CROS_SERVICE_INTERNAL_H_

#include "cros_message.h"
#include "cros_message_internal.h"
#include "cros_err_codes.h"

static const char* FILEEXT_SRV = "srv";

// string that denotes the separation between request/response service parts
static const char* SRV_DELIMITER = "---";

struct t_srvDef
{
    char* name;
    char* package;
    char* root_dir;
    char* plain_text;
    cRosMessageDef* request;
    cRosMessageDef* response;
};

typedef struct t_srvDef cRosSrvDef;

cRosErrCodePack cRosServiceBuildInner(cRosMessage **request, cRosMessage **response, char **message_definition, char *md5sum, const char *srv_filepath);
cRosErrCodePack initCrosSrv(cRosSrvDef* srv);

cRosErrCodePack getFileDependenciesSrv(char* filename, cRosSrvDef* srv, msgDep* deps);

//  Compute full text of service, including text of embedded
//  types.  The text of the main srv is listed first. Embedded
//  srv files are denoted first by an 80-character '=' separator,
//  followed by a type declaration line,'MSG: pkg/type', followed by
//  the text of the embedded type.
char* computeFullTextSrv(cRosSrvDef* srv, msgDep* deps);

cRosErrCodePack loadFromFileSrv(const char *filename, cRosSrvDef* srv);

void cRosServiceDefFree(cRosSrvDef* service_def);

#endif // _CROS_SERVICE_INTERNAL_H_
