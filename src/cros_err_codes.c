#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "cros_node.h"
#include "cros_err_codes.h"

// Populate the error message list global variable using the messages defined before (ERROR_CODE_LIST_DEF) but
// using the format defined by MSG_COD_ELEM for them.
// This an array of structures where the error messages are searched for.
#define MSG_COD_ELEM(code,msg) {code,msg},
struct cRosErrCodeListElem CRosErrCodeList[]=
{
  ERROR_CODE_LIST_DEF
  {LAST_ERR_LIST_CODE, NULL} // Last value of the list not used
};
#undef MSG_COD_ELEM

const char *cRosGetErrCodeStr(cRosErrCode err_code)
{
  int msg_ind;
  const char *ret_msg;

  // Search for a message in the error message list according to the specified message code
  ret_msg=NULL;
  for(msg_ind=0;CRosErrCodeList[msg_ind].code!=LAST_ERR_LIST_CODE && ret_msg==NULL;msg_ind++)
  {
    if(CRosErrCodeList[msg_ind].code == err_code) // We have found a message with the specified code
      ret_msg = CRosErrCodeList[msg_ind].msg; // Use the found message as return value
  }
  return ret_msg;
}

cRosErrCodePack cRosAddErrCode(cRosErrCodePack prev_err_pack, cRosErrCode err_code)
{
  cRosErrCodePack new_err_pack;

  if(err_code == CROS_NO_ERR) // If no new error:
    new_err_pack = prev_err_pack; // do not add any error code to the pack
  else // if we have some new error code
    new_err_pack = (prev_err_pack << 8) | err_code; // add more info to the error pack: add the new code

  return new_err_pack;
}

cRosErrCodePack cRosAddErrCodeIfErr(cRosErrCodePack prev_err_pack, cRosErrCode err_code)
{
  cRosErrCodePack new_err_pack;

  if(prev_err_pack == CROS_SUCCESS_ERR_PACK) // If no error stored in the error pack
    new_err_pack = prev_err_pack; // do not add any error code to the pack
  else // if we had some error code in the pack
    new_err_pack = cRosAddErrCode(prev_err_pack, err_code); // add more info to the error pack: add the new code

  return new_err_pack;
}

cRosErrCodePack cRosRemoveLastErrCode(cRosErrCodePack prev_err_pack)
{
  cRosErrCodePack new_err_pack;

  new_err_pack = prev_err_pack >> 8; // Remove the last error code from the error pack (8 least-significant bits)

  return new_err_pack;
}

cRosErrCode cRosGetLastErrCode(cRosErrCodePack err_pack)
{
  cRosErrCode last_err_code;

  last_err_code = err_pack & 0xFFU; // Get the last error code from the error pack (8 least-significant bits)

  return last_err_code;
}

cRosErrCodePack cRosAddErrCodePackIfErr(cRosErrCodePack prev_err_pack_0, cRosErrCodePack prev_err_pack_1)
{
  cRosErrCodePack new_err_code_pack;
  cRosErrCode curr_err_cod;

  // First, add the error codes from prev_err_pack_0 to the output
  new_err_code_pack = prev_err_pack_0;
  // Then, add the error codes from prev_err_pack_1 to the output:
  // Iterate throughout all the errors contained in prev_err_pack_1
  while((curr_err_cod=cRosGetLastErrCode(prev_err_pack_1)) != CROS_NO_ERR)
  {
    new_err_code_pack = cRosAddErrCode(new_err_code_pack, curr_err_cod);
    prev_err_pack_1 = cRosRemoveLastErrCode(prev_err_pack_1); // Pass to the next error code
  }

  return new_err_code_pack;
}

int cRosPrintErrCodePack(cRosErrCodePack err_cod_pack, const char *fmt_str, ...)
{
  int n_prn_chars;
  const char *msg_str;
  cRosErrCode curr_err_cod;
  FILE *msg_out = cRosOutStreamGet();

  n_prn_chars=0;

  if(fmt_str != NULL)
  {
    va_list arg_list;

    va_start(arg_list, fmt_str);
    n_prn_chars+=vfprintf(msg_out, fmt_str, arg_list);
    va_end(arg_list);
  }

  // Iterate throughout all the errors contained in the pack (up to 4)
  while((curr_err_cod=cRosGetLastErrCode(err_cod_pack)) != CROS_NO_ERR)
  {
    // Print error info included in err
    msg_str = cRosGetErrCodeStr(curr_err_cod);
    if(msg_str != NULL) // The error code has been found in the list
      n_prn_chars+=fprintf(msg_out, ". Err %u: %s", curr_err_cod, msg_str);
    else
      n_prn_chars+=fprintf(msg_out, ". Error code %u (The description string for this error code has not been found).", curr_err_cod);

    err_cod_pack = cRosRemoveLastErrCode(err_cod_pack); // Pass to the next error code
  }
  n_prn_chars+=fprintf(msg_out, "\n");
  return(n_prn_chars);
}

int cRosErrCodePackStr(char *out_str_buf, size_t out_str_buf_len, cRosErrCodePack err_cod_pack, const char *fmt_str, ...)
{
  int n_prn_chars;
  const char *msg_str;
  cRosErrCode curr_err_cod;

  n_prn_chars=0;
  if(out_str_buf_len == 0)
    return(n_prn_chars);

  out_str_buf[0]='\0';

  if(fmt_str != NULL)
  {
    va_list arg_list;

    va_start(arg_list, fmt_str);
    n_prn_chars+=vsnprintf(out_str_buf+strlen(out_str_buf), out_str_buf_len-strlen(out_str_buf), fmt_str, arg_list);
    va_end(arg_list);
  }

  // Iterate throughout all the errors contained in the pack (up to 4)
  while((curr_err_cod=cRosGetLastErrCode(err_cod_pack)) != CROS_NO_ERR)
  {
    // Print error info included in err
    msg_str = cRosGetErrCodeStr(curr_err_cod);
    if(msg_str != NULL) // The error code has been found in the list
      n_prn_chars+=snprintf(out_str_buf+strlen(out_str_buf), out_str_buf_len-strlen(out_str_buf), ". Err %u: %s", curr_err_cod, msg_str);
    else
      n_prn_chars+=snprintf(out_str_buf+strlen(out_str_buf), out_str_buf_len-strlen(out_str_buf), ". Error code %u (The description string for this error code has not been found).", curr_err_cod);

    err_cod_pack = cRosRemoveLastErrCode(err_cod_pack); // Pass to the next error code
  }

  return(n_prn_chars);
}

