#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "cros_err_codes.h"

// Populate the error message list global variable using the messages defined before (ERROR_CODE_LIST_DEF) but
// using the format defined by MSG_COD_ELEM for them
#define MSG_COD_ELEM(code,msg) {code,msg},
struct cROSErrCodeListElem CROSErrCodeList[]=
{
  ERROR_CODE_LIST_DEF
  {LAST_ERR_LIST_CODE, NULL} // Last value of the list (marker)
};
#undef MSG_COD_ELEM

const char *cRosGetErrCodeStr(cROSErrCode err_code)
{
  int msg_ind;
  const char *ret_msg;

  // Search for a message in the error message list according to the specified message code
  ret_msg=NULL;
  for(msg_ind=0;CROSErrCodeList[msg_ind].code!=LAST_ERR_LIST_CODE && ret_msg==NULL;msg_ind++)
  {
    if(CROSErrCodeList[msg_ind].code == err_code) // We have found a message with the specified code
      ret_msg = CROSErrCodeList[msg_ind].msg; // Use the found message as return value
  }
  return ret_msg;
}

cROSErrCodePack cRosAddErrCodeIfErr(cROSErrCodePack prev_err_pack, cROSErrCode err_code)
{
  cROSErrCodePack new_err_pack;

  if(prev_err_pack == CROS_SUCCESS_ERR_PACK) // If no error stored in the error pack
    new_err_pack = prev_err_pack; // do not add any error code to the pack
  else // if we have some error code in the pack
    new_err_pack = (prev_err_pack << 8) | err_code; // add more info to the error pack: add the new code

  return new_err_pack;
}

cROSErrCodePack cRosRemoveLastErrCode(cROSErrCodePack prev_err_pack)
{
  cROSErrCodePack new_err_pack;

  new_err_pack = prev_err_pack >> 8; // Remove the last error code from the error pack (8 least-significant bits)

  return new_err_pack;
}

cROSErrCode cRosGetLastErrCode(cROSErrCodePack err_pack)
{
  cROSErrCode last_err_code;

  last_err_code = err_pack & 0xFFU; // Get the last error code from the error pack (8 least-significant bits)

  return last_err_code;
}

int cROSPrintErrCodePack(cROSErrCodePack err_cod_pack, const char *fmt_str, ...)
{
  int n_prn_chars;
  const char *msg_str;
  cROSErrCode curr_err_cod;

  n_prn_chars=0;

  if(fmt_str != NULL)
  {
    va_list arg_list;

    va_start(arg_list, fmt_str);
    n_prn_chars+=vprintf(fmt_str, arg_list);
    va_end(arg_list);
  }

  // Iterate throughout all the errors contained in the pack (up to 4)
  while((curr_err_cod=cRosGetLastErrCode(err_cod_pack)) != CROS_NO_ERR)
  {
    // Print error info included in err
    msg_str = cRosGetErrCodeStr(curr_err_cod);
    if(msg_str != NULL) // The error code has been found in the list
      n_prn_chars+=printf(". Err %hu: %s", curr_err_cod, msg_str);
    else
      n_prn_chars+=printf(". Error code %hu (The description string for this error code has not been found).", curr_err_cod);

    err_cod_pack = cRosRemoveLastErrCode(err_cod_pack); // Pass to the next error code
  }
  n_prn_chars+=printf("\n");
  return(n_prn_chars);
}

