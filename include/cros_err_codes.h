/// \file cros_err_codes.h
/// \brief This header file define the codes for the error messages of the cROS library.
/// The functions for managing these codes are declared as well.
///
/// \author Richard R. Carrillo, Aging in Vision and Action lab, Institut de la Vision, Sorbonne University, Paris, France.

#ifndef __CROS_ERR_CODES_H__
#define __CROS_ERR_CODES_H__
#include <stdint.h>

///////////////////////////////// ERROR messages /////////////////////////////////
//! Definition of all the error code. One code per line. Up to 256 error code
#define ERROR_CODE_LIST_DEF \
  MSG_COD_ELEM(CROS_NO_ERR, "Operation completed successfully") \
  MSG_COD_ELEM(CROS_UNKNOWN_ERR, "An unspecified error occurred") \
  MSG_COD_ELEM(CORS_MEM_ALLOC_ERR, "Error allocating memory")

#define CROS_SUCCESS_ERR_PACK 0U //! Function return value indicating success

// Declare the enum data type used for encoding error codes and declare the error code for errors (CROS_NO_ERR,...)
#define MSG_COD_ELEM(code,msg) code,
enum cROSErrorCode { ERROR_CODE_LIST_DEF };
#undef MSG_COD_ELEM

typedef enum cROSErrorCode cROSErrCode;
// Declare an entry type of the message list variable that will contain the global list of error messages
struct cROSErrCodeListElem
{
  cROSErrCode code;
  const char *msg;
};

//! Error data type returned by many cROS library functions. It is a pack that can contain up to 4 accumulated error codes (uint8_t)
typedef uint32_t cROSErrCodePack;

// Sentinel code used to mark the last element of the global error list
#define LAST_ERR_LIST_CODE 255

/*! \brief Locates the message string corresponding to the specified error code in the global error message list.
 *
 * The function first to locate the message string describing the specified error code (a single error code, not a pack).
 * \param msg_list is pointer to a array of structures where the message is searched for
 * \param err_code number of the error
 * \return Pointer to the message string. If no message is found for the specified code, it returns NULL
 */
const char *cRosGetErrCodeStr(cROSErrCode err_code);

/*! \brief Add a new error code to the specified error code pack.
 *
 * This function is intended to add more information (a new error code) to an error code pack that already contains errors.
 * If the specified error code pack does not contain any error, this function returns the same error code pack specified in the
 * parameter list.
 * \param prev_err_pack is the original error code pack.
 * \param err_code number of the new error
 * \return the new error pack (containing the specified err_code if prev_err_pack was not initially CROS_SUCCESS_ERR_PACK)
 */
cROSErrCodePack cRosAddErrCodeIfErr(cROSErrCodePack prev_err_pack, cROSErrCode err_code);

/// \brief Composes the error message corresponding to the specified error structure and tries to print if to the console and to the log file
///
/// The function search for the specified strings indicated in the specified error structure. The composed message is printed
/// in the console if Console_output global variable is different from 0. The message is also written to the log file if
/// the log has been previously initialized (function log_open() has been called).
/// \param err structure encoding the occurred error (usually returned by a failing function)
/// \return the number of characters written in the console
int cROSPrintErrCodePack(cROSErrCodePack err_cod_pack, const char *fmt_str, ...);

#endif /* __CROS_ERR_CODES_H__ */
