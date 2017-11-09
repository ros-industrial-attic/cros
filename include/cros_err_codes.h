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
  MSG_COD_ELEM(CROS_UNSPECIFIED_ERR, "An unspecified error occurred") \
  MSG_COD_ELEM(CROS_MEM_ALLOC_ERR, "Error allocating memory") \
  MSG_COD_ELEM(CROS_BAD_PARAM_ERR, "An invalid value has been specified for at least one of the input parameters") \
  MSG_COD_ELEM(CROS_OPEN_MSG_FILE_ERR, "The file defining the topic message cannot be opened") \
  MSG_COD_ELEM(CROS_LOAD_MSG_FILE_ERR, "Error loading the message difinition file") \
  MSG_COD_ELEM(CROS_OPEN_SVC_FILE_ERR, "The file defining the service cannot be opened") \
  MSG_COD_ELEM(CROS_UNREG_TIMEOUT_ERR, "The unregistration from the ROS master was abandoned before it finished because was taking too long") \
  MSG_COD_ELEM(CROS_SELECT_FD_ERR, "An error ocurred while monitoring the socket file descriptors (select() function)")

#define CROS_SUCCESS_ERR_PACK 0U //! Function return value indicating success

// Declare the enum data type used for encoding error codes and declare the error code for errors (CROS_NO_ERR,...)
#define MSG_COD_ELEM(code,msg) code,
enum cRosErrorCode { ERROR_CODE_LIST_DEF };
#undef MSG_COD_ELEM

typedef enum cRosErrorCode cRosErrCode;
// Declare an entry type of the message list variable that will contain the global list of error messages
struct cRosErrCodeListElem
{
  cRosErrCode code;
  const char *msg;
};

//! Error data type returned by many cROS library functions. It is a pack that can contain up to 4 accumulated error codes (uint8_t)
typedef uint32_t cRosErrCodePack;

// Sentinel code used to mark the last element of the global error list
#define LAST_ERR_LIST_CODE 255

/*! \brief Locates the message string corresponding to the specified error code in the global error message list.
 *
 * The function first to locate the message string describing the specified error code (a single error code, not a pack).
 * \param msg_list is pointer to a array of structures where the message is searched for
 * \param err_code number of the error
 * \return Pointer to the message string. If no message is found for the specified code, it returns NULL
 */
const char *cRosGetErrCodeStr(cRosErrCode err_code);

/*! \brief Add a new error code to the specified error code pack.
 *
 * This function is intended to add more information (a new error code) to an error code pack that already contains errors.
 * If the specified error code pack does not contain any error, this function returns the same error code pack specified in the
 * parameter list.
 * \param prev_err_pack is the original error code pack.
 * \param err_code number of the new error
 * \return the new error pack (containing the specified err_code if prev_err_pack was not initially CROS_SUCCESS_ERR_PACK)
 */
cRosErrCodePack cRosAddErrCodeIfErr(cRosErrCodePack prev_err_pack, cRosErrCode err_code);

/*! \brief Remove the last error code inserted in the specified error code pack.
 *
 * \param err_pack the original error code pack.
 * \return the new error code pack with the last code removed, or the original pack if no error code was contained in it.
 */
cRosErrCodePack cRosRemoveLastErrCode(cRosErrCodePack prev_err_pack);

/*! \brief Obtain the last error code inserted in the specified error code pack.
 *
 * \param err_pack the error code pack.
 * \return the last error code of the pack or CROS_NO_ERR (0) if no error is codified in the pack
 */
cRosErrCode cRosGetLastErrCode(cRosErrCodePack err_pack);

/*! \brief Composes and print the error message corresponding to the specified error code pack
 *
 *  The function search for the error strings indicated in the specified error pack. The composed message is printed
 *  in the console. The error pack can contain up to 4 error codes, so up to 4 message string can be printed.
 *  Additionally the text string specified by fmt_str is printed before the error strings. This string can be used to
 *  supply the user with context information about the error.
 *  \param err structure encoding the occurred error (usually returned by a failing function)
 * \return the number of characters written in the console
 */
int cRosPrintErrCodePack(cRosErrCodePack err_cod_pack, const char *fmt_str, ...);

#endif /* __CROS_ERR_CODES_H__ */