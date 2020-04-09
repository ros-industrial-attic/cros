/// \file cros_err_codes.h
/// \brief This header file define the codes for the error messages of the cROS library.
/// The functions for managing these codes are declared as well.
///
/// \author Richard R. Carrillo, Aging in Vision and Action lab, Institut de la Vision, Sorbonne University, Paris, France.

#ifndef __CROS_ERR_CODES_H__
#define __CROS_ERR_CODES_H__
#include <stdint.h>

///////////////////////////////// ERROR messages /////////////////////////////////
//! Definition of all the error codes. One code per line. Up to 255 error codes
#define ERROR_CODE_LIST_DEF \
  MSG_COD_ELEM(CROS_NO_ERR, "Operation completed successfully") \
  MSG_COD_ELEM(CROS_UNSPECIFIED_ERR, "An unspecified error occurred") \
  MSG_COD_ELEM(CROS_MEM_ALLOC_ERR, "Error allocating memory") \
  MSG_COD_ELEM(CROS_BAD_PARAM_ERR, "An invalid value has been specified for at least one of the input parameters") \
  MSG_COD_ELEM(CROS_OPEN_MSG_FILE_ERR, "The message file definition the cannot be opened") \
  MSG_COD_ELEM(CROS_READ_MSG_FILE_ERR, "Error reading the message definition file") \
  MSG_COD_ELEM(CROS_LOAD_MSG_FILE_ERR, "Error loading or processing the message definition file") \
  MSG_COD_ELEM(CROS_OPEN_SVC_FILE_ERR, "The file defining the service cannot be opened") \
  MSG_COD_ELEM(CROS_READ_SVC_FILE_ERR, "Error reading the service definition file") \
  MSG_COD_ELEM(CROS_UNREG_TIMEOUT_ERR, "The unregistration from the ROS master was abandoned before it finished because it was taking too long") \
  MSG_COD_ELEM(CROS_SELECT_FD_ERR, "An error ocurred while monitoring the socket file descriptors (select() function)") \
  MSG_COD_ELEM(CROS_MANY_PARAM_ERR, "The maximum number of paramter subscriptions has been reached") \
  MSG_COD_ELEM(CROS_PARAM_SUB_IND_ERR, "The provided parameter subscriber index does not corresponds to a valid subscriber to be unsubscribed") \
  MSG_COD_ELEM(CROS_TOPIC_PUB_IND_ERR, "The provided topic publisher index does not corresponds to a valid publisher to be unregistered") \
  MSG_COD_ELEM(CROS_TOPIC_SUB_IND_ERR, "The provided topic subscriber index does not corresponds to a valid subscriber to be unregistered") \
  MSG_COD_ELEM(CROS_SVC_FILE_DELIM_ERR, "The delimiter string between resquest and response definition could not be found in the service definition file") \
  MSG_COD_ELEM(CROS_FILE_ENTRY_TYPE_ERR, "The definition file contains an entry that specifies and incorrect data type") \
  MSG_COD_ELEM(CROS_LOAD_SVC_FILE_REQ_ERR, "Error loading the service request definition in the file") \
  MSG_COD_ELEM(CROS_LOAD_SVC_FILE_RES_ERR, "Error loading the service response definition in the file") \
  MSG_COD_ELEM(CROS_CREATE_CUSTOM_MSG_ERR, "Error loading the specified custom message definition file or creating a message of its type") \
  MSG_COD_ELEM(CROS_FILE_ENTRY_NO_SEP_ERR, "The definition file contains an entry that is syntactically incorrect (a white space is expected between data type and data name)") \
  MSG_COD_ELEM(CROS_DEPACK_INSUFF_DAT_ERR, "Error decoding a received packet: The length of the packet is too small for the expected data type") \
  MSG_COD_ELEM(CROS_DEPACK_NO_MSG_DEF_ERR, "Error decoding a received packet: The definition of the custom message does not corresponds to the fields of the message to send") \
  MSG_COD_ELEM(CROS_TOP_PUB_CALLBACK_ERR, "The callback function specified for a topic publisher returned a non-zero value") \
  MSG_COD_ELEM(CROS_TOP_SUB_CALLBACK_ERR, "The callback function specified for a topic subscriber returned a non-zero value") \
  MSG_COD_ELEM(CROS_SVC_REQ_CALLBACK_ERR, "The callback function specified for a service client returned a non-zero value when generating the service request") \
  MSG_COD_ELEM(CROS_SVC_RES_CALLBACK_ERR, "The callback function specified for a service client returned a non-zero value when generating the service response") \
  MSG_COD_ELEM(CROS_SVC_SER_CALLBACK_ERR, "The callback function specified for a service server returned a non-zero value") \
  MSG_COD_ELEM(CROS_SVC_RES_OK_BYTE_ERR, "The response received from the service server contains an 'ok' byte codifying a value different from true (1)") \
  MSG_COD_ELEM(CROS_RCV_TOP_TIMEOUT_ERR, "The specified timeout was up while waiting for a topic message") \
  MSG_COD_ELEM(CROS_SEND_TOP_TIMEOUT_ERR, "The specified timeout was up while waiting for space in the message transmission queue") \
  MSG_COD_ELEM(CROS_CALL_SVC_TIMEOUT_ERR, "The specified timeout was up while waiting for the service call response") \
  MSG_COD_ELEM(CROS_CALL_INI_TIMEOUT_ERR, "The specified timeout was up while the service caller was waiting for the previous call to finish") \
  MSG_COD_ELEM(CROS_XMLRPC_CLI_CONN_ERR, "An error occurred when an XMLRPC client process was trying to establish the connection to the target address") \
  MSG_COD_ELEM(CROS_XMLRPC_CLI_REFUS_ERR, "An XMLRPC client process could not establish the connection to the target address because the connection was refused") \
  MSG_COD_ELEM(CROS_XMLRPC_CLI_WRITE_ERR, "An error arised when the XMLRPC client process tried to write the request on the socket") \
  MSG_COD_ELEM(CROS_XMLRPC_CLI_READ_ERR, "An error arised when the XMLRPC client process tried to read the response from the socket") \
  MSG_COD_ELEM(CROS_TCPROS_CLI_CONN_ERR, "A TCPROS client process could not establish the connection to the target address due to an error") \
  MSG_COD_ELEM(CROS_TCPROS_CLI_REFUS_ERR, "An TCPROS client process could not establish the connection to the target address because the connection was refused") \
  MSG_COD_ELEM(CROS_RPCROS_CLI_CONN_ERR, "A RPCROS client process could not establish the connection to the target address due to an error") \
  MSG_COD_ELEM(CROS_RPCROS_CLI_REFUS_ERR, "An RPCROS client process could not establish the connection to the target address because the connection was refused") \
  MSG_COD_ELEM(CROS_SOCK_OPEN_TIMEOUT_ERR, "The specified timeout was up while waiting for the specified port to be open") \
  MSG_COD_ELEM(CROS_SOCK_OPEN_CONN_ERR, "An error occurred when the specified target port was tried to be connected (target address could not be resolved?)") \
  MSG_COD_ELEM(CROS_EXTRACT_MSG_INT_ERR, "An internal error occurred when sending an inmediate message: The message could not be extracted from the queue") \
  MSG_COD_ELEM(LAST_ERR_LIST_CODE, "") // Sentinel code used to mark the last element of the global error list

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


/*! \brief Locates the message string corresponding to the specified error code in the global error message list.
 *
 * The function locates the message string describing the specified error code (a single error code, not a pack).
 * \param err_code number of the error
 * \return Pointer to the message string. If no message is found for the specified code, it returns NULL
 */
const char *cRosGetErrCodeStr(cRosErrCode err_code);

/*! \brief Add a new error code to the specified error code pack.
 *
 * This function is intended to add more information (a new error code) to an error code pack.
 * If the new error code is CROS_NO_ERR (0), nothing is added, that is, the prev_err_pack is returned.
 * parameter list.
 * \param prev_err_pack is the original error code pack.
 * \param err_code number of the new error.
 * \return the new error pack (containing the specified err_code if err_code was not CROS_NO_ERR)
 */
cRosErrCodePack cRosAddErrCode(cRosErrCodePack prev_err_pack, cRosErrCode err_code);

/*! \brief Add a new error code to the specified non-empty error code pack.
 *
 * This function is intended to add more information (a new error code) to an error code pack that already contains errors.
 * If the specified error code pack does not contain any error, this function returns the same error code pack specified in the
 * parameter list.
 * \param prev_err_pack is the original error code pack.
 * \param err_code number of the new error. If this code is CROS_NO_ERR, prev_err_pack is returned
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

/*! \brief Compile the error codes contained in two error code packs into a single error code pack.
 *
 * This function is intended to add new error codes to an error code pack that already may contain errors.
 * If the specified error code packs do not contain any error, this function returns an empty code pack (CROS_SUCCESS_ERR_PACK).
 * \param prev_err_pack_0 is the first error code pack.
 * \param prev_err_pack_1 is the second error code pack.
 * \return the new error pack joining the two error packs
 */
cRosErrCodePack cRosAddErrCodePackIfErr(cRosErrCodePack prev_err_pack_0, cRosErrCodePack prev_err_pack_1);

/*! \brief Composes and print the error message corresponding to the specified error code pack.
 *
 *  The function search for the error strings indicated in the specified error pack. The composed message is printed
 *  in the output file stream. The error pack can contain up to 4 error codes, so up to 4 message string can be printed.
 *  Additionally the text string specified by fmt_str is printed before the error strings. This string can be used to
 *  supply the user with context information about the error.
 *  \param err structure encoding the occurred error (usually returned by a failing function).
 *  \param fmt_str string that specifies how subsequent arguments are printed. It has the same format as printf
 *         function.
 * \return the number of characters written in the output stream.
 */
int cRosPrintErrCodePack(cRosErrCodePack err_cod_pack, const char *fmt_str, ...);

/*! \brief Composes the error message corresponding to the specified error code pack and writes it to the specified buffer.
 *
 *  The function search for the error strings indicated in the specified error pack. The error pack can contain up to 4 error
 *  codes, so up to 4 message string can be printed.
 *  Additionally the text string specified by fmt_str is printed before the error strings. This string can be used to
 *  supply the user with context information about the error.
 * If not enough space if available in the output buffer the output is truncated and a '\0' is added at the end of the buffer.
 *  \param out_str_buf Pointer to the buffer where the output string will be stored.
 *  \param out_str_buf_len Length of the output buffer.
 *  \param err structure encoding the occurred error (usually returned by a failing function).
 *  \param fmt_str string that specifies how subsequent arguments are printed. It has the same format as printf
 *         function.
 * \return the number of characters that would be written to the output buffer if it had enough space.
 */
int cRosErrCodePackStr(char *out_str_buf, size_t out_str_buf_len, cRosErrCodePack err_cod_pack, const char *fmt_str, ...);

#endif /* __CROS_ERR_CODES_H__ */
