#ifndef _XMLRPC_PARAMS_VECTOR_H_
#define _XMLRPC_PARAMS_VECTOR_H_

#include "xmlrpc_params.h"

/*! \defgroup xmlrpc_param_vector XMLRPC parameters vector */

/*! \addtogroup xmlrpc_param_vector 
 *  @{
 */

/*! \brief Dynamic vector object for XMLRPC parameters. Don't modify its internal members directly: 
 *         use the related functions instead */
typedef struct XmlrpcParamVector XmlrpcParamVector;
struct XmlrpcParamVector
{
  int size;                    //! Current vector size
  int max;                     //! Max vector size
  XmlrpcParam *data;           //! buffer data
};

/*! \brief Initialize a dynamic vector 
 * 
 *  \param p_vec Pointer to a XmlrpcParamVector object to be initialized
 */
void xmlrpcParamVectorInit( XmlrpcParamVector *p_vec );

/*! \brief Release a dynamic vector. It also release all the internal 
 *         data dynamically allocated (e.g., string and arrays) calling 
 *         the xmlrpcParamReleaseData() function
 * 
 *  \param p_vec Pointer to a XmlrpcParamVector object to be be released
 */
void xmlrpcParamVectorRelease( XmlrpcParamVector *p_vec );

/*! \brief Append a new XMLRPC boolean parameter to the vector. 
 * 
 *  \param p_vec Pointer to a XmlrpcParamVector object 
 *  \param val An integer value (false if it is equalt to 0, true otherwise)
 * 
 *  \return The new dynamic vector size, or -1 on failure
 */
int xmlrpcParamVectorPushBackBool( XmlrpcParamVector *p_vec, int val );

/*! \brief Append a new XMLRPC integer parameter to the vector. 
 * 
 *  \param p_vec Pointer to a XmlrpcParamVector object 
 *  \param val An integer value
 * 
 *  \return The new dynamic vector size, or -1 on failure
 */
int xmlrpcParamVectorPushBackInt( XmlrpcParamVector *p_vec, int32_t val );

/*! \brief Append a new XMLRPC double floating point parameter to the vector. 
 * 
 *  \param p_vec Pointer to a XmlrpcParamVector object 
 *  \param val A double value
 * 
 *  \return The new dynamic vector size, or -1 on failure
 */
int xmlrpcParamVectorPushBackDouble( XmlrpcParamVector *p_vec, double val );

/*! \brief Append a new XMLRPC string parameter to the vector. 
 *         The string is copied inside a dynamically allocated memory
 * 
 *  \param p_vec Pointer to a XmlrpcParamVector object 
 *  \param val Pointer to a string
 * 
 *  \return The new dynamic vector size, or -1 on failure
 */
int xmlrpcParamVectorPushBackString( XmlrpcParamVector *p_vec, const char *val );

/*! \brief Append a new (empty) XMLRPC array parameter to the vector,
 *         and start to allocate internal memory for the array  
 * 
 *  \param p_vec Pointer to a XmlrpcParamVector object 
 * 
 *  \return The new dynamic vector size, or -1 on failure
 */
int xmlrpcParamVectorPushBackArray( XmlrpcParamVector *p_vec );

/*! \brief Append a new (empty) XMLRPC struct parameter to the vector,
 *         and start to allocate internal memory
 *
 *  \param p_vec Pointer to a XmlrpcParamVector object
 *
 *  \return The new dynamic vector size, or -1 on failure
 */
int xmlrpcParamVectorPushBackStruct( XmlrpcParamVector *p_vec );

/*! \brief Append a new XMLRPC parameter to the vector. 
 *         Warning: data as string and arrays ARE NOT copied, i.e. only references are copyed.
 * 
 *  \param p_vec Pointer to a XmlrpcParamVector object 
 *  \param param Pointer to the XMLRPC parameter to be appended
 * 
 *  \return The new dynamic vector size, or -1 on failure
 */
int xmlrpcParamVectorPushBack( XmlrpcParamVector *p_vec, XmlrpcParam *param );

/*! \brief Get the current dynamic vector size
 * 
 *  \param p_vec Pointer to a XmlrpcParamVector object 
 * 
 *  \return The current dynamic vector size
 */
int xmlrpcParamVectorGetSize( XmlrpcParamVector *p_vec );

/*! \brief Return a reference to a XMLRPC parameter 
 * 
 *  \param p_vec Pointer to a XmlrpcParamVector object 
 *  \param pos Index of the parameter
 * 
 *  \return Pointer to the pos-th XMLRPC parameter 
 */
XmlrpcParam *xmlrpcParamVectorAt( XmlrpcParamVector *p_vec, int pos );

/*! \brief Print a vector of XMLRPC parameters to stdout in human readable form
 * 
 *  \param p_vec Pointer to a XmlrpcParamVector object 
 */
void xmlrpcParamVectorPrint( XmlrpcParamVector *p_vec );

/*! @}*/

#endif