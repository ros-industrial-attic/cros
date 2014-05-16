#ifndef _DYN_BUFFER_H_
#define _DYN_BUFFER_H_

#include <stdint.h>

/*! \defgroup dyn_buffer Dynamic buffer */

/*! \addtogroup dyn_buffer
 *  @{
 */

/*! \brief Dynamic buffer object. Don't modify its internal members: use
 *         the related functions instead */
typedef struct DynBuffer DynBuffer;
struct DynBuffer
{
  size_t size;                    //! Current buffer size
  size_t pos_offset;              //! Current position indicator
  size_t max;                     //! Max buffer size
  unsigned char *data;         //! buffer data
};

/*! \brief Initialize a dynamic buffer
 * 
 *  \param d_buf Pointer to a DynBuffer object to be initialized
 */
void dynBufferInit( DynBuffer *d_buf );

/*! \brief Release a dynamic buffer
 * 
 *  \param d_buf Pointer to a DynBuffer object to be released
 */
void dynBufferRelease( DynBuffer *d_buf );

/*! \brief Append a copy of the n bytes pointed by new_buf to the end of the dynamic buffer pointed by d_buf
 * 
 *  \param d_buf Pointer to a DynBuffer object
 *  \param new_str Pointer to the buffer to be appended
 *  \param n Number of the bytes to be copied
 * 
 *  \return The new dynamic bufer size, or -1 on failure
 */
int dynBufferPushBackBuf( DynBuffer *d_buf, const unsigned char *new_buf, size_t n );

/*! \brief Append a 8 bit signed integer
 *         to the end of the dynamic buffer pointed by d_buf
 * 
 *  \param d_buf Pointer to a DynBuffer object
 *  \param val The value to be appended
 * 
 *  \return The new dynamic bufer size, or -1 on failure
 */
int dynBufferPushBackInt8( DynBuffer *d_buf, int8_t val );

/*! \brief Append a 16 bit signed integer (in binary format)
 *         to the end of the dynamic buffer pointed by d_buf
 *         WARNING:  The byte order depends on the endianess of your platform
 *
 *  \param d_buf Pointer to a DynBuffer object
 *  \param val The value to be appended
 *
 *  \return The new dynamic bufer size, or -1 on failure
 */
int dynBufferPushBackInt16( DynBuffer *d_buf, int16_t val );

/*! \brief Append a 32 bit signed integer (in binary format)
 *         to the end of the dynamic buffer pointed by d_buf
 *         WARNING:  The byte order depends on the endianess of your platform
 *
 *  \param d_buf Pointer to a DynBuffer object
 *  \param val The value to be appended
 *
 *  \return The new dynamic bufer size, or -1 on failure
 */
int dynBufferPushBackInt32( DynBuffer *d_buf, int32_t val );

/*! \brief Append a 64 bit signed integer (in binary format)
 *         to the end of the dynamic buffer pointed by d_buf
 *         WARNING:  The byte order depends on the endianess of your platform
 *
 *  \param d_buf Pointer to a DynBuffer object
 *  \param val The value to be appended
 *
 *  \return The new dynamic bufer size, or -1 on failure
 */
int dynBufferPushBackInt64( DynBuffer *d_buf, int64_t val );

/*! \brief Append a 8 bit signed integer
 *         to the end of the dynamic buffer pointed by d_buf
 *
 *  \param d_buf Pointer to a DynBuffer object
 *  \param val The value to be appended
 *
 *  \return The new dynamic bufer size, or -1 on failure
 */
int dynBufferPushBackUint8( DynBuffer *d_buf, uint8_t val );

/*! \brief Append a 16 bit unsigned integer (in binary format)
 *         to the end of the dynamic buffer pointed by d_buf
 *         WARNING:  The byte order depends on the endianess of your platform
 *
 *  \param d_buf Pointer to a DynBuffer object
 *  \param val The value to be appended
 *
 *  \return The new dynamic bufer size, or -1 on failure
 */
int dynBufferPushBackUint16( DynBuffer *d_buf, uint16_t val );

/*! \brief Append a 32 bit unsigned integer (in binary format)
 *         to the end of the dynamic buffer pointed by d_buf
 *         WARNING:  The byte order depends on the endianess of your platform
 *
 *  \param d_buf Pointer to a DynBuffer object
 *  \param val The value to be appended
 *
 *  \return The new dynamic bufer size, or -1 on failure
 */
int dynBufferPushBackUint32( DynBuffer *d_buf, uint32_t val );

/*! \brief Append a 64 bit unsigned integer (in binary format)
 *         to the end of the dynamic buffer pointed by d_buf
 *         WARNING:  The byte order depends on the endianess of your platform
 *
 *  \param d_buf Pointer to a DynBuffer object
 *  \param val The value to be appended
 *
 *  \return The new dynamic bufer size, or -1 on failure
 */
int dynBufferPushBackUint64( DynBuffer *d_buf, uint64_t val );

/*! \brief Append a single precision floating point value (in binary format)
 *         to the end of the dynamic buffer pointed by d_buf
 *
 *  \param d_buf Pointer to a DynBuffer object
 *  \param val The value to be appended
 *
 *  \return The new dynamic bufer size, or -1 on failure
 */
int dynBufferPushBackSingle( DynBuffer *d_buf, float val );

/*! \brief Append a double precision floating point value (in binary format)
 *         to the end of the dynamic buffer pointed by d_buf
 *
 *  \param d_buf Pointer to a DynBuffer object
 *  \param val The value to be appended
 *
 *  \return The new dynamic bufer size, or -1 on failure
 */
int dynBufferPushBackDouble( DynBuffer *d_buf, double val );

/*! \brief Clear a dynamic buffer (the internal memory IS NOT released)
 * 
 *  \param d_buf Pointer to a DynBuffer object
 */
void dynBufferClear( DynBuffer *d_buf );

/*! \brief Get the current dynamic buffer size
 * 
 *  \param d_buf Pointer to a DynBuffer object
 * 
 *  \return The current dynamic buffer size
 */
size_t dynBufferGetSize( DynBuffer *d_buf );

/*! \brief Get a const pointer to the internal buffer data
 * 
 *  \param d_buf Pointer to a DynBuffer object
 * 
 *  \return The const pointer to the internal buffer data
 */
const unsigned char *dynBufferGetData( DynBuffer *d_buf );

/*! \brief Move the position indicator, adding offset to the current position
 *         The position indicator may be exploited to mark alreadey "used" bytes
 * 
 *  \param d_buf Pointer to a DynBuffer object
 *  \param offset Offset in bytes added to the position indicator
 */
void dynBufferMovePoseIndicator( DynBuffer *d_buf, int offset );

/*! \brief Move the position indicator, setting the current position
 *         The position indicator may be exploited to mark alreadey "used" bytes
 *
 *  \param d_str Pointer to a DynBuffer object 
 *  \param pos The new the position indicator (in bytes)
 */
void dynBufferSetPoseIndicator( DynBuffer *d_buf, size_t pos );

/*! \brief Reset the position indicator
 *         The position indicator may be exploited to mark alreadey "used" bytes
 *
 *  \param d_buf Pointer to a DynBuffer object 
 */
void dynBufferRewindPoseIndicator( DynBuffer *d_buf );

/*! \brief Get the current offset of the position indicator
 * 
 *  \param d_buf Pointer to a DynBuffer object 
 
 *  \return Offset in bytes of the position indicator
 */
int dynBufferGetPoseIndicatorOffset( DynBuffer *d_buf );

/*! \brief Get a const pointer to the internal buffer data, starting from the current position indicator
 * 
 *  \param d_buf Pointer to a DynBuffer object
 * 
 *  \return The const pointer to the internal buffer data
 */
const unsigned char *dynBufferGetCurrentData( DynBuffer *d_buf );

/*! \brief Get the remaining dynamic buffer size, starting from the current position indicator
 * 
 *  \param d_buf Pointer to a DynBuffer object
 * 
 *  \return The remaining dynamic buffer size
 */
int dynBufferGetRemainingDataSize( DynBuffer *d_buf );

/*! @}*/

#endif