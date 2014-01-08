#ifndef _DYN_STRING_H_
#define _DYN_STRING_H_

/*! \defgroup dyn_string Dynamic string */

/*! \addtogroup dyn_string
 *  @{
 */

/*! \brief Dynamic string object. Don't modify its internal members: use
 *         the related functions instead */
typedef struct DynString DynString;
struct DynString
{
  int len;            //! Current string lenght
  int pos_offset;     //! Current position indicator
  int max;            //! Max string size
  char *data;         //! String data
};

/*! \brief Initialize a dynamic string
 * 
 *  \param d_str Pointer to a DynString object to be initialized
 */
void dynStringInit( DynString *d_str );

/*! \brief Release a dynamic string
 *  \param d_str Pointer to a DynString object to be released
 */
void dynStringRelease( DynString *d_str );

/*! \brief Append a copy of the string pointed by new_str to the end of the dynamic string pointed by d_str
 * 
 *  \param d_str Pointer to a DynString object
 *  \param new_str Pointer to the new string to be appended
 * 
 *  \return The current dynamic string lenght, not including the terminating null byte, or -1 on failure
 */
int dynStringPushBackStr( DynString *d_str, const char *new_str );

/*! \brief Append a copy of the first n characters of the 
 *         string pointed by new_str to the end of the dynamic string pointed by d_str
 * 
 *  \param d_str Pointer to a DynString object
 *  \param new_str Pointer to the new string to be appended
 *  \param n Number of characters to be copied
 * 
 *  \return The current dynamic string lenght, not including the terminating null byte, or -1 on failure
 */
int dynStringPushBackStrN( DynString *d_str, const char *new_str, int n );

/*! \brief Append a character to the end of the dynamic string pointed by d_str
 * 
 *  \param d_str Pointer to a DynString object
 *  \param c The character to be appended
 * 
 *  \return The current dynamic string lenght, not including the terminating null byte, or -1 on failure
 */
int dynStringPushBackChar( DynString *d_str, const char c );

/*! \brief Copy the string pointed by new_str (not including the terminating null byte) 
 *         inside the dynamic string pointed by d_str, starting from the position pos
 * 
 *  \param d_str Pointer to a DynString object
 *  \param new_str Pointer to the string to be copied
 *  \param pos Starting position for the copy, must be smaller than the current dynamic string lenght
 * 
 *  \return The new dynamic string lenght, not including the terminating null byte, or -1 on failure
 */
int dynStringPatch( DynString *d_str, const char *new_str, int pos );

/*! \brief Clear a dynamic string (the internal memory IS NOT released)
 * 
 *  \param d_str Pointer to a DynString object
 */
void dynStringClear( DynString *d_str );

/*! \brief Get the current dynamic string lenght, not including the terminating null byte
 * 
 *  \param d_str Pointer to a DynString object
 * 
 *  \return The new dynamic string lenght, not including the terminating null byte
 */
int dynStringGetLen( DynString *d_str );

/*! \brief Get a const pointer to the internal string data
 * 
 *  \param d_str Pointer to a DynString object
 * 
 *  \return The const pointer to the internal string data
 */
const char *dynStringGetData( DynString *d_str );

/*! \brief Move the position indicator, adding offset to the current position
 *         The position indicator may be exploited to mark alreadey "used" characters
 *
 *  \param d_str Pointer to a DynString object 
 *  \param offset Offset in bytes added to the position indicator
 */
void dynStringMovePoseIndicator( DynString *d_str, int offset );

/*! \brief Move the position indicator, setting the current position
 *         The position indicator may be exploited to mark alreadey "used" characters
 *
 *  \param d_str Pointer to a DynString object 
 *  \param pos The new the position indicator (in bytes)
 */
void dynStringSetPoseIndicator( DynString *d_str, int pos );

/*! \brief Reset the position indicator
 *         The position indicator may be exploited to mark alreadey "used" characters
 *
 *  \param d_str Pointer to a DynString object 
 */
void dynStringRewindPoseIndicator( DynString *d_str );

/*! \brief Get the current offset of the position indicator
 * 
 *  \param d_str Pointer to a DynString object 
 
 *  \return Offset in bytes of the position indicator
 */
int dynStringGetPoseIndicatorOffset( DynString *d_str );

/*! \brief Get a const pointer to the internal string data, starting from the current position indicator
 * 
 *  \param d_buf Pointer to a DynString object
 * 
 *  \return The const pointer to the internal buffer data
 */
const char *dynStringGetCurrentData( DynString *d_str );

/*! \brief Get the remaining dynamic string lenght starting from the current position indicator,
 *         not including the terminating null byte
 * 
 *  \param d_str Pointer to a DynString object
 * 
 *  \return The remaining dynamic string lenght
 */
int dynStringGetRemainingDataSize( DynString *d_str );

/*! @}*/

#endif