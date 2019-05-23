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
  int len;            //! Current string length not including the \0 character at the end of the string
  int pos_offset;     //! Movable current string position indicator (cursor) that indicates the start of a last string chunk
  int max;            //! Max string size not including the space for \0 character at the end of the string
  char *data;         //! Pointer to the \0-terminated string data
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
 *  \return The current dynamic string length, not including the terminating null byte, or -1 on failure
 */
int dynStringPushBackStr( DynString *d_str, const char *new_str );

/*! \brief Append a copy of the first n characters of the
 *         string pointed by new_str to the end of the dynamic string pointed by d_str
 *
 *  \param d_str Pointer to a DynString object
 *  \param new_str Pointer to the new string to be appended
 *  \param n Number of characters to be copied
 *
 *  \return The current dynamic string length, not including the terminating null byte, or -1 on failure
 */
int dynStringPushBackStrN( DynString *d_str, const char *new_str, int n );

/*! \brief Replace the content of the dynamic string pointed by d_str by the first n characters of the
 *         character array pointed by new_str
 *
 *  \param d_str Pointer to a DynString object
 *  \param new_str Pointer to the string that will be the new d_str content
 *  \param n Number of characters of the inserted in d_str
 *
 *  \return The current dynamic string length, not including the terminating null byte, or -1 on failure
 */
int dynStringReplaceWithStrN ( DynString *d_str, const char *new_str, int n );

/*! \brief Append a character to the end of the dynamic string pointed by d_str
 *
 *  \param d_str Pointer to a DynString object
 *  \param c The character to be appended
 *
 *  \return The current dynamic string length, not including the terminating null byte, or -1 on failure
 */
int dynStringPushBackChar( DynString *d_str, const char c );

/*! \brief Copy the string pointed by new_str (not including the terminating null byte)
 *         inside the dynamic string pointed by d_str, starting from the position pos
 *
 *  \param d_str Pointer to a DynString object
 *  \param new_str Pointer to the string to be copied
 *  \param pos Starting position for the copy, must be smaller than the current dynamic string length
 *
 *  \return The new dynamic string length, not including the terminating null byte, or -1 on failure
 */
int dynStringPatch( DynString *d_str, const char *new_str, int pos );

/*! \brief Clear a dynamic string (the internal memory IS NOT released)
 *
 *  \param d_str Pointer to a DynString object
 */
void dynStringClear( DynString *d_str );

/*! \brief Truncates a DynString object. If more characters than available are tried to be removed, the string just becomes empty.
 *         The offset index of the DynString is moved left the number of positions removed from left up to pointing to 0. If
 *         characters are removed from the right, the offset is adjusted to limit its right position to the \0 character at most.
 *
 *  \param d_str Pointer to a DynString object
 *  \param rem_left Number of character to remove from the beggning of the string
 *  \param rem_right Number of character to remove from the end of the string
 */
void dynStringReduce ( DynString *d_str, int rem_left, int rem_right);

/*! \brief Get the current dynamic string length, not including the terminating null byte
 *
 *  \param d_str Pointer to a DynString object
 *
 *  \return The new dynamic string length, not including the terminating null byte
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
 *         The position indicator may be exploited to mark already "used" characters
 *
 *  \param d_str Pointer to a DynString object
 *  \param offset Offset in bytes added to the position indicator
 */
void dynStringMovePoseIndicator( DynString *d_str, int offset );

/*! \brief Move the position indicator, setting the current position
 *         The position indicator may be exploited to mark already "used" characters
 *
 *  \param d_str Pointer to a DynString object
 *  \param pos The new the position indicator (in bytes)
 */
void dynStringSetPoseIndicator( DynString *d_str, int pos );

/*! \brief Reset the position indicator
 *         The position indicator may be exploited to mark already "used" characters
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

/*! \brief Get the remaining dynamic string length starting from the current position indicator,
 *         not including the terminating null byte
 *
 *  \param d_str Pointer to a DynString object
 *
 *  \return The remaining dynamic string length
 */
int dynStringGetRemainingDataSize( DynString *d_str );

/*! @}*/

#endif
