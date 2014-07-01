#ifndef _CROS_GENTOOLS_H_
#define _CROS_GENTOOLS_H_

/*! \defgroup cros_gentools cROS message generation tool
 * 
 * Implemenation of the  ROS message generation tool and
 * dependency resolutor (gentools and gendeps)
 */

/*! \addtogroup cros_gentool
 *  @{
 */

/*! \brief Generate md5 hash of files
 * 
 *  \param filename Full path of the message/service file
 */
char* cRosGentoolsMD5(char* filename);

/*! \brief Generate SHA1 hash of files
 * 
 *  \param filename Full path of the message/service file
 * 
 *  \return Returns 1 on success, 0 on failure
 */
int cRosGentoolsSHA1(char* filename);

/*! \brief Generate concatenated list of files
 * 
 *  \param filename Full path of the message/service file
 *
 */
int cRosGentoolsFulltext(char* filename);

/*! @}*/

#endif
