/**
 * @brief Limited "standard" API for OCR
 **/

/*
 * This file is subject to the license agreement located in the file LICENSE
 * and cannot be distributed without it. This notice cannot be
 * removed or modified.
 */


#ifndef __OCR_STD_H__
#define __OCR_STD_H__
#ifdef __cplusplus
extern "C" {
#endif

#include "ocr-types.h"


/**
 * @defgroup OCRStandrdAPI Limited "standard" API for OCR
 * @brief Describes the limited "standard" API for OCR
 *
 * This file provides APIs for very limited standard functions
 * that replicate certain libc functions.
 *
 * @note This may be extended in the future.
 *
 * @{
 **/

/**
 * @brief Build string from format args
 *
 * This maps to the typical C-like snprintf() functionality.
 *
 * @param[in] buf       Output buffer for formatted string.
 * @param[in] size      Maximum number of characters to write to
 *                      buf (including the terminating '\0').
 * @param[in] fmt       Format specifier, as per printf(). Not
 *                      all modifiers are supported. Standard ones
 *                      like %d, %f, %s, %x, %p work properly
 * @param[in] ...       Arguments, as per printf().
 *
 * @return number of characters in output, as per snprintf().
 *
 **/
u32 SNPRINTF(char * buf, u32 size, const char * fmt, ...) /*__attribute__((__format__ (__printf__, 3, 4))) */;

#ifdef NOPRINTS
u32 PRINTF(const char *fmt, ...) {}
u32 ocrPrintf(const char * fmt, ...) {} /* __attribute__((__format__ (__printf__, 1, 2))) */;
#else

/**
 * @brief Console output
 *
 * This maps to the typical C-like printf() functionality.
 *
 * @param[in] fmt       Format specifier, as per printf(). Not
 *                      all modifiers are supported. Standard ones
 *                      like %d, %f, %s, %x, %p work properly
 * @param[in] ...       Arguments, as per printf().
 *
 * @return number of characters printed, as per printf().
 *
 **/
extern u32 ocrPrintf(const char * fmt, ...) /* __attribute__((__format__ (__printf__, 1, 2))) */;
extern u32 PRINTF(const char * fmt, ...) /* __attribute__((__format__ (__printf__, 1, 2))) */;
#endif

/**
 * @brief Platform independent 'assert' functionality
 *
 * This will cause the program to abort and return an
 * assertion failure.
 * This function should be called using the #ocrAssert macro
 *
 * @param[in] val       If non-zero, will cause the assertion failure
 * @param[in] str       Stringified condition so we can output it again
 * @param[in] file      File in which the assertion failure occured
 * @param[in] line      Line at which the assertion failure occured
 */
extern void _ocrAssert(bool val, const char* str, const char* file, u32 line);

#ifdef OCR_ASSERT
/**
 * @brief ASSERT macro to replace the assert functionality
 *
 * @param[in] a  Condition for the assert
 */
#define ASSERT(a) do {                                                              \
    ocrPrintf("ASSERT is deprecated as of OCR v1.2.0... use ocrAssert\n");    \
    _ocrAssert((bool)((a) != 0), #a, __FILE__, __LINE__); } while(0);

#define ocrAssert(a) do { _ocrAssert((bool)((a) != 0), #a, __FILE__, __LINE__); } while(0);

#else
/**
 * @brief ASSERT macro to replace the assert functionality
 *
 * @param[in] a  Condition for the assert
 */
/* The deprecated fallback printed on every call without evaluating the
 * condition; on hot paths every worker serialized on the stdout mutex.
 * Drop the print, keep the no-op. */
#define ASSERT(a) do { } while(0);

#define ocrAssert(a)

#endif

/**
 * @brief Primitive method to print FAILURE or PASSED
 * messages
 *
 * This macro is used to simply verify if the execution
 * of the program is correct and prints out either a
 * message starting with PASSED if successful or FAILURE
 * if not
 *
 * @param[in] cond    If true, will print a PASSED message.
 * @param[in] format  Message to print (see #PRINTF)
 * @param[in] ...     Arguments for message
 */
#define VERIFY(cond, format, ...)                                       \
    do {                                                                \
        if(!(cond)) {                                                   \
            ocrPrintf("FAILURE @ '%s:%" PRId32 "' " format, __FILE__, __LINE__, ## __VA_ARGS__); \
        } else {                                                        \
            ocrPrintf("PASSED @ '%s:%" PRId32 "' " format, __FILE__, __LINE__, ## __VA_ARGS__); \
        }                                                               \
    } while(0);

/**
 * @}
 */
#ifdef __cplusplus
}
#endif

#endif /* __OCR_STD_H__ */


