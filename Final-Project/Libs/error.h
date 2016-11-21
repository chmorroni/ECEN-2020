#ifndef ERROR
#define ERROR

typedef enum {
	ERR_BUSY = -10,
	ERR_RACE_COND = -9,
	ERR_FULL = -8,
	ERR_EMPTY = -7,
	ERR_OVERFLOW = -6,
	ERR_NAN = -5,
	ERR_OUT_OF_MEM = -4 ,
	ERR_UNINITIALIZED = -3,
	ERR_PARAM_OUT_OF_BOUNDS = -2,
	ERR_NULL_PTR = -1,
	ERR_NO = 0
} error;

/**
 * @desc  Returns a pointer to a static string that provides a wordier
 *        description of the error, useful for logging errors.
 * @param err - The error to describe.
 */
char * errorToStr(error err);

#endif
