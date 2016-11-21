#include "error.h"

/**
 * @desc  Returns a pointer to a static string that provides a wordier
 *        description of the error, useful for logging errors.
 * @param err - The error to describe.
 */
char * errorToStr(error err) {
	switch (err) {
	case ERR_RACE_COND:
		return "race condition";
	case ERR_FULL:
		return "full";
	case ERR_EMPTY:
		return "empty";
	case ERR_OVERFLOW:
		return "overflow";
	case ERR_NAN:
		return "not a number";
	case ERR_OUT_OF_MEM:
		return "out of heap memory";
	case ERR_UNINITIALIZED:
		return "parameter is uninitialized";
	case ERR_PARAM_OUT_OF_BOUNDS:
		return "parameter is out of bounds";
	case ERR_NULL_PTR:
		return "passed a null pointer";
	case ERR_NO:
		return "no error";
	default:
		return "Error in error.c! No description available.";
	}
}
