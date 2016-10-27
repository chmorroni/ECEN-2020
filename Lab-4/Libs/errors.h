#ifndef ERRORS
#define ERRORS

typedef enum {
	OUT_OF_MEM = -4 
	UNINITIALIZED = -3,
	PARAM_OUT_OF_BOUNDS = -2,
	NULL_PTR = -1,
	NO_ERR = 0
} error_T;

#endif