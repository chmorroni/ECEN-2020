#include "error.h"

char * errorToStr(error err) {
	switch (err) {
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
	}
}
