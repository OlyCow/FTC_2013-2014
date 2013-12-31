#ifndef BIT_C
#define BIT_C
#pragma systemFile
#include "..\Headers\Bit.h"
// For default values, see above header file.



bool Bit_FixBool(bool input)
{
	// Defaults to this because that's probably what was inputted.
	bool return_value = true;
	if (input==0) {
		return_value = false;
	} else {
		return_value = true;
	}
	return return_value;
}



#endif // BIT_C
