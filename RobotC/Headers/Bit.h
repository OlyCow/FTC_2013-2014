#ifndef BIT_H
#define BIT_H
#pragma systemFile



#define HIGH (1)
#define LOW (0)

#define Bit_Set(byte, pos, val)		((byte) |= ((val)<<(pos)))
#define Bit_Clear(byte, pos)		((byte) &= ~((1)<<(pos)))



#include "..\Libraries\Bit.c"
#endif // BIT_H
