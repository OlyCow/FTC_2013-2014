// I really don't know what this is for.

#include "\Headers\includes.h"

task main() {
	Sound_Unmute();
	Sound_SetVolume(4);

	while (true) {
		Sound_Moo();
		while (Sound_IsPlaying()==true) {
			Time_Wait(1000);
		}
	}
}



//................................................................//
//................................................................//
//................................MM+.............................//
//................................MM...............M .............//
//............................... MMMM,$MMMM......MM .............//
//.................................MMMMMMMMMMMMMMMMM..............//
//................................ .MMMMMMMMMMMMMM ...............//
//...........................MMMMMMMMMMMMMMMMMMM .................//
//.........................MMMMMMMMMMMMMMMMMMMMMMMMM?.............//
//........................MMMMMMMMMMMMMMMMMMMMMMMMMMMMM...........//
//......................... MMMMM  MMMMMMMMMMMMMMMMMM.............//
//................................MMMMMMMMMMMMM..N ...............//
//..............................8MMMMMMMMMMMMMMMMM ...............//
//.............................MMMM..MMMMMMMMMMMMMMMMMMM .........//
//............................MMD.....MMMMMMMMMMMMMMMMMMM.........//
//....................................MMMMMMMMMMMMMMMMMMMM........//
//.....................................MMMMMMMMMMMMMMMMMMMM.......//
//..........................DMMMM.........O=.MMMMMMMMMMMMMMM......//
//.. ......................MMMMMMMMM?........,MMMMMMMMMMMMMM .....//
//.. .................... MMMMMMMMMMMM .......MMMMMMMMMMMMMMM.....//
//.......................MMMMMMMMMMMMMM .....MMMMMMMMMMMMMMMMD....//
//......................,MMMMMMMMMMMMMMM....OMMMMMMMMMMMMMMMMM ...//
//......................MMMMMMMMMMMMMMMM.....MMMMMMMMMMMMMMMMM ...//
//......................MMMMMMMMMMMMMMMM.....DMMMMMMMMMMMMMMMMO...//
//.....................=MMMMMMMMMMMMMMMM.......MMMMMMMMMMMMMMM,...//
//......................MMMMMMMMMMMMMMMM.......MMMMMMMMMMMMMMM....//
//......................MMMMMMMMMMMMMMM .......MMMMMMMMMMMMMMN....//
//......................MMMMMMMMMMMMMM ....+MMMMMMMMMMMMMMMMM.....//
//....................... MMMMMMMMMM .....IMMMMMMMMMMMMMMMMM......//
//........................................MMMMMMMMMMMMMMMMM ......//
//........................................MMMMMMMMMMMMMMM ........//
//...................................... $MMMMMMMMMMMMM ..........//
//................................MMMMMMMMMMMMMMMMMMMM ...........//
//...............................MMMMMMMMMMMMMMMMMMMMM ...........//
//...............................MMMMM 7MMMMMMMMMMMMMM............//
//................................MMMM?.MMMMMMMMMMMMM.............//
//................................MMMMM .MMMMMMMMMMMM.............//
//.................................MMMM ..MMMMM.MMMMM.............//
//.................................MMMM ..MMMMM MMMM..............//
//.................................NMMM...MMMMM MMMM..............//
//.................................OMMM...MMMMM MMMM..............//
//.................................MMMM....MMMM.MMMM..............//
//.................................MMMM....MMMM.MMMM..............//
//...............NMMMMMMMMMMMMMMMMMMMMMMMMMMMMN...................//
//...MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMN..............................//
//........NNNNNN..................................................//
//................................................................//
//................................................................//
