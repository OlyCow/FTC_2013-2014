#ifndef SOUND_C
#define SOUND_C
#pragma systemFile
#include "..\Headers\Sound.h"
// For default values, see above header file.



void Sound_PlayFile(string fileName) {
	PlaySoundFile(fileName);
}
void Sound_PlaySound(TSounds sound) {
	PlaySound(sound);
}
void Sound_PlayTone(NoteFrequency frequency, int duration, bool ignoreQueue) {
	switch (ignoreQueue) {
		case true:
			PlayImmediateTone(frequency, duration/10); //intentional int division
			break;
		case false:
			PlayTone(frequency, duration/10); //intentional int division
			break;
	}
}
void Sound_SetVolume(int volume) {
	nVolume = volume;
}
int  Sound_GetVolume() {
	return nVolume;
}
void Sound_Mute() {
	MuteSound();
}
void Sound_Unmute(int volume) {
	UnmuteSound();
	nVolume = volume;
}
void Sound_ClearQueue() {
	ClearSounds();
}
bool Sound_IsPlaying() {
	return bSoundActive;
}
bool Sound_IsQueueEmpty() {
	return bSoundQueueAvailable;
}
void Sound_SetQueueStatus(bool isOpen) {
	bPlaySounds = isOpen;
}
bool Sound_GetQueueStatus() {
	return bPlaySounds;
}
void Sound_Moo() {
	PlaySoundFile("..\Resources\moo.rso");
}



//---PlayImmediateTone(const int frequency, const int durationIn10MsecTicks)
//---PlayTone(const int frequency, const int durationIn10MsecTicks)
//---PlaySoundFile(const string &sFileName)
//---PlaySound(TSounds sound)
//---bPlaySounds
//---bSoundActive
//---bSoundQueueAvailable
//---nVolume
//---MuteSound()
//---UnmuteSound()
//---ClearSounds()
#endif // SOUND_C
