#ifndef SOUND_H
#define SOUND_H
#pragma systemFile



void Sound_PlayFile(string fileName);
void Sound_PlaySound(TSounds sound);
void Sound_PlayTone(NoteFrequency frequency, int duration, bool ignoreQueue=false);
void Sound_SetVolume(int volume);
int  Sound_GetVolume();
void Sound_Mute();
void Sound_Unmute(int volume=nVolume);
void Sound_ClearQueue();
bool Sound_IsPlaying();
bool Sound_IsQueueEmpty();
void Sound_SetQueueStatus(bool isOpen);
bool Sound_GetQueueStatus();



#include "..\Libraries\Sound.c"
#endif // SOUND_H
