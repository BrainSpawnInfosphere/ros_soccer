
/************************************************* 
cTone.h

  A cTone Generator Library

Version Modified By Date     Comments
------- ----------- -------- --------
0001    B Hagman    09/08/02 Initial coding
0002    K Walchko   6/Oct/09 Removed usage of timers, now SW 

*************************************************/


#ifndef __TONE_H__
#define __TONE_H__

//#include "WProgram.h"
#include <inttypes.h>
#include <Arduino.h> 


/*************************************************
* Definitions
*************************************************/

//typedef unsigned char uint8_t;
//typedef bool boolean;

class cTone{
  public:
    void init(uint8_t tonePin);
    void playTone(int tone, int duration);
    void playNote(char note, unsigned char octave, int duration, boolean sharp);
    void play_rtttl(const char *song);
    void beep(int duration=500);
    void enable(boolean b=true);

  private:
    uint8_t _pin; // could replace with a define, doubt you would have more than one speaker???
    boolean enabled;
};

#endif
