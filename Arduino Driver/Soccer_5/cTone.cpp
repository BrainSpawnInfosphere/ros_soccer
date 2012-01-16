

#include "cTone.h"


#define OCTAVE_OFFSET 0

void cTone::init(uint8_t tonePin)
{
  _pin = tonePin;
  enabled = true;
  
  pinMode(_pin,OUTPUT);
}


void cTone::beep(int duration) {
  if(!enabled) return;
  playTone(1014,duration);
}


void cTone::enable(boolean b){
  enabled = b;
}

// frequency (in hertz) and duration (in milliseconds).
void cTone::playTone(int tone, int duration) {
  if(!enabled) return;
  
  for (long i = 0; i < duration * 1000L; i += tone * 2) {
    digitalWrite(_pin, HIGH);
    delayMicroseconds(tone);
    digitalWrite(_pin, LOW);
    delayMicroseconds(tone);
  }
}

/*
 * The calculation of the tones is made following the mathematical
 * operation:
 *
 *       timeHigh = 1/(2 * toneFrequency) = period / 2
 *
 * where the different tones are described as in the table:
 *
 * note 	frequency 	period 	PW (timeHigh)	
 * c 	        261 Hz 	        3830 	1915 	
 * d 	        294 Hz 	        3400 	1700 	
 * e 	        329 Hz 	        3038 	1519 	
 * f 	        349 Hz 	        2864 	1432 	
 * g 	        392 Hz 	        2550 	1275 	
 * a 	        440 Hz 	        2272 	1136 	
 * b 	        493 Hz 	        2028	1014	
 * C	        523 Hz	        1912 	956
 *
 */
void cTone::playNote(char note, unsigned char octave, int duration, boolean sharp) {
#if 1
	if(!enabled) return;
  
  char names[] = { 'c', 'd', 'e', 'f', 'g', 'a', 'b', 'C'}; // removed 'p', should be handled before here
  int tones[] = { 1915, 1700, 1519, 1432, 1275, 1136, 1014, 956 };
  int sharpTones[] = { 1807, 1605, 0, 1352, 1203, 1072, 0, 0 };
  
  // play the tone corresponding to the note name
  for (int i = 0; i < 8; i++) {
    if (names[i] == note) {
      if(sharp) playTone(sharpTones[i] >> (octave-4), duration);
      else playTone(tones[i] >> (octave-4), duration);
      break;
    }
  }
#endif
}


#define isdigit(n) (n >= '0' && n <= '9')

void cTone::play_rtttl(const char *p){
#if 1
  if(!enabled) return;
  
  // Absolutely no error checking in here
  byte default_dur = 4;
  byte default_oct = 6;
  int bpm = 63;
  int num;
  long wholenote;
  long duration;
  //byte note;
  byte scale;

  // format: d=N,o=N,b=NNN:
  // find the start (skip name, etc)

  while(*p != ':') p++;    // ignore name
  p++;                     // skip ':'

  // get default duration
  if(*p == 'd')
  {
    p++; p++;              // skip "d="
    num = 0;
    while(isdigit(*p))
    {
      num = (num * 10) + (*p++ - '0');
    }
    if(num > 0) default_dur = num;
    p++;                   // skip comma
  }

  //Serial.print("ddur: "); Serial.println(default_dur, 10);

  // get default octave
  if(*p == 'o')
  {
    p++; p++;              // skip "o="
    num = *p++ - '0';
    if(num >= 3 && num <=7) default_oct = num;
    p++;                   // skip comma
  }

  //Serial.print("doct: "); Serial.println(default_oct, 10);

  // get BPM
  if(*p == 'b')
  {
    p++; p++;              // skip "b="
    num = 0;
    while(isdigit(*p))
    {
      num = (num * 10) + (*p++ - '0');
    }
    bpm = num;
    p++;                   // skip colon
  }

  //Serial.print("bpm: "); Serial.println(bpm, 10);

  // BPM usually expresses the number of quarter notes per minute
  wholenote = (60 * 1000L / bpm) * 4;  // this is the time for whole note (in milliseconds)

  //Serial.print("wn: "); Serial.println(wholenote, 10);


  // now begin note loop
  while(*p)
  {
    // first, get note duration, if available
    num = 0;
    while(isdigit(*p))
    {
      num = (num * 10) + (*p++ - '0');
    }
    
    if(num) duration = wholenote / num;
    else duration = wholenote / default_dur;  // we will need to check if we are a dotted note after

    // now get the note
    //note = 0;
    
    char knote = *p;

    p++;
    
    boolean ksharp = false;

    // now, get optional '#' sharp
    if(*p == '#')
    {
      p++;
      ksharp = true;
      
    }

    // now, get optional '.' dotted note
    if(*p == '.')
    {
      //duration += duration<<1;
      duration <<= 1;
      p++;
    }
  
    // now, get scale
    if(isdigit(*p))
    {
      scale = *p - '0';
      p++;
    }
    else
    {
      scale = default_oct;
    }

    scale += OCTAVE_OFFSET;

    if(*p == ',') p++;       // skip comma for next note (or we may be at the end)

    // now play the note

    if(knote != 'p') playNote(knote,scale,duration,ksharp);
    else delay(duration);
  }
#endif
}
