/*
The Rad-Fi Patchable Synthesizer RS01
http://bleeplabs.com/store/the-rad-fi-system/ 

This works well but needs some work by the time the Rad-Fis ship


Todo:
commenting and moding instructions
PWM rate
general loop speedup

*/

#include <avr/pgmspace.h>
//#define DIGITALIO_NO_MIX_ANALOGWRITE
#include "digitalIOPerformance.h"
#include <MIDI.h>

int drymix,wetmix,dely_in;
int32_t write_head,read_head;
int32_t lerp_dly,lerp_len, filter, feedback,fb_tmp_inv,fb_tmp,lerp_tick,dly_length;


PROGMEM prog_char  waveTable[830] =

{  
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
  255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
  255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
  255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
  255,255,255,255,255,255,

  1,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,
  61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,
  114,115,116,117,118,119,120,121,122,123,124,125,126,127,128,129,130,131,132,133,134,135,136,137,138,139,140,141,142,143,144,145,146,147,148,149,150,151,152,153,154,155,156,
  157,158,159,160,161,162,163,164,165,166,167,168,169,170,171,172,173,174,175,176,177,178,179,180,181,182,183,184,185,186,187,188,189,190,191,192,193,194,195,196,197,198,199,
  200,201,202,203,204,205,206,207,208,209,210,211,212,213,214,215,216,217,218,219,220,221,222,223,224,225,226,227,228,229,230,231,232,233,234,235,236,237,238,239,240,241,242,
  243,244,245,246,247,248,249,250,251,252,253,254,

  255,254,252,250,248,246,244,242,
  240,238,236,234,232,230,228,226,224,222,220,218,216,214,212,210,208,
  206,204,202,200,198,196,194,192,190,188,186,184,182,180,178,176,174,
  172,170,168,166,164,162,160,158,156,154,152,150,148,146,144,142,
  140,138,136,134,132,130,128,126,124,122,120,118,116,114,112,110,108,
  106,104,102,100,98,96,94,92,90,88,86,84,82,80,78,76,74,72,70,68,66,
  64,62,60,58,56,54,52,50,48,46,44,42,40,38,36,34,32,30,28,26,24,22,
  20,18,16,14,12,10,8,6,4,2,0,
  1,3,5,7,9,11,13,15,17,19,21,23,25,27,29,31,
  33,35,37,39,41,43,45,47,49,51,53,55,57,59,61,63,65,
  67,69,71,73,75,77,79,81,83,85,87,89,91,93,95,97,99,
  101,103,105,107,109,111,113,115,117,119,121,123,125,127,129,131,133,
  135,137,139,141,143,145,147,149,151,153,155,157,159,161,163,165,167,
  169,171,173,175,177,179,181,183,185,187,189,191,193,195,197,199,201,
  203,205,207,209,211,213,215,217,219,221,223,225,227,229,231,233,235,
  237,239,241,243,245,247,249,251,253,

};

PROGMEM prog_char  waveTable2[830] =

{  
254,253,252,251,250,249,248,247,246,245,244,243,242,241,240,239,238,237,236,235,234,233,232,231,230,229,228,227,226,225,224,223,222,221,220,219,218,217,216,215,214,213,212,211,210,209,208,207,206,205,204,203,202,201,200,199,198,197,196,195,194,193,192,191,190,189,188,187,186,185,184,183,182,181,180,179,178,177,176,175,174,173,172,171,170,169,168,167,166,165,164,163,162,161,160,159,158,157,156,155,154,153,152,151,150,149,148,147,146,145,144,143,142,141,140,139,138,137,136,135,134,133,132,131,130,129,128,127,126,125,124,123,122,121,120,119,118,117,116,115,114,113,112,111,110,109,108,107,106,105,104,103,102,101,100,99,98,97,96,95,94,93,92,91,90,89,88,87,86,85,84,83,82,81,80,79,78,77,76,75,74,73,72,71,70,69,68,67,66,65,64,63,62,61,60,59,58,57,56,55,54,53,52,51,50,49,48,47,46,45,44,43,42,41,40,39,38,37,36,35,34,33,32,31,30,29,28,27,26,25,24,23,22,21,20,19,18,17,16,15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0,


255,254,252,250,248,246,244,242,
  240,238,236,234,232,230,228,226,224,222,220,218,216,214,212,210,208,
  206,204,202,200,198,196,194,192,190,188,186,184,182,180,178,176,174,
  172,170,168,166,164,162,160,158,156,154,152,150,148,146,144,142,
  140,138,136,134,132,130,128,126,124,122,120,118,116,114,112,110,108,
  106,104,102,100,98,96,94,92,90,88,86,84,82,80,78,76,74,72,70,68,66,
  64,62,60,58,56,54,52,50,48,46,44,42,40,38,36,34,32,30,28,26,24,22,
  20,18,16,14,12,10,8,6,4,2,0,  1,3,5,7,9,11,13,15,17,19,21,23,25,27,29,31,
  33,35,37,39,41,43,45,47,49,51,53,55,57,59,61,63,65,
  67,69,71,73,75,77,79,81,83,85,87,89,91,93,95,97,99,
  101,103,105,107,109,111,113,115,117,119,121,123,125,127,129,131,133,
  135,137,139,141,143,145,147,149,151,153,155,157,159,161,163,165,167,
  169,171,173,175,177,179,181,183,185,187,189,191,193,195,197,199,201,
  203,205,207,209,211,213,215,217,219,221,223,225,227,229,231,233,235,
  237,239,241,243,245,247,249,251,253,255,255,255,255,


1,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,
  61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,
  114,115,116,117,118,119,120,121,122,123,124,125,126,127,128,129,130,131,132,133,134,135,136,137,138,139,140,141,142,143,144,145,146,147,148,149,150,151,152,153,154,155,156,
  157,158,159,160,161,162,163,164,165,166,167,168,169,170,171,172,173,174,175,176,177,178,179,180,181,182,183,184,185,186,187,188,189,190,191,192,193,194,195,196,197,198,199,
  200,201,202,203,204,205,206,207,208,209,210,211,212,213,214,215,216,217,218,219,220,221,222,223,224,225,226,227,228,229,230,231,232,233,234,235,236,237,238,239,240,241,242,
  243,244,245,246,247,248,249,250,251,252,253,254,

};

PROGMEM prog_char  squareTable[500] ={

0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254
};
PROGMEM prog_char  sinTable[256] =
{  
0,0,0,0,1,1,1,2,2,3,4,5,5,6,7,9,10,11,12,14,15,17,18,20,21,23,25,27,29,31,33,35,37,40,42,44,47,49,52,54,57,59,62,65,67,70,73,76,79,82,85,88,90,93,97,100,103,106,109,112,115,118,121,124,128,
131,134,137,140,143,146,149,152,155,158,162,165,167,170,173,176,179,182,185,188,190,193,196,198,201,203,206,208,211,213,215,218,220,222,224,226,228,230,232,234,235,237,238,240,241,243,244,245,246,248,249,250,250,251,252,253,253,254,254,254,255,255,255,255,255,255,255,254,254,254,253,253,252,251,250,250,249,248,246,245,244,243,241,240,238,237,235,234,232,230,228,226,224,222,220,218,215,213,211,208,206,203,201,198,196,193,190,188,185,182,179,176,173,170,167,165,162,158,155,152,149,146,143,140,137,134,131,
128,124,121,118,115,112,109,106,103,100,97,93,90,88,85,82,79,76,73,70,67,65,62,59,57,54,52,49,47,44,42,40,37,35,33,31,29,27,25,23,21,20,18,17,15,14,12,11,10,9,7,6,5,5,4,3,2,2,1,1,1,0,0,0,
};

PROGMEM prog_char log_look[129]={  0,0,0,0,0,0,0,1,1,1,1,1,1,2,2,2,2,3,3,3,3,4,4,5,5,5,6,6,7,7,8,8,9,9,10,10,11,11,12,13,13,14,14,15,16,17,17,18,19,20,20,21,22,23,24,25,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,41,42,43,44,45,46,48,49,50,51,53,54,55,56,58,59,61,62,63,65,66,68,69,71,72,74,75,77,78,80,81,83,85,86,88,89,91,93,95,96,98,100,102,103,105,107,109,111,113,114,116,118,120,122,124,126,126,126
};

float midi_freq_table[128]={ 
0,8.1758,8.6620,9.1770,10.3009,10.3009,10.9134,11.5623,12.2499,12.9783,13.7500,14.5676,15.4339,16.3516,17.3239,18.3540,19.4454,20.6017,21.8268,23.1247,24.4997,25.9565,27.5000,29.1352,30.8677,32.7032,34.6478,36.7081,38.8909,41.2034,43.6535,46.2493,48.9994,51.9131,55.0000,58.2705,61.7354,65.4064,69.2957,73.4162,77.7817,82.4069,87.3071,92.4986,97.9989,103.8262,110.0000,116.5409,123.4708,130.8128,138.5913,146.8324,155.5635,164.8138,174.6141,184.9972,195.9977,207.6523,220.0000,233.0819,246.9417,261.6256,277.1826,293.6648,311.1270,329.6276,349.2282,369.9944,391.9954,415.3047,440.0000,466.1638,493.8833,523.2511,554.3653,587.3295,622.2540,659.2551,698.4565,739.9888,783.9909,830.6094,880.0000,932.3275,987.7666,1046.5023,1108.7305,1174.6591,1244.5079,1318.5102,1396.9129,1479.9777,1567.9817,1661.2188,1760.0000,1864.6550,1975.5332,2093.0045,2217.4610,2349.3181,2489.0159,2637.0205,2793.8259,2959.9554,3135.9635,3322.4376,3520.0000,3729.3101,3951.0664,4186.0090,4434.9221,4698.6363,4978.0317,5274.0409,5587.6517,5919.9108,5919.9108,6644.8752,7040.0000,7458.6202,7902.1328,8372.0181,8869.8442,9397.2726,9956.0635,10548.0818,11175.3034,11839.8215
};

float gen_scale[200]={ };

#define rec_max 5

uint16_t rec_bank[rec_max]={ };

byte natural_m[7]={  2,1,2,2,1,2,2};

uint16_t todac;
long prev2,d,t,prev;
static long LMSIZE =16;
float j;
uint32_t accumulator[4]={};
uint32_t increment[4]={};
int16_t out[4]={};
byte digout[4]={};

uint32_t freq[4]={};
int waveindex[4]={};
int16_t amp[4]={};

byte tick;
int wr,tap,tap2,echo2,echo1;

long dds_tune;
int raw;
int f;
float freq_out,freq_out2;
byte d4,pd4,chro_trig,d5,pd5,d6,pd6,d7,pd7;
int chro_cnt;
int step_rate;
int shape_pot,chro_step;
byte trig3;
int tirg3_cnt;
byte chro_tick;

int16_t pot2,pot3; 
int16_t to_dly_temp;
byte pot_tick;
int16_t ain,dly_out;
int16_t vco1,vco2,vca1,vca2,shape1,shape2;
int16_t vcaA,vcaB;

int dds_rate=10;
/*
const float chromatic[129]={
  0,16.35,17.32,18.35,19.45,20.6,21.83,23.12,24.5,25.96,27.5,29.14,30.87,32.7,34.65,36.71,38.89,41.2,43.65,46.25,49,51.91,55,58.27,61.74,65.41,69.3,73.42,77.78,82.41,87.31,92.5,98,103.83,110,116.54,123.47,130.81,138.59,146.83,155.56,164.81,174.61,185,196,207.65,220,233.08,246.94,261.63,277.18,293.66,311.13,329.63,349.23,369.99,392,415.3,440,466.16,493.88,523.25,554.37,587.33,622.25,659.25,698.46,739.99,783.99,830.61,880,932.33,987.77,1046.5,1108.73,1174.66,1244.51,1318.51,1396.91,1479.98,1567.98,1661.22,1760,1864.66,1975.53,2093,2217.46,2349.32,2489.02,2637.02,2793.83,2959.96,3135.96,3322.44,3520,3729.31,3951.07,4186.01,4434.92,4698.63,4978.03,5274.04,5587.65,5919.91,6271.93,6644.88,7040,7458.62,7902.13};
*/
byte note_step,trig_button,ptb;
long attackms,relms;
float vco2f=220;
float vco1f=220;
byte prb,rec_button,recording,ppb,play_button,playing,pltb,rec_led,env_fall,loop_fall,env_b,env_loop,status_led,shift_button,env_lock;
uint16_t rec_cnt,play_cnt,loop_trig_button,loop_vco1,env_f;
int16_t loop_len,shapeB,shapeA;
uint32_t playcms,recms,ledms;
byte direc=1;
byte noise_mode_pin,range_pin,dual_voice_pin,vco2m,dual_mode,suboct_pin,detune_pin,sh_pin,direct;
byte MIDI_ENABLE=1;

byte midi_note,channel,incoming_note;
byte midi_happend=0;
uint16_t tick_dly;
byte oct_up,oct_down;

byte env_out[4]={};
byte env_mode[4]={};
byte env_dly[4]={};
byte env_cnt[4]={};

void setup() {
  randomSeed(analogRead(5));
  
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

 
 sbi (TIMSK1,TOIE1);
 
  sbi (TCCR1B, CS20);
  cbi (TCCR1B, CS21);
  cbi (TCCR1B, CS22);
  
  cbi (TCCR1A, COM2A0);  // clear Compare Match
  sbi (TCCR1A, COM2A1);
  sbi (TCCR1A, COM2B1);
  
  sbi (TCCR1A, WGM20);  // Mode 1  / Phase Correct PWM
  cbi (TCCR1A, WGM21);
  cbi (TCCR1B, WGM22);

  sbi (TCCR2B, CS20);
  cbi (TCCR2B, CS21);
  cbi (TCCR2B, CS22);
  
  cbi (TCCR2A, COM2A0);  // clear Compare Match
  sbi (TCCR2A, COM2A1);
  sbi (TCCR2A, COM2B1);
  
  sbi (TCCR2A, WGM20);  // Mode 1  / Phase Correct PWM
  cbi (TCCR2A, WGM21);
  cbi (TCCR2B, WGM22);
  
 // cbi(ADCSRA,ADPS2) ;
 // sbi(ADCSRA,ADPS1) ;
 // cbi(ADCSRA,ADPS0) ;


  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);

  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);
  
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, INPUT_PULLUP);
  pinMode(13, INPUT_PULLUP);

  pinMode(14, INPUT_PULLUP);
  pinMode(15, INPUT_PULLUP);
  //

  delay(100);

  if (MIDI_ENABLE==1)
  {
      MIDI.begin();
  }

    if (MIDI_ENABLE==0)
  {
    Serial.begin(9600); 
  }

}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int16_t comb;
byte dds_tick;
ISR(TIMER1_OVF_vect)
{ 
  digitalWrite(13,1);

dds_tick=!dds_tick;

if (dds_tick==0)
{

if (dual_mode==1)
{

if (shape1<127)
{

  multi_voice(1,env_f,0,shape1,noise_mode_pin,dual_mode); // vn, amplitude,  offset,   shape,  noise_mode){

  comb=((out[1]*shapeB)>>8)+((out[2]*shapeA)>>8);
  

}

if (shape1>=127)
{
  multi_voice(1,env_f,255,shape1,noise_mode_pin,dual_mode); 
  comb=((out[2]*shapeA)>>8)+((out[1]*shapeB)>>8);

}

 
}

if (dual_mode==0)
{

  multi_voice(1,env_f,0,shape1,noise_mode_pin,dual_mode); // vn, amplitude,  offset,   shape,  noise_mode){

  comb=out[1];

}
}
 
if (dds_tick==1)
{

  if (comb>254)

  {
    int fold_amt=(comb-254);
    comb-=fold_amt<<1;
  }

    if (out[3]>254)

  {
    int fold_amt=(out[3]-254);
    out[3]-=fold_amt<<1;
  }

  OCR1A=(comb);  //9

  OCR1B=(out[3]);       //10
  //analogWrite(11,env_f);  

}

  digitalWrite(13,0);

}



void loop() {
  digitalWrite(12,1);


//////////////////////////////////////////////////////////////////miimmmmid

  if (MIDI_ENABLE==1)
  {
if (MIDI.read()) {                    
    byte type = MIDI.getType();
    switch (type) {
      case NoteOn:
        incoming_note = MIDI.getData1();
        //velocity = MIDI.getData2();
        channel = MIDI.getChannel();
        if (channel ==6) {}
          midi_note=incoming_note;
          midi_happend=1;
          digitalWrite(13,HIGH);
        
        break;

      case NoteOff:

        incoming_note = MIDI.getData1();
                channel = MIDI.getChannel();

        if (midi_note==incoming_note )
        {
          midi_note=0;
        }

        //velocity = MIDI.getData2();
        break;
    }
  }
  
}
//////////////////////////////////////////////////////////////////rec

  sh_pin=digitalRead(14);

  suboct_pin=digitalRead(6);
  detune_pin=digitalRead(5);
  range_pin=digitalRead(7);
  noise_mode_pin=digitalRead(8);


  oct_up=digitalRead(12);
  oct_down=digitalRead(13);

  if (suboct_pin==0 || detune_pin==0)
  {
    dual_mode=0;
  }

    if (suboct_pin==0 && detune_pin==0)
  {
    dual_mode=0;
  }

if (suboct_pin==1 && detune_pin==1)
  {
    dual_mode=1;
  }

prb=rec_button;
  rec_button=digitalRead(2);


  if (rec_button==0 && prb!=rec_button){
  if (recording==1)
  {
    loop_len=rec_cnt-2;
  }
  recording=!recording;
  playing=0;
  rec_cnt=0;
  }

  if (recording==1 && playing == 0){
    if ((millis()-recms)>10){
    recms=millis();
    rec_cnt++;
    }

    if (rec_cnt>rec_max-2){
      loop_len=rec_cnt-2;
      recording=0;
      rec_cnt=0;
    }

    rec_bank[rec_cnt]= vco1 | trig_button<<15;


  }

  ppb=play_button;
  play_button=digitalRead(4);

  if (play_button==0 && ppb!=play_button){
  playing=!playing;
  recording=0;
  }

  if (playing==1 && recording==0){

    if ((millis()-playcms)>10){
    playcms=millis();
    play_cnt++;
    }

    if (play_cnt>loop_len){
      play_cnt=0;
    }

    pltb=loop_trig_button;
    loop_trig_button=rec_bank[play_cnt]>>15;
    loop_vco1=rec_bank[play_cnt]<<1;
    loop_vco1=loop_vco1>>1;

  }


//////////////////////////////////////////////////////////////////env



  ptb=trig_button;
  trig_button=digitalRead(15);
  if (ptb==1 && trig_button==0)
  {
    env_fall=1;
    env_cnt[0]=0;
  }
  
  if (ptb==0){
    env_fall=0;
  }
  
  
  if (pltb==1 && loop_trig_button==0)
  {
    loop_fall=1;
        env_cnt[1]=0;

  }
  if (pltb==0){
    loop_fall=0;
  }


  env_b=enveloper(0,env_fall, trig_button, 0, 15);
  
  if (playing==1){
  env_loop=enveloper(1,loop_fall, loop_trig_button, 0, 15);
  }

  env_f=env_b+env_loop+vca1;

  if (env_f>255){
   // env_f=255;
  }


//////////////////////////////////////////////////////////////////pots


  pot_tick++;

  if (pot_tick>19)
  {
    pot_tick=0;
  }

    //pot_tick=0;


  if (pot_tick==0)  {
 // digitalWrite(8,1);
int raw5=analogRead(5);
  //vco1f=chromatic[note_step];

  if (midi_happend==1 && sh_pin==1){
    if (midi_note>0){
  vco1=raw5-512+midi_freq_table[midi_note];
  if (vco1<1)
  {
    vco1=1;
  }

    if (vco1>4000)
  {
    vco1=4000;
  }
  }
  if (midi_note==0)
  {
    vco1=0;
  }
}

  if (playing==1 && sh_pin==1){
  vco1=raw5+loop_vco1-512;
  if (vco1<1)
  {
    vco1=1;
  }

    if (vco1>3000)
  {
    vco1=3000;
  }

    

  }

  if (playing==0 && sh_pin==1 && midi_happend==0){
    loop_trig_button=0;
    env_loop=0;
    loop_vco1=0;
    vco1=raw5;

  }

    if (oct_up==0)
    {
      vco1=vco1<<1;
    }

    if (oct_down==0)
    {
      vco1=vco1>>1;
    }

    if (oct_down==0 && oct_up==0)
    {
      vco1=(vco1<<2);
    }
    
}

if (pot_tick==5){
  if (detune_pin==0){ 
  vco2=vco1-(vco1>>7);
}

  if (suboct_pin==0){ 
  vco2=vco1>>1;
}

  if (detune_pin==0 && suboct_pin==0){ 
  vco2=vco1>>2;
}

  if (range_pin==1){ 
  increment[1]=(pow(2,32)*vco1)/(15688);  //3137660
  increment[2]=(pow(2,32)*(vco2))/(15688);  //3137660
}

  if (range_pin==0){
  vco1=vco1<<1;
  increment[1]=(pow(2,32)*(vco1<<1))/(1568830);  //3137660
  increment[2]=(pow(2,32)*(vco1<<1))/(1568830);  //3137660
}
    //digitalWrite(8,0);
}


  

  if (pot_tick==10)  {//////////////***********************************************************LED

  if (recording==1){

  if ((millis()-ledms)>25){
    ledms=millis();
    status_led=200;
  }
  }

  if (playing==1){

  if ((millis()-ledms)>(loop_len<<1)){
    ledms=millis();
              direct=!direct;
    if (direct==0){status_led=90;}
    if (direct==1){status_led=20;}

  }
  }

  if (playing==0 && recording==0){
      if ((millis()-ledms)>30){
    ledms=millis();
    if (direct==0){status_led++;}
    if (direct==1){status_led--;}
    if (status_led>45 ){
          direct=1;
    }
    if (status_led<2 ){
          direct=0;
    }
    if (status_led>80 ){
      status_led=44;
          direct=1;
    }
    }
}


  analogWrite(11,status_led);  

  }

  if (pot_tick==15)  {
  vca1=(analogRead(4)>>1)-16;
  if (vca1<0)
  {
    vca1=0;
  }
}

  if (pot_tick==3 && sh_pin==1)  {
  shape1=(analogRead(3)>>2);
  if (shape1<127){
  shapeA=shape1<<1;
  shapeB=(shape1-127)*-2;
}
  

    if (shape1>=127){
  shapeA=(shape1-127)<<1;
  shapeB=(shape1-255)*-2;
}
  

  }






  digitalWrite(12,0);

/*
  if ((millis()-prev)>400 && MIDI_ENABLE==0){
    prev=millis();

    note_step +=random(2,12);
    if (note_step >110){
      note_step=10;
    }
    Serial.println(vco1);       
          Serial.println(); 

          
  }
 */
}


void multi_voice(byte vn,uint16_t amplitude, uint16_t offset,  uint16_t shape, byte noise_mode, byte dual){
  int16_t temp,tempB,squtemp,outf,outfB,squout,temp2,tempB2,squ2temp;
  
if (dual==0){ //on

  accumulator[vn] += increment[vn];
  waveindex[vn]=((accumulator[vn]) >> 24); 

  if (shape>250)
  {
    shape=255;
  }

  temp = pgm_read_byte(&waveTable2[waveindex[vn]]+shape)-127;
  //tempB = pgm_read_byte(&waveTable2[waveindex[vn]+255+offset]);

  squtemp = pgm_read_byte(&squareTable[waveindex[vn]]+shape);

  accumulator[2] += increment[2];
  waveindex[2]=((accumulator[2]) >> 24); 

  temp2 = pgm_read_byte(&waveTable2[waveindex[2]]+shape)-127;
  //tempB2 = pgm_read_byte(&waveTable2[waveindex[2]+255+offset]);

  squ2temp = pgm_read_byte(&squareTable[waveindex[2]]+shape);




  if (noise_mode==0){ //on
  outf=(((temp+temp2)^amplitude)<<2)&amplitude;   //^ gives facets. | simplifies, & |s the other direction % folds but kills amplitude
  outfB=(((tempB+tempB2)^amplitude)<<2)&amplitude; //this is the guy. noise and volume control!
  squout=((((squtemp+squ2temp)|waveindex[vn])<<3)^(squtemp))&amplitude; //this is the guy. noise and volume control!
}

if (noise_mode==1){
  outf=((temp+temp2)>>1)+127;
  outf=(outf*amplitude)>>8;
  //outfB=(((temp))*amplitude)>>8;
  squout=(((squtemp+squ2temp)>>1)*(amplitude>>1))>>8;

}

  out[vn]=outf;
  //out[vn+1]=outfB;
  out[vn+2]=squout;

}


if (dual==1){

  accumulator[vn] += increment[vn];
  waveindex[vn]=((accumulator[vn]) >> 24); 

  temp = pgm_read_byte(&waveTable2[waveindex[vn]]+offset);
  tempB = pgm_read_byte(&waveTable2[waveindex[vn]+255+offset]);

  squtemp = pgm_read_byte(&squareTable[waveindex[vn]]+shape);

  if (noise_mode==0){
  outf=((temp^amplitude)<<2)&amplitude;   //^ gives facets. | simplifies, & |s the other direction % folds but kills amplitude
  outfB=((tempB^amplitude)<<2)&amplitude; //this is the guy. noise and volume control!
  squout=(((squtemp|waveindex[vn])<<3)^(squtemp))&amplitude; //this is the guy. noise and volume control!
}

if (noise_mode==1){
  outf=(temp*amplitude)>>8;
  outfB=(tempB*amplitude)>>8;
  squout=(squtemp*(amplitude>>1))>>8;

}

}



  out[vn]=outf;
  out[vn+1]=outfB;
  out[vn+2]=squout;

}



byte enveloper(byte vn,byte env_t, byte button, byte a_t, byte d_t){

if (env_t==1)
{
  env_mode[vn]=1;
}

  if (env_mode[vn]==1) //attack
  {
    env_dly[vn]++;
    if (env_dly[vn]>a_t)
    {

    env_cnt[vn]++;
    env_cnt[vn]+=env_cnt[vn]>>4;
    env_dly[vn]=0;
        }

    if (env_cnt[vn]>=200)
    {
      env_mode[vn]=2;
    }

    env_out[vn]=env_cnt[vn];
}
  

  if (env_mode[vn]==2) {
    if (button==1)
    {
      env_mode[vn]=3;
    }
  }

  if (env_mode[vn]==3) //release
  {

    env_dly[vn]++;
    if (env_dly[vn]>d_t)
    {
    env_cnt[vn]-=env_cnt[vn]>>4;
          env_dly[vn]=0;
        }
 

      if (env_cnt[vn]<=16)
      {
        env_cnt[vn]=0;
        env_mode[vn]=0;
      }

       env_out[vn]=env_cnt[vn];
    
  }

  if (env_mode[vn]==0){
    env_out[vn]=0;
    env_cnt[vn]=0;
    env_dly[vn]=0;
  }
  return env_out[vn];

}




