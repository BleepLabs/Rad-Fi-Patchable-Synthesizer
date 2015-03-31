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
#include <TimerOne.h>

int drymix,wetmix,dely_in;
int32_t write_head,read_head;
int32_t lerp_dly,lerp_len, filter, feedback,fb_tmp_inv,fb_tmp,lerp_tick,dly_length;
byte jjj;

const unsigned char PROGMEM bandlimited_table[803] =
{  
//saw
127,137,147,156,166,175,184,192,200,208,215,222,228,233,238,242,245,248,250,252,253,253,253,253,251,250,248,246,243,240,237,234,231,227,224,220,217,214,211,208,205,203,201,199,197,196,194,193,193,192,192,192,192,192,192,193,193,194,194,195,195,196,196,196,196,196,196,195,195,194,193,192,191,189,187,186,184,182,180,178,176,173,171,169,167,165,163,161,160,158,157,155,154,153,152,151,151,150,150,150,150,150,150,150,150,150,150,150,150,149,149,149,149,148,147,147,146,145,143,142,141,139,137,136,134,132,130,128,126,124,122,120,118,117,115,113,112,111,109,108,107,107,106,105,105,105,105,104,104,104,104,104,104,104,104,104,104,104,104,103,103,102,101,100,99,97,96,94,93,91,89,87,85,83,81,78,76,74,72,70,68,67,65,63,62,61,60,59,59,58,58,58,58,58,58,59,59,60,60,61,61,62,62,62,62,62,62,61,61,60,58,57,55,53,51,49,46,43,40,37,34,30,27,23,20,17,14,11,8,6,4,3,1,1,1,1,2,4,6,9,12,16,21,26,32,39,46,54,62,70,79,88,98,107,117,127,
//TRI
127,133,139,145,151,156,162,167,172,176,180,184,188,191,194,197,200,202,204,207,209,211,212,214,216,218,220,222,224,226,227,229,231,233,234,236,237,239,240,241,242,243,244,245,245,246,247,247,248,248,249,249,250,250,251,251,252,252,253,253,254,254,254,254,254,254,254,254,253,253,253,252,252,251,251,250,250,249,248,248,247,247,246,246,245,244,243,243,242,240,239,238,237,235,233,232,230,228,227,225,223,221,219,217,215,213,211,210,208,205,203,201,198,196,193,190,186,182,178,174,169,164,159,154,148,142,136,130,124,118,112,106,100,95,90,85,80,76,72,68,64,61,58,56,53,51,49,46,44,43,41,39,37,35,33,31,29,27,26,24,22,21,19,17,16,15,14,12,11,11,10,9,8,8,7,7,6,6,5,4,4,3,3,2,2,1,1,1,0,0,0,0,0,0,0,0,1,1,2,2,3,3,4,4,5,5,6,6,7,7,8,9,9,10,11,12,13,14,15,17,18,20,21,23,25,27,28,30,32,34,36,38,40,42,43,45,47,50,52,54,57,60,63,66,70,74,78,82,87,92,98,103,109,115,121,127,
//squ
127,144,161,178,193,207,219,230,239,246,251,254,255,255,253,250,246,242,237,232,227,223,219,216,213,211,211,210,211,212,214,216,218,220,223,224,226,227,228,228,228,227,226,224,223,221,219,218,216,215,215,214,214,214,215,216,217,219,220,222,223,224,225,225,225,225,224,223,222,221,219,218,217,216,215,214,214,214,215,216,217,219,220,222,224,225,226,227,228,228,228,227,225,224,222,219,217,215,213,212,211,210,211,212,214,217,221,225,230,235,239,244,248,252,254,255,255,253,249,243,235,225,213,200,186,170,153,136,118,101,84,68,54,41,29,19,11,5,3,1,0,0,2,6,10,15,19,24,29,33,37,40,42,43,44,43,42,41,39,37,35,32,30,29,27,26,26,26,27,28,29,30,32,34,35,37,38,39,40,40,40,39,38,37,36,35,33,32,31,30,29,29,29,29,30,31,32,34,35,37,38,39,40,40,40,39,39,38,36,35,33,31,30,28,27,26,26,26,27,28,30,31,34,36,38,40,42,43,44,43,43,41,38,35,31,27,22,17,12,8,4,2,1,0,0,3,8,15,24,35,47,61,76,93,110,127,
118,101,84,68,54,41,29,19,11,5,3,1,0,0,2,6,10,15,19,24,29,33,37,40,42,43,44,43,42,41,39,37,35,32,30//,29,27,26,26,26,27,28,29,30,32,34,35//,37,38,39,40,40,40,39,38,37,36,35,33,32,31,30,29

};
float midi_freq_table[128]={ 
8.1758,8.6620,9.1770,10.3009,10.3009,10.9134,11.5623,12.2499,12.9783,13.7500,14.5676,15.4339,16.3516,17.3239,18.3540,19.4454,20.6017,21.8268,23.1247,24.4997,25.9565,27.5000,29.1352,30.8677,32.7032,34.6478,36.7081,38.8909,41.2034,43.6535,46.2493,48.9994,51.9131,55.0000,58.2705,61.7354,65.4064,69.2957,73.4162,77.7817,82.4069,87.3071,92.4986,97.9989,103.8262,110.0000,116.5409,123.4708,130.8128,138.5913,146.8324,155.5635,164.8138,174.6141,184.9972,195.9977,207.6523,220.0000,233.0819,246.9417,261.6256,277.1826,293.6648,311.1270,329.6276,349.2282,369.9944,391.9954,415.3047,440.0000,466.1638,493.8833,523.2511,554.3653,587.3295,622.2540,659.2551,698.4565,739.9888,783.9909,830.6094,880.0000,932.3275,987.7666,1046.5023,1108.7305,1174.6591,1244.5079,1318.5102,1396.9129,1479.9777,1567.9817,1661.2188,1760.0000,1864.6550,1975.5332,2093.0045,2217.4610,2349.3181,2489.0159,2637.0205,2793.8259,2959.9554,3135.9635,3322.4376,3520.0000,3729.3101,3951.0664,4186.0090,4434.9221,4698.6363,4978.0317,5274.0409,5587.6517,5919.9108,5919.9108,6644.8752,7040.0000,7458.6202,7902.1328,8372.0181,8869.8442,9397.2726,9956.0635,10548.0818,11175.3034,11839.8215
};


const unsigned char PROGMEM LFO_table[1251] =
{
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,255,255,255,255,254,253,252,251,250,249,248,247,246,245,244,243,242,241,240,239,238,237,236,235,234,233,232,231,230,229,228,227,226,225,224,223,222,221,220,219,218,217,216,215,214,213,212,211,210,209,208,207,206,205,204,203,202,201,200,199,198,197,196,195,194,193,192,191,190,189,188,187,186,185,184,183,182,181,180,179,178,177,176,175,174,173,172,171,170,169,168,167,166,165,164,163,162,161,160,159,158,157,156,155,154,153,152,151,150,149,148,147,146,145,144,143,142,141,140,139,138,137,136,135,134,133,132,131,130,129,128,127,126,125,124,123,122,121,120,119,118,117,116,115,114,113,112,111,110,109,108,107,106,105,104,103,102,101,100,99,98,97,96,95,94,93,92,91,90,89,88,87,86,85,84,83,82,81,80,79,78,77,76,75,74,73,72,71,70,69,68,67,66,65,64,63,62,61,60,59,58,57,56,55,54,53,52,51,50,49,48,47,46,45,44,43,42,41,40,39,38,37,36,35,34,33,32,31,30,29,28,27,26,25,24,23,22,21,20,19,18,17,16,15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0,0,0,0,0,2,4,6,8,10,12,14,16,18,20,22,24,26,28,30,32,34,36,38,40,42,44,46,48,50,52,54,56,58,60,62,64,66,68,70,72,74,76,78,80,82,84,86,88,90,92,94,96,98,100,102,104,106,108,110,112,114,116,118,120,122,124,126,128,130,132,134,136,138,140,142,144,146,148,150,152,154,156,158,160,162,164,166,168,170,172,174,176,178,180,182,184,186,188,190,192,194,196,198,200,202,204,206,208,210,212,214,216,218,220,222,224,226,228,230,232,234,236,238,240,242,244,246,248,250,252,254,252,250,248,246,244,242,240,238,236,234,232,230,228,226,224,222,220,218,216,214,212,210,208,206,204,202,200,198,196,194,192,190,188,186,184,182,180,178,176,174,172,170,168,166,164,162,160,158,156,154,152,150,148,146,144,142,140,138,136,134,132,130,128,126,124,122,120,118,116,114,112,110,108,106,104,102,100,98,96,94,92,90,88,86,84,82,80,78,76,74,72,70,68,66,64,62,60,58,56,54,52,50,48,46,44,42,40,38,36,34,32,30,28,26,24,22,20,18,16,14,12,10,8,6,4,2,0,0,0,0,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,127,128,129,130,131,132,133,134,135,136,137,138,139,140,141,142,143,144,145,146,147,148,149,150,151,152,153,154,155,156,157,158,159,160,161,162,163,164,165,166,167,168,169,170,171,172,173,174,175,176,177,178,179,180,181,182,183,184,185,186,187,188,189,190,191,192,193,194,195,196,197,198,199,200,201,202,203,204,205,206,207,208,209,210,211,212,213,214,215,216,217,218,219,220,221,222,223,224,225,226,227,228,229,230,231,232,233,234,235,236,237,238,239,240,241,242,243,244,245,246,247,248,249,250,251,252,253,254,255,255,255,255,255,255};

#define rec_max 300

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

byte tick,lfo_rate_pin;
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
byte note_bank[16]={};
int16_t pot2,pot3; 
int16_t to_dly_temp;
byte pot_tick;
int16_t ain,dly_out;
int16_t vco1,vco2,vca1,vca2,shape1,shape2;
int16_t vcaA,vcaB;
int raw5;
int dds_rate=10;
/*
const float chromatic[129]={
  0,16.35,17.32,18.35,19.45,20.6,21.83,23.12,24.5,25.96,27.5,29.14,30.87,32.7,34.65,36.71,38.89,41.2,43.65,46.25,49,51.91,55,58.27,61.74,65.41,69.3,73.42,77.78,82.41,87.31,92.5,98,103.83,110,116.54,123.47,130.81,138.59,146.83,155.56,164.81,174.61,185,196,207.65,220,233.08,246.94,261.63,277.18,293.66,311.13,329.63,349.23,369.99,392,415.3,440,466.16,493.88,523.25,554.37,587.33,622.25,659.25,698.46,739.99,783.99,830.61,880,932.33,987.77,1046.5,1108.73,1174.66,1244.51,1318.51,1396.91,1479.98,1567.98,1661.22,1760,1864.66,1975.53,2093,2217.46,2349.32,2489.02,2637.02,2793.83,2959.96,3135.96,3322.44,3520,3729.31,3951.07,4186.01,4434.92,4698.63,4978.03,5274.04,5587.65,5919.91,6271.93,6644.88,7040,7458.62,7902.13};
*/
byte note_step,trig_button,ptb;
long attackms,relms;
float vco2f=220;
float vco1f=220;
float vco_f;
byte prb,rec_button,recording,ppb,play_button,playing,pltb,rec_led,env_fall,loop_fall,env_b,env_loop,status_led,shift_button,env_lock;
uint16_t rec_cnt,play_cnt,loop_trig_button,loop_vco1,env_f;
int16_t loop_len,shapeB,shapeA;
uint32_t playcms,recms,ledms;
byte direc=1;
byte noise_mode_pin,range_pin,dual_voice_pin,vco2m,dual_mode,suboct_pin,detune_pin,sh_pin,direct;
byte MIDI_ENABLE=1;
byte lfo_amp_pin;
byte midi_note,channel,incoming_note;
byte midi_happend=0;
uint16_t tick_dly;
byte oct_up,oct_down;
byte midi_velocity;
byte env_out[4]={};
byte env_mode[4]={};
byte env_dly[4]={};
byte env_cnt[4]={};
byte midi_trig_button=1;
byte smooth_cnt[4]={};
int smooth_ar[4][8]={};
byte midi_env,midi_fall,midi_env_button,pmt;
byte cc_atk=0;
byte cc_rel=10;
byte cc_happend,cc_d1,cc_d2;
int pcnt;
byte cc_raw=0;
byte plock;
byte off_happened=0;
byte set_ch=4;


void setup() {

  TCCR0B = TCCR0B & 0b11111000 | 0x01 ; //timer 0 62500Hz

  const unsigned char PS_16 = (1 << ADPS2);
  const unsigned char PS_32 = (1 << ADPS2) | (1 << ADPS0);
  const unsigned char PS_64 = (1 << ADPS2) | (1 << ADPS1);
  const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

  //ADCSRA &= ~PS_128;
  //ADCSRA |=PS_64;


  Timer1.initialize(65); //65 =15.4 70=14.29 //75=13.33
  Timer1.attachInterrupt(DDS); 


  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);

  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);

  pinMode(7, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);
  
  pinMode(9, INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);
  pinMode(11, OUTPUT);
  pinMode(12, INPUT_PULLUP);
  pinMode(13, INPUT_PULLUP);

  pinMode(14, INPUT_PULLUP);
  pinMode(15, INPUT_PULLUP);

  pinMode(16, OUTPUT);

  //

  delay(100);

if (digitalRead(3)==0)
{
  set_ch=3;
}

if (digitalRead(3)==1)
{
  set_ch=1;
}

      MIDI.begin(set_ch);

  //mySerial.println("Hello, world?");

}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int16_t comb;
byte dds_tick;

void DDS()
{ 
  //digitalWrite(13,1);

//dds_tick=!dds_tick;


  multi_voice(1,env_f,0,shape1,noise_mode_pin,dual_mode); // vn, amplitude,  offset,   shape,  noise_mode){

  if (out[1]>254)

  {
    int fold_amt=(comb-254);
    comb-=fold_amt<<1;
  }

  accumulator[3] += increment[3];        
  waveindex[3]=((accumulator[3]) >> 24); 

  int temp = pgm_read_byte(&LFO_table[waveindex[3]+(shape1)]);

  if (lfo_amp_pin==1)
  {
      out[3]=(temp*vca1)>>8;
  }

    if (lfo_amp_pin==0)
  {
      out[3]=temp;
  }

  if (out[3]>254)
  {
    int fold_amt=(out[3]-254);
    out[3]-=fold_amt<<1;
  }

  //OCR1A=(comb);  //9
  //OCR1B=(out[3]);       //10

  analogWrite(5,out[1]);  
  analogWrite(6,(out[3]));  


 // digitalWrite(13,0);

}



void loop() {
 // digitalWrite(12,1);


  pot_tick++;

  if (pot_tick>6)
  {

    pot_tick=0;
  }


//////////////////////////////////////////////////////////////////miimmmmid

  if (pot_tick==1)
  {  

  // digitalWrite(13,0);


  pmt=midi_trig_button;

 // if (MIDI_ENABLE==1){
if (MIDI.read(set_ch)) { 
     digitalWrite(11,HIGH);

    byte type = MIDI.getType();
    switch (type) {
      case NoteOn:
        incoming_note = MIDI.getData1();
        midi_velocity = MIDI.getData2();
   
        if (midi_velocity>0){
        
          pcnt=0;



          for (int i = 0; i < 6; i++)
          {

            if (note_bank[i]==0 && plock==0){
            note_bank[pcnt]=incoming_note;
            plock=1;
            }

            if (note_bank[i]>0 && plock==0){

            pcnt++;
            }

          if (pcnt>5){
            pcnt=5;
          }

          }
          

          midi_note=note_bank[pcnt];
          midi_happend=1;
          midi_trig_button=0;
          midi_fall=1;
          plock=0;
      
        }

        if (midi_velocity==0 && off_happened==0)
        {
          off_happened=1;
          if (note_bank[0]==10000){
          midi_note=note_bank[0];
          note_bank[0]=0;
          pcnt=0;

          }
          

          for (int i = 0; i <6  ; i++)
          {

            if (note_bank[i]==incoming_note){
              if (pcnt>0)
              {
                pcnt--;
              }

              for (int j = i; j < 6; j++)
              {
                note_bank[j]=note_bank[j+1];
                note_bank[j+1]=0;
              }
              
            }



           }
            midi_note=note_bank[pcnt];
            if (pcnt==0 && note_bank[0]==0){
            midi_trig_button=1;

            midi_note=incoming_note;
           }


        }


        break;

      case NoteOff:


        if (off_happened==0)
        {
            off_happened=1;

        

          for (int i = 0; i <6  ; i++)
          {

            if (note_bank[i]==incoming_note){
              if (pcnt>0)
              {
                pcnt--;
              }

              for (int j = i; j < 6; j++)
              {
                note_bank[j]=note_bank[j+1];
                note_bank[j+1]=0;
              }
              
            }



           }
            midi_note=note_bank[pcnt];
            if (pcnt==0 && note_bank[0]==0){
            midi_trig_button=1;

            midi_note=incoming_note;
           }

}
        break;
    
  
      case ControlChange:

       cc_d1 = MIDI.getData1();
       cc_d2  = MIDI.getData2();

        if (cc_d1==22 )
        {
        if (cc_d2>120){
         cc_raw=1;}
        
        if (cc_d2<120){
         cc_raw=0;}
        
        }

        if (cc_d1==20 )
        {
        // analogWrite(11,200);  

          cc_atk=cc_d2>>1;
        }
        if (cc_d1==21 )
        {
        // analogWrite(11,200);  

          cc_rel=cc_d2;
        }


        break;


        //default:
        //midi_trig_button=1;
        

    

  }
}

off_happened=0;
    if (pmt==1 && midi_trig_button==0)
  { 
    midi_fall=1;
    //env_cnt[2]=env_cnt[2]>>1;

  }

  if (pmt==0){
    midi_fall=0;
  }
  
}
//////////////////////////////////////////////////////////////////rec


  if (pot_tick==3)
  {


  
  suboct_pin=digitalRead(7);
  detune_pin=digitalRead(8);

  noise_mode_pin=digitalRead(9);
  sh_pin=digitalRead(10);

  range_pin=1;

  oct_up=digitalRead(12);
  oct_down=digitalRead(13);

lfo_amp_pin= digitalRead(14);
lfo_rate_pin=digitalRead(15);
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
    loop_len=rec_cnt-1;
  }
  recording=!recording;
  playing=0;
  rec_cnt=0;
  }

  if (recording==1 && playing == 0){
    if ((millis()-recms)>500){ //not actually a .5 second since timer one is changed
    recms=millis();
    rec_cnt++;
    }

    if (rec_cnt>rec_max-1){
      loop_len=rec_cnt-1;
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

    if ((millis()-playcms)>500){ //not actually a .5 second since timer one is changed
    playcms=millis();
    play_cnt++;
    }

    if (play_cnt>loop_len){
      play_cnt=1;
    }

    pltb=loop_trig_button;
    loop_trig_button=rec_bank[play_cnt]>>15;
    loop_vco1=rec_bank[play_cnt]<<1;
    loop_vco1=loop_vco1>>1;

  }
}


//////////////////////////////////////////////////////////////////pots

  if (pot_tick==0 && sh_pin==1)  {
 // digitalWrite(8,1);
 //raw5=120;

    //int smooth_5=cheap_smooth(0,analogRead(5));
    //raw5=(smooth_5);

    raw5=analogRead(5);
  //vco1f=chromatic[note_step];

  if (midi_happend==1 ){
    if (midi_note>0){
  //vco_f=raw5-512+midi_freq_table[midi_note];

    

  vco_f=midi_freq_table[midi_note]+((raw5>>2)-128);    


    if (oct_up==0)
    {
      vco_f*=2;
    }

    if (oct_down==0)
    {
      vco_f/=2;
    }

    if (oct_down==0 && oct_up==0)
    {
            vco_f*=4;
    }

  vco1=vco_f;
  if (vco_f<1)
  {
    vco_f=1;
  }

    if (vco_f>4000)
  {
   // vco_f=4000;
  }
  }
  if (midi_note==0)
  {
    vco_f=0;
  }





}

  if (playing==1 ){
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

  if (playing==0  && midi_happend==0){
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



  if (pot_tick==2)  {

  //int smooth_4=cheap_smooth(1,analogRead(4));
  //vca1=(smooth_4)-32;

  vca1=(analogRead(4)>>1)-32;
  if (vca1<0)
  {
    vca1=0;
  }
}

  if (pot_tick==4)  {


 // int smooth_3=cheap_smooth(2,analogRead(3));
 //shape1=(smooth_3);
 //shape2=(smooth_3);
 shape1=(analogRead(3));

//shape1+=12;

if (shape1>512){
  //shape1=512;
}

  }

if (pot_tick==6){

if (midi_happend==0)
{

  if (detune_pin==0){ 
  vco2=vco1-(vco1>>7);
}

  if (suboct_pin==0){ 
  vco2=vco1>>1;
}

  if (detune_pin==0 && suboct_pin==0){ 
  vco2=vco1>>2;
}
  
  increment[1]=(pow(2,32)*vco1)/(15385);  //3137660
  increment[2]=(pow(2,32)*(vco2))/(15385);  //3137660

if (lfo_rate_pin==1)
{
  increment[3]=(pow(2,32)*(vco1<<3))/(15385000);  //3137660
}

if (lfo_rate_pin==0)
{
  increment[3]=(pow(2,32)*(vco1<<3))/(153850);  //3137660
}

}

if (midi_happend==1)
{
  float vco_f2;

  if (detune_pin==0){ 
  vco_f2=vco_f-(vco_f/128);
}

  if (suboct_pin==0){ 
  vco_f2=vco_f/2;
}

  if (detune_pin==0 && suboct_pin==0){ 
  vco_f2=vco_f/4;
}

  increment[1]=(pow(2,32)*vco_f)/(15385);  //3137660
  increment[2]=(pow(2,32)*(vco_f2))/(15385);  //3137660

if (lfo_rate_pin==1)
{
  increment[3]=(pow(2,32)*(vco1<<3))/(15385000);  //3137660
}

if (lfo_rate_pin==0)
{
  increment[3]=(pow(2,32)*(vco1<<3))/(153850);  //3137660
}

}

  

}


  

  if (pot_tick==5)  {//////////////***********************************************************LED

  if (recording==1){

  if ((millis()-ledms)>25){
    ledms=millis();
    status_led=200;
  }
  }

  if (playing==1){
      if (loop_trig_button==0){status_led=200;}

  if ((millis()-ledms)>(loop_len<<5)){
    ledms=millis();
    direct=!direct;
    if (direct==0){status_led=120;}
    if (direct==1){status_led=20;}


  }


  }

  if (playing==0 && recording==0){
      if ((millis()-ledms)>800){
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


//////////////////////////////////////////////////////////////////env


  ptb=trig_button;
  trig_button=digitalRead(16);

  if (ptb==1 && trig_button==0)
  {
    env_fall=1;
//    env_cnt[0]=0;
       // env_cnt[0]=env_cnt[0]>>1;

  }
  
  if (ptb==0){
    env_fall=0;
  }
  
  
  if (pltb==1 && loop_trig_button==0)
  {
    loop_fall=1;
   // env_cnt[1]=0;
     //   env_cnt[1]=env_cnt[1]>>1;


  }
  if (pltb==0){
    loop_fall=0;
  }



if (cc_raw==0){
  midi_env=enveloper(2,midi_fall, midi_trig_button, cc_atk, cc_rel);
}
if (cc_raw==1){
midi_env=midi_velocity;
}

  env_b=enveloper(0,env_fall, trig_button, cc_atk, cc_rel);
  
  if (playing==1){
  env_loop=enveloper(1,loop_fall, loop_trig_button, cc_atk, cc_rel);
  }

  int env_c=env_b+env_loop+vca1+midi_env;
  env_f=env_c;





}



void multi_voice(byte vn,uint16_t amplitude, uint16_t offset,  uint16_t shape, byte noise_mode, byte dual){
  int16_t temp,tempB,squtemp,outf,outfB,squout,temp2,tempB2,squ2temp;
  
if (dual==0){ //on

  accumulator[vn] += increment[vn];
  waveindex[vn]=((accumulator[vn]) >> 24); 

  accumulator[vn+1] += increment[vn+1];
  waveindex[vn+1]=((accumulator[vn+1]) >> 24); 


  temp = pgm_read_byte(&bandlimited_table[waveindex[vn]]+shape)-127;
  temp2 = pgm_read_byte(&bandlimited_table[waveindex[vn+1]]+shape)-127;


  if (noise_mode==0){ //on
  outf=(((temp+temp2)^amplitude)<<2)&amplitude;   //^ gives facets. | simplifies, & |s the other direction % folds but kills amplitude
 }

if (noise_mode==1){
  outf=((temp+temp2)>>1)+127;
  outf=(outf*amplitude)>>8;
 }

  out[vn]=outf;
  }


if (dual==1){ //off

  accumulator[vn] += increment[vn];
  waveindex[vn]=((accumulator[vn]) >> 24); 

  temp = pgm_read_byte(&bandlimited_table[waveindex[vn]]+shape);

  //squtemp = pgm_read_byte(&squareTable[waveindex[vn]]+shape);

  if (noise_mode==0){
  outf=((temp^amplitude)<<2)&amplitude;   //^ gives facets. | simplifies, & |s the other direction % folds but kills amplitude
 }

if (noise_mode==1){
  outf=(temp*amplitude)>>8;
}

}

  out[vn]=outf;

}


void LF_voice(byte vn,uint16_t amplitude, uint16_t shape){

  int16_t temp,tempB,squtemp,outf,outfB,squout,temp2,tempB2,squ2temp;
  


  accumulator[vn] += increment[vn];
  waveindex[vn]=((accumulator[vn]) >> 24); 

  temp = pgm_read_byte(&LFO_table[waveindex[vn]]+(shape>>1))-127;

  out[vn]=temp;

  }




byte enveloper(byte vn,byte env_t, byte button, byte a_t, byte d_t){

static byte env_step=8;

if (env_t==1)
{
  env_mode[vn]=1;
}

  if (env_mode[vn]==1) //attack
  {
    if (env_cnt[vn]<200){
      env_dly[vn]++;
      if (env_dly[vn]>a_t)
      {
      env_cnt[vn]+=env_step;
      env_cnt[vn]+=env_cnt[vn]>>4;
      env_dly[vn]=0;
      }
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
    env_cnt[vn]-=env_step;

    env_cnt[vn]-=env_cnt[vn]>>4;
          env_dly[vn]=0;
        }
 

      if (env_cnt[vn]<=env_step)
      {
        env_cnt[vn]=0;
        env_mode[vn]=0;
      }

       env_out[vn]=env_cnt[vn];
    
  }

  if (env_mode[vn]==4) // quick release
  {
   if (env_cnt[vn]>8)
      {
    env_cnt[vn]-=8;
     }
    //env_cnt[vn]-=env_cnt[vn]>>4;
  
      if (env_cnt[vn]<=8)
      {
        env_cnt[vn]=0;
        env_mode[vn]=1;
      }

    env_out[vn]=env_cnt[vn];
    
  }

  if (env_mode[vn]==0){
    env_cnt[vn]=0;
    env_dly[vn]=0;
  }
  return env_out[vn];

}





int cheap_smooth(byte place, int input){     
  // http://playground.arduino.cc/Main/DigitalSmooth without the costly divdes
  int j, k, temp, top, bottom;
  long total;
  static int i;
  static byte filterSamples=8;
  static int sorted[8];
  boolean done;


i++;
if (i>filterSamples-1){
  i=0;
}  

smooth_ar[place][i] = input;                 // input new data into the oldest slot

  total = 0;
  for ( j = 0; j< 8; j++){
    total += smooth_ar[place][j];  // total remaining indices
  }

  return total >>3;
}

/*
int not_cheap_smooth(byte place, int input){     
  // http://playground.arduino.cc/Main/DigitalSmooth without the costly divdes
  int j, k, temp, top, bottom;
  long total;
  static int i;
  static byte filterSamples=10;
  static int sorted[10];
  boolean done;

i++;
if (i>filterSamples-1){
  i=0;
}  
smooth_ar[place][i] = input;                 // input new data into the oldest slot

  // Serial.print("raw = ");

  for (j=0; j<filterSamples; j++){     // transfer data array into anther array for sorting and averaging
    sorted[j] = smooth_ar[place][j];
  }

  done = 0;                // flag to know when we're done sorting              
  while(done != 1){        // simple swap sort, sorts numbers from lowest to highest
    done = 1;
    for (j = 0; j < (filterSamples - 1); j++){
      if (sorted[j] > sorted[j + 1]){     // numbers are out of order - swap
        temp = sorted[j + 1];
        sorted [j+1] =  sorted[j] ;
        sorted [j] = temp;
        done = 0;
      }
    }
  }

  // throw out top and bottom sample
  total = 0;
  for ( j = 1; j< 8; j++){
    total += sorted[j];  // total remaining indices
    k++; 
  }

  return total >>3;
}
*/
