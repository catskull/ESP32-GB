#include <Arduino.h>
#include <BLEMIDI_Transport.h>
#include <hardware/BLEMIDI_ESP32_NimBLE.h>

BLEMIDI_CREATE_INSTANCE("GB MIDI", MIDI)

#define LED_BUILTIN 2
#define LED_MGB     12
#define LED_NL      13
#define LED_KB      5
#define LED_LEAD    23
#define LED_SYNC    19
#define LED_STATUS  18

int pinLeds[] = {
  LED_MGB,
  LED_NL,
  LED_KB,
  LED_LEAD,
  LED_SYNC,
  LED_STATUS
}; // LED Pins

#define PIN_GB_CLOCK      4
#define PIN_GB_SERIAL_OUT 35
#define PIN_GB_SERIAL_IN  34

#define GB_MIDI_DELAY 500 //Microseconds to delay the sending of a byte to gb
#define GB_SET(bit_cl, bit_out, bit_in) digitalWrite(PIN_GB_CLOCK, bit_cl); digitalWrite(PIN_GB_SERIAL_OUT, bit_out); digitalWrite(PIN_GB_SERIAL_IN, bit_in);

uint16_t blinkMaxCount = 1000;
boolean blinkSwitch[6];
unsigned long int blinkSwitchTime[6];
uint8_t switchLight = 0;
boolean statusLedIsOn    =false;
boolean statusLedBlink   =false;
int countStatusLedOn =0;

void updateStatusLed()
{
  if(statusLedIsOn) {                  //is the led on?
    countStatusLedOn++;                //then increment the counter by 1
    if(countStatusLedOn > 3000) {      //if the counter is pretty high
      countStatusLedOn = 0;            //then reset it to zero.
       digitalWrite(LED_STATUS,LOW); //and turn off the status led
       statusLedIsOn  = false;         //and set our "is it on?" to false, cause its off now. ;p

    } else if (statusLedBlink && countStatusLedOn == 1) {  //someone told me to blink, because i was already on
       digitalWrite(LED_STATUS,LOW);                     //so I'll turn off and turn back on later..

    } else if (statusLedBlink && countStatusLedOn > 1000) {//Now that I've waited long enough I'll finish my blink.
       statusLedBlink = false;                             //Turn off the issued blink
       digitalWrite(LED_STATUS,HIGH);                    //... and finally turn back on.
    }
  }
}

void sendByteToGameboy(byte send_byte) {
 for(int countLSDJTicks = 0; countLSDJTicks != 8; countLSDJTicks++) {  //we are going to send 8 bits, so do a loop 8 times
   if(send_byte & 0x80) {
       GB_SET(0,1,0);
       GB_SET(1,1,0);
   } else {
       GB_SET(0,0,0);
       GB_SET(1,0,0);
   }

   send_byte <<= 1;
 }
}

void updateBlinkLight(uint8_t light)
{
  if(blinkSwitch[light]) {
    blinkSwitchTime[light]++;
    if(blinkSwitchTime[light] == blinkMaxCount) {
      blinkSwitch[light]=0;
      blinkSwitchTime[light]=0;
      digitalWrite(pinLeds[light],LOW);
    }
  }
}

void updateBlinkLights()
{
  updateBlinkLight(0);
  updateBlinkLight(1);
  updateBlinkLight(2);
  updateBlinkLight(3);
  updateBlinkLight(4);
  updateBlinkLight(5);
}

void blinkLight(byte midiMessage, byte midiValue)
{
  if(midiValue) {
  switch(midiMessage) {
    case 0x90:
      if(!blinkSwitch[0]) digitalWrite(pinLeds[0],HIGH);
      blinkSwitch[0]=1;
      blinkSwitchTime[0]=0;
      break;
    case 0x95:
      if(!blinkSwitch[0]) digitalWrite(pinLeds[0],HIGH);
      blinkSwitch[0]=1;
      blinkSwitchTime[0]=0;
      break;
    case 0x9A:
      if(!blinkSwitch[0]) digitalWrite(pinLeds[0],HIGH);
      blinkSwitch[0]=1;
      blinkSwitchTime[0]=0;
      break;
    case 0x91:
      if(!blinkSwitch[1]) digitalWrite(pinLeds[1],HIGH);
      blinkSwitch[1]=1;
      blinkSwitchTime[1]=0;
      break;
    case 0x96:
      if(!blinkSwitch[1]) digitalWrite(pinLeds[1],HIGH);
      blinkSwitch[1]=1;
      blinkSwitchTime[1]=0;
      break;
    case 0x9B:
      if(!blinkSwitch[1]) digitalWrite(pinLeds[1],HIGH);
      blinkSwitch[1]=1;
      blinkSwitchTime[1]=0;
      break;
    case 0x92:
      if(!blinkSwitch[2]) digitalWrite(pinLeds[2],HIGH);
      blinkSwitch[2]=1;
      blinkSwitchTime[2]=0;
      break;
    case 0x97:
      if(!blinkSwitch[2]) digitalWrite(pinLeds[2],HIGH);
      blinkSwitch[2]=1;
      blinkSwitchTime[2]=0;
      break;
    case 0x9C:
      if(!blinkSwitch[2]) digitalWrite(pinLeds[2],HIGH);
      blinkSwitch[2]=1;
      blinkSwitchTime[2]=0;
      break;
    case 0x93:
      if(!blinkSwitch[3]) digitalWrite(pinLeds[3],HIGH);
      blinkSwitch[3]=1;
      blinkSwitchTime[3]=0;
      break;
    case 0x98:
      if(!blinkSwitch[3]) digitalWrite(pinLeds[3],HIGH);
      blinkSwitch[3]=1;
      blinkSwitchTime[3]=0;
      break;
    case 0x9D:
      if(!blinkSwitch[3]) digitalWrite(pinLeds[3],HIGH);
      blinkSwitch[3]=1;
      blinkSwitchTime[3]=0;
      break;
    case 0x94:
      if(!blinkSwitch[0])  digitalWrite(pinLeds[0],HIGH);
      blinkSwitch[0]=1;
      blinkSwitchTime[0]=0;
      if(!blinkSwitch[1]) digitalWrite(pinLeds[1],HIGH);
      blinkSwitch[1]=1;
      blinkSwitchTime[1]=0;
      if(!blinkSwitch[2]) digitalWrite(pinLeds[2],HIGH);
      blinkSwitch[2]=1;
      blinkSwitchTime[2]=0;
      break;
    case 0x99:
      if(!blinkSwitch[0])  digitalWrite(pinLeds[0],HIGH);
      blinkSwitch[0]=1;
      blinkSwitchTime[0]=0;
      if(!blinkSwitch[1]) digitalWrite(pinLeds[1],HIGH);
      blinkSwitch[1]=1;
      blinkSwitchTime[1]=0;
      if(!blinkSwitch[2]) digitalWrite(pinLeds[2],HIGH);
      blinkSwitch[2]=1;
      blinkSwitchTime[2]=0;
      break;
    case 0x9E:
      if(!blinkSwitch[0])  digitalWrite(pinLeds[0],HIGH);
      blinkSwitch[0]=1;
      blinkSwitchTime[0]=0;
      if(!blinkSwitch[1]) digitalWrite(pinLeds[1],HIGH);
      blinkSwitch[1]=1;
      blinkSwitchTime[1]=0;
      if(!blinkSwitch[2]) digitalWrite(pinLeds[2],HIGH);
      blinkSwitch[2]=1;
      blinkSwitchTime[2]=0;
      break;
  }
  }
  switch(midiMessage) {
    case 0xE0:
    case 0xE1:
    case 0xE2:
    case 0xE3:
    case 0xE4:
    case 0xB0:
    case 0xB1:
    case 0xB2:
    case 0xB3:
    case 0xB4:
      if(!blinkSwitch[5]) digitalWrite(LED_STATUS,HIGH);
      blinkSwitch[5]=1;
      blinkSwitchTime[5]=0;
      break;
    default:
      break;
  }
}

void statusLedOn()
{
  if(statusLedIsOn) {
    statusLedBlink = true;   //Make it blink even though its already on
  }
  statusLedIsOn  = true;     //This is the flag the updator function looks for to know if its ok to increment the timer and wait to turn off the led
  countStatusLedOn = 0;      //Reset the timer
  digitalWrite(LED_STATUS,HIGH); //Turn on the led
}

void noteOn(byte ch, byte pitch, byte velocity) {
  MIDI.sendNoteOn(pitch, velocity, ch);
  uint8_t s;
  s = 0x90 + (ch - 1);
  sendByteToGameboy(s);
  delayMicroseconds(GB_MIDI_DELAY);
  sendByteToGameboy(pitch);
  delayMicroseconds(GB_MIDI_DELAY);
  sendByteToGameboy(velocity);
  delayMicroseconds(GB_MIDI_DELAY);
  blinkLight(s, pitch);
  statusLedOn();
}

void noteOff(byte ch, byte pitch, byte velocity) {
  MIDI.sendNoteOff(pitch, velocity, ch);
  uint8_t s;
  s = 0x80 + (ch - 1);
  sendByteToGameboy(s);
  delayMicroseconds(GB_MIDI_DELAY);
  sendByteToGameboy(pitch);
  delayMicroseconds(GB_MIDI_DELAY);
  sendByteToGameboy(velocity);
  delayMicroseconds(GB_MIDI_DELAY);
  blinkLight(s, pitch);
  statusLedOn();
}

void modeMidiGbSetup() {
  digitalWrite(LED_STATUS, LOW);
  pinMode(PIN_GB_CLOCK, OUTPUT);
  digitalWrite(PIN_GB_CLOCK, HIGH);

  blinkMaxCount = 1000;
  MIDI.setHandleNoteOn(noteOn);
  MIDI.setHandleNoteOff(noteOff);
}

void OnBLEConnected() {
  digitalWrite(LED_BUILTIN, HIGH);
  modeMidiGbSetup();
}

void OnBLEDisconnected() {
  digitalWrite(LED_BUILTIN, LOW);
}

void setup() {
  MIDI.begin(MIDI_CHANNEL_OMNI);
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(LED_MGB, OUTPUT);
  pinMode(LED_NL, OUTPUT);
  pinMode(LED_KB, OUTPUT);
  pinMode(LED_LEAD, OUTPUT);
  pinMode(LED_SYNC, OUTPUT);
  pinMode(LED_STATUS, OUTPUT);

  pinMode(PIN_GB_CLOCK, OUTPUT);
  pinMode(PIN_GB_SERIAL_IN, INPUT);
  pinMode(PIN_GB_SERIAL_OUT, OUTPUT);

  BLEMIDI.setHandleConnected(OnBLEConnected);
  BLEMIDI.setHandleDisconnected(OnBLEDisconnected);
}

void loop() {
  MIDI.read();
  updateBlinkLights();
  updateStatusLed();
}
