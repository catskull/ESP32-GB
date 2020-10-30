#include <Arduino.h>
#include <BLEMIDI_Transport.h>
#include <hardware/BLEMIDI_ESP32_NimBLE.h>

BLEMIDI_CREATE_INSTANCE("GB MIDI", MIDI)

#define LED_BUILTIN 2

#define PIN_GB_CLOCK      25
#define PIN_GB_SERIAL_OUT 17
#define PIN_GB_SERIAL_IN  26

#define GB_MIDI_DELAY 500 //Microseconds to delay the sending of a byte to gb

#define GB_SET(bit_cl, bit_out, bit_in) digitalWrite(PIN_GB_CLOCK, bit_cl); digitalWrite(PIN_GB_SERIAL_OUT, bit_out); digitalWrite(PIN_GB_SERIAL_IN, bit_in);

byte incomingMidiByte;  //incomming midi message
byte readgbClockLine;
byte readGbSerialIn;
byte bit;
byte midiData[] = {0, 0, 0};
byte lastMidiData[] = {0, 0, 0};

int incomingMidiNote = 0;
int incomingMidiVel = 0;
byte readToggleMode;
byte serialWriteBuffer[256];
byte midiDefaultStartOffset;
int  writePosition=0;
int  readPosition=0;
int lastMode=0; //Stores the last selected mode for leds.

byte midiSyncByte;
byte midiSyncByteLast;

byte midiStatusType;

int countLSDJTicks = 0;            //for loop int (we need to cycle 8 pulses)
int countSyncTime  = 0;
int countSyncLightTime=0;
int countSyncSteps = 0;
int countSyncPulse = 0;
int countGbClockTicks =0;
int countClockPause =0;
int countIncommingMidiByte =0;
int countStatusLedOn =0;
unsigned int waitClock =0;

boolean sequencerStarted = false;        //Sequencer has Started
boolean midiSyncEffectsTime = false;
boolean midiNoteOnMode   =false;
boolean midiNoteOffMode  =false;
boolean midiProgramChange=false;
boolean midiAddressMode  =false;
boolean midiValueMode    =false;

int midiOutLastNote[4] = {-1,-1,-1,-1};

unsigned long midioutNoteTimer[4];
byte midioutNoteHold[4][4];
byte midioutNoteHoldCounter[4];
int midioutNoteTimerThreshold = 10;

int midioutBitDelay = 0;
int midioutByteDelay = 0;

boolean getIncommingSlaveByte()
{
  delayMicroseconds(midioutBitDelay);
  GB_SET(0,0,0);
  delayMicroseconds(midioutBitDelay);
  GB_SET(1,0,0);
  delayMicroseconds(2);
  if(digitalRead(PIN_GB_SERIAL_IN)) {
    incomingMidiByte = 0;
    for(countClockPause=0;countClockPause!=7;countClockPause++) {
      GB_SET(0,0,0);
      delayMicroseconds(2);
      GB_SET(1,0,0);
      incomingMidiByte = (incomingMidiByte << 1) + digitalRead(PIN_GB_SERIAL_IN);
    }
    return true;
  }
  return false;
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

void stopNote(byte m)
{
  for(int x=0;x<midioutNoteHoldCounter[m];x++) {
    midiData[0] = (0x80 + m);
    midiData[1] = midioutNoteHold[m][x];
    midiData[2] = 0x00;
    MIDI.sendNoteOff(midioutNoteHold[m][x], 0, m+1);
  }
  midiOutLastNote[m] = -1;
  midioutNoteHoldCounter[m] = 0;
}

void checkStopNote(byte m)
{
  if((midioutNoteTimer[m]+midioutNoteTimerThreshold) < millis()) {
    stopNote(m);
  }
}

void stopAllNotes()
{
  for(int m=0;m<4;m++) {
    if(midiOutLastNote[m]>=0) {
      stopNote(m);
    }
    midiData[0] = (0xB0 + m);
    midiData[1] = 123;
    midiData[2] = 0x7F;
    MIDI.sendControlChange(123, 127, m+1);
  }
}

void playNote(byte m, byte n)
{
  midiData[0] = (0x90 + m);
  midiData[1] = n;
  midiData[2] = 0x7F;
  MIDI.sendNoteOn(n, 127, m+1);

  midioutNoteHold[m][midioutNoteHoldCounter[m]] =n;
  midioutNoteHoldCounter[m]++;
  midioutNoteTimer[m] = millis();
  midiOutLastNote[m] =n;
}

void playCC(byte m, byte n)
{
  byte v = n;

  if(m) {
    if(m) {
      v = (v & 0x0F)*8;
      //if(v) v --;
    }
    n=(m*7)+((n>>4) & 0x07);
    midiData[0] = (0xB0 + m);
    midiData[1] = n;
    midiData[2] = v;
    MIDI.sendControlChange(n, v, m+1);
  } else {
    if(m) {
      float s;
      s = n;
      v = ((s / 0x6f) * 0x7f);
    }
    n=(m*7);
    midiData[0] = (0xB0 + m);
    midiData[1] = m;
    midiData[2] = v;
    MIDI.sendControlChange(n, v, m+1);
  }
}

void playPC(byte m, byte n)
{
  midiData[0] = (0xC0 + m);
  midiData[1] = n;
  MIDI.sendProgramChange(n, m+1);
}

void midioutDoAction(byte m, byte v)
{
  if(m < 4) {
    //note message
    if(v) {
      checkStopNote(m);
      playNote(m,v);
    } else if (midiOutLastNote[m]>=0) {
      stopNote(m);
    }
  } else if (m < 8) {
    m-=4;
    //cc message
    playCC(m,v);
  } else if(m < 0x0C) {
    m-=8;
    playPC(m,v);
  }
}

void modeLSDJMidioutSetup()
{
  pinMode(PIN_GB_CLOCK,OUTPUT);
  digitalWrite(PIN_GB_CLOCK,HIGH);

  countGbClockTicks=0;
  lastMidiData[0] = -1;
  lastMidiData[1] = -1;
  midiValueMode = false;
}

void OnBLEConnected() {
  digitalWrite(LED_BUILTIN, HIGH);
  modeLSDJMidioutSetup();
}

void OnBLEDisconnected() {
  digitalWrite(LED_BUILTIN, LOW);
}

void setup() {
  MIDI.begin(MIDI_CHANNEL_OMNI);
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);

  // pinMode(LED_MGB, OUTPUT);
  // pinMode(LED_NL, OUTPUT);
  // pinMode(LED_KB, OUTPUT);
  // pinMode(LED_LEAD, OUTPUT);
  // pinMode(LED_SYNC, OUTPUT);
  // pinMode(LED_STATUS, OUTPUT);

  pinMode(PIN_GB_CLOCK, OUTPUT);
  pinMode(PIN_GB_SERIAL_IN, INPUT);
  pinMode(PIN_GB_SERIAL_OUT, OUTPUT);

  BLEMIDI.setHandleConnected(OnBLEConnected);
  BLEMIDI.setHandleDisconnected(OnBLEDisconnected);
}

void loop() {
     if(getIncommingSlaveByte()) {
        if(incomingMidiByte > 0x6f) {
          switch(incomingMidiByte)
          {
           case 0x7F: //clock tick
            MIDI.sendClock();
            break;
           case 0x7E: //seq stop
             MIDI.sendStop();
             stopAllNotes();
             break;
           case 0x7D: //seq start
             MIDI.sendStart();
             break;
           default:
             midiData[0] = (incomingMidiByte - 0x70);
             midiValueMode = true;
             break;
          }
        } else if (midiValueMode == true) {
          midiValueMode = false;
          midioutDoAction(midiData[0],incomingMidiByte);
        }

      } else {
        MIDI.read();
      }
}
