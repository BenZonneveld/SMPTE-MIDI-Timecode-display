// Code from forum post Dec 12, 2007

// include the library code:
#include "I2CBackpack.h"

#if defined(USBCON)
#include <MIDIUSB.h>
#include <MIDIUSB_Defs.h>
#endif

//#define DEBUG

Adafruit_7segment lsb_matrix = Adafruit_7segment();
Adafruit_7segment msb_matrix = Adafruit_7segment();

char lsb_pos2 = 0;
#define BOOT_DELAY 333
long int keepAlive = millis();
boolean bKeepAlive = false;

#define icpPin 13 // ICP input pin on arduino
#define one_time_max 675 // these values are setup for NTSC video
#define one_time_min 325 // PAL would be around 1000 for 0 and 500 for 1
#define zero_time_max 1080 // 80bits times 29.97 frames per sec
#define zero_time_min 750 // equals 833 (divide by 8 clock pulses)

const word sync = 0xBFFC; // Sync word to expect when running tape forward

volatile unsigned int bit_time;
volatile uint8_t tcFlags = 0;

uint8_t tc[10] = {0};          // ISR Buffer to store incoming bits
volatile uint8_t xtc[8] = {0}; // Buffer to store valid TC data - sync bytes
volatile uint8_t mtc[8] = {0}; // Buffer to store valid TC date - from midi
volatile uint8_t raw_mtc[4]= {0};

#ifdef DEBUG
volatile unsigned int avg_zero_time;
volatile unsigned int avg_one_time;
#endif

enum flagBits {
  tcValid,        // TC copied to xtc is valid (Used by main loop to determing if a timecode has arrived)
  tcFrameError,   // ISR edge out of timing bounds (only gets reset after next valid TC read)
  tcOverrun,      // TC was not read from xtc by main loop before next value was ready (so main loop can tell if timecodes have been lost)
  tcForceUpdate,  // Valid TC will always be copied to buffer even if last value not read (if set by main code then TC will always be copied)
  tcHalfOne       // ISR is reading a 1 bit so ignore next edge (Internal to ISR)
};

#ifdef DEBUG
void printMtc()
{
  
    for (int i=0 ; i < 8 ; i++)
    {
      Serial.print(mtc[i]);
      if ( i%2 == 1 ) Serial.print(":");
    }
    Serial.println();
}
#endif

void KeepAlive()
{
  if ( (millis() - keepAlive ) >= 500 )
  {
    keepAlive = millis();
    if ( !bKeepAlive )
    {
      lsb_pos2 = (lsb_pos2 & 0x11 ) | 0x2 | 0x4;
      lsb_matrix.writeDigitRaw(2, lsb_pos2);
    } else {
      lsb_pos2 = (lsb_pos2 & 0x11 ) | 0x2 | 0x8;       
      lsb_matrix.writeDigitRaw(2, lsb_pos2);      
    }
    lsb_matrix.writeDisplay();
    bKeepAlive = !bKeepAlive;
  }
}

void TimeCodeMode(char unsigned mode)
{
#ifdef DEBUG
  Serial.print("Mode: ");
  Serial.println(mode);
#endif

  switch (mode)
  {
    case 0: // 24 fps
            lsb_pos2 = lsb_pos2 & 0x0F;
            break;
    case 1: // 25 fps
            lsb_pos2 = lsb_pos2 & 0x0F;
            break;
    case 2: // 30 fps (Drop Frame )
            lsb_pos2 = lsb_pos2 | 0x10;
            break;
    case 3: // 30 fps
            lsb_pos2 = lsb_pos2 & 0x0F;
            break;
    default:
            lsb_pos2 = lsb_pos2 & 0x0F;
            break;
  }
}

/* ICR interrupt vector */
ISR(TIMER3_CAPT_vect) {
  //ISR=Interrupt Service Routine, and timer1 capture event
  //toggleCaptureEdge
  TCCR3B ^= _BV(ICES3);
  //toggles the edge that triggers the handler so that the duration of both high and low pulses is measured.
  bit_time = ICR3; //this is the value the timer generates
  //resetTimer1
  TCNT3 = 0;

  // Handle out of range timings as Frame Error
  if ((bit_time < one_time_min) or (bit_time > zero_time_max)) { // Drop out now if edge time not withing bounds
    bitSet(tcFlags, tcFrameError);
    clearBuffer(tc, sizeof(tc));
    return;
  }

  // 
  if ( bit_time > one_time_max ) // A zero bit arrived
  {
    if (bitRead(tcFlags, tcHalfOne) == 1){   // But we are expecting a 1 edge
      bitClear(tcFlags, tcHalfOne);
      clearBuffer(tc, sizeof(tc));
      return;
    }

#ifdef DEBUG
    avg_zero_time = (avg_zero_time + bit_time) / 2;
#endif

    // 0 bit
    shiftRight(tc, sizeof(tc));              // Rotate buffer right
    // Shift replaces top bit with zero so nothing else to do
    //bitClear(tc[0], 7);                                   // Reset the 1 bit in the buffer
  }
    else                                                    // Not zero so must be a 1 bit
  { // 1 bit
    if (bitRead(tcFlags, tcHalfOne) == 0){                // First edge of a 1 bit
      bitSet(tcFlags, tcHalfOne);                         // Flag we have the first half
      return;
    }
    // Second edge of a 1 bit
    bitClear(tcFlags, tcHalfOne);                         // Clear half 1 flag
    shiftRight(tc, sizeof(tc));                           // Rotate buffer right
    bitSet(tc[0], 7);                                     // Set the 1 bit in the buffer

#ifdef DEBUG
    avg_one_time = ( avg_one_time + bit_time ) /2;
#endif

  }

  // Congratulations, we have managed to read a valid 0 or 1 bit into buffer
  if (word(tc[0], tc[1]) == sync){                        // Last 2 bytes read = sync?
    bitClear(tcFlags, tcFrameError);                      // Clear framing error
    bitClear(tcFlags, tcOverrun);                         // Clear overrun error
    if (bitRead(tcFlags, tcForceUpdate) == 1){
      bitClear(tcFlags, tcValid);                         // Signal last TC read
    }
    if (bitRead(tcFlags, tcValid) == 1){                  // Last TC not read
      bitSet(tcFlags, tcOverrun);                         // Flag overrun error
      return;                                             // Do nothing else
    }
    for (uint8_t x = 0; x < sizeof(xtc); x++){            // Copy buffer without sync word
      xtc[x] = tc[x + 2];
    }
    bitSet(tcFlags, tcValid);                             // Signal valid TC
  }
}

void displayTest(){
  msb_matrix.writeDigitNum(4,8);
  lsb_matrix.writeDigitNum(0,8);
  msb_matrix.writeDisplay();
  lsb_matrix.writeDisplay();
  delay(BOOT_DELAY);
  msb_matrix.clear();
  lsb_matrix.clear();
  msb_matrix.writeDigitNum(3,8);
  lsb_matrix.writeDigitNum(1,8);
  lsb_matrix.writeDisplay();
  msb_matrix.writeDisplay();
  delay(BOOT_DELAY);
  lsb_matrix.clear();
  msb_matrix.clear();
  msb_matrix.writeDigitNum(1,8);
  lsb_matrix.writeDigitNum(3,8);
  lsb_matrix.writeDisplay();
  msb_matrix.writeDisplay();
  delay(BOOT_DELAY);
  msb_matrix.clear();
  lsb_matrix.clear();
  msb_matrix.writeDigitNum(0,8);
  lsb_matrix.writeDigitNum(4,8);
  lsb_matrix.writeDisplay();
  msb_matrix.writeDisplay();
  delay(BOOT_DELAY);
  lsb_matrix.writeDigitNum(4,0);
  msb_matrix.writeDigitNum(0,0);
  msb_matrix.writeDisplay();
  lsb_matrix.writeDisplay();
  delay(BOOT_DELAY/2);
  lsb_matrix.writeDigitNum(3,0);
  msb_matrix.writeDigitNum(1,0);
  msb_matrix.writeDisplay();
  lsb_matrix.writeDisplay();
  delay(BOOT_DELAY/2);
  lsb_matrix.writeDigitNum(1,0);
  msb_matrix.writeDigitNum(3,0);
  msb_matrix.writeDisplay();
  lsb_matrix.writeDisplay();
  delay(BOOT_DELAY/2);
  lsb_matrix.writeDigitNum(0,0);
  msb_matrix.writeDigitNum(4,0);
  msb_matrix.writeDisplay();
  lsb_matrix.writeDisplay();
}

void setup() {
#ifdef DEBUG
  Serial.begin(115200);
#endif
  lsb_matrix.begin(0x70);
  msb_matrix.begin(0x71);

  lsb_matrix.clear();
  msb_matrix.clear();
  lsb_matrix.writeDisplay();
  msb_matrix.writeDisplay();

  pinMode(icpPin, INPUT); // ICP pin (digital pin 8 on arduino) as input
  pinMode(2, INPUT);
  bit_time = 0;

#ifdef DEBUG
  avg_zero_time = (zero_time_min + zero_time_max ) /2;
  avg_one_time = (one_time_min + one_time_max) /2;
#endif

  displayTest();

  lsb_matrix.setBrightness(5);
  msb_matrix.setBrightness(5);
  
  msb_matrix.writeDigitRaw( 2 ,0x02 | 0x04 | 0x08);
  lsb_pos2=0x02;
  lsb_matrix.writeDigitRaw(2, lsb_pos2);

  lsb_matrix.writeDisplay();
  msb_matrix.writeDisplay();

  keepAlive = millis();
  
  TCCR3A = B00000000; // clear all
  TCCR3B = B11000010; // ICNC3 noise reduction + ICES3 start on rising edge + CS11 divide by 8
  TCCR3C = B00000000; // clear all
  TIMSK3 = B00100000; // ICIE3 enable the icp
  TCNT3 = 0; // clear timer1

}

#if defined(USBCON)
void MidiTimeCode(midiEventPacket_t rx) {
  byte qframe;
  char unsigned mode = 0;
  byte old_digit;
  qframe = ( rx.byte2 & 0xF0) >> 4;

#ifdef DEBUG
    Serial.print("MTC-QF: ");
//    Serial.print(qframe, HEX);
//    Serial.print(" ");
#endif
//  printMtc();
  switch (qframe)
  {
    case 0: // Frames Low Nibble
            raw_mtc[3] = (raw_mtc[3] & 0xF0) | (rx.byte2 & 0xF);
            break;
    case 1: // Frames High Nibble 
            raw_mtc[3] = (raw_mtc[3] & 0x0F ) | ((rx.byte2 & 0xF) << 4);
            old_digit=mtc[7]; 
            mtc[7] = raw_mtc[3] % 10;
            if ( old_digit != mtc[7] )
            {
              lsb_matrix.writeDigitNum(4,mtc[7]);
              lsb_matrix.writeDisplay();
            }
            old_digit = mtc[6];
            mtc[6] = ( raw_mtc[3]/10 ) % 10;
            if ( old_digit != mtc[6] )
            {
              lsb_matrix.writeDigitNum(3,mtc[6]);
              lsb_matrix.writeDisplay();
            }
#ifdef DEBUG
            Serial.print("f: ");
            printMtc();
#endif
            break;
    case 2: // Seconds Low Nibble
            raw_mtc[2] = (raw_mtc[2] & 0xF0 ) | (rx.byte2 & 0xF);
            break;
    case 3: // Seconds High Nibble
            raw_mtc[2] = (raw_mtc[2] & 0x0F ) | ((rx.byte2 & 0xF) << 4);
            old_digit = mtc[5];
            mtc[5] = raw_mtc[2] % 10;
            if ( old_digit != mtc[5] )
            {
              lsb_matrix.writeDigitNum(1,mtc[5]);
              lsb_matrix.writeDisplay();
            }
            old_digit = mtc[4];  
            mtc[4] = ( raw_mtc[2] / 10 ) % 10;
            if ( old_digit != mtc[4] )
            {
              lsb_matrix.writeDigitNum(0,mtc[4]);
              lsb_matrix.writeDisplay();
            }
#ifdef DEBUG
            Serial.print("s: ");
            printMtc();
#endif
            break;
    case 4: // Minutes Low Nibble
            raw_mtc[1] = (raw_mtc[1] & 0xF0 ) | (rx.byte2 & 0xF);
            break;
    case 5: // Minutes High Nibble
            raw_mtc[1] = (raw_mtc[1] & 0x0F ) | ((rx.byte2 & 0xF) << 4 );
            old_digit = mtc[3];
            mtc[3] = raw_mtc[1] % 10;
            if ( old_digit != mtc[3] )
            {
              msb_matrix.writeDigitNum(4,mtc[3]);
              msb_matrix.writeDisplay();
            }
            old_digit = mtc[2];  
            mtc[2] = ( raw_mtc[1] / 10 ) % 10;
            if ( old_digit != mtc[2] )
            {
              msb_matrix.writeDigitNum(3,mtc[2]);
              msb_matrix.writeDisplay();
            }
#ifdef DEBUG
            Serial.print("m: ");
            printMtc();
#endif
            break;
    case 6: // Hours Low Nibble
            raw_mtc[0] = (raw_mtc[0] & 0xF0 ) | (rx.byte2 & 0xF);
            break;
    case 7: // Hour High Nibble
            raw_mtc[0] = (raw_mtc[0] & 0x0F ) | ((rx.byte2 & 0xF) << 4);
            old_digit = mtc[1];
            mtc[1] = ( raw_mtc[0] & 0x1F )% 10;
            if ( old_digit != mtc[1] )
            {
              msb_matrix.writeDigitNum(1,mtc[1]);
              msb_matrix.writeDisplay();
            }
            old_digit = mtc[0]; 
            mtc[0] = ( (raw_mtc[0] & 0x1F)/ 10 ) % 10;
            if ( old_digit != mtc[0] )
            {
              msb_matrix.writeDigitNum(0,mtc[0]);
              msb_matrix.writeDisplay();
            }
//#ifdef DEBUG
//            Serial.print("h: ");
//            printMtc();
//#endif
            mode = (raw_mtc[0] & 0xE0 ) >> 5;
            TimeCodeMode(mode);
            break;
    default:
      // if nothing else matches, do the default
      // default is optional
              break;
  } 
}

void FullFrame(midiEventPacket_t rx) {
  byte fframe[10]= {0};
  char unsigned mode = 0;
  fframe[0]=rx.byte1;
  fframe[1]=rx.byte2;
  fframe[2]=rx.byte3;
  int i=3;
  do {
    rx = MidiUSB.read();
    fframe[i]=rx.byte1;
    i++;
    fframe[i]=rx.byte2;
    i++;
    fframe[i]=rx.byte3;
    i++;
  } while (rx.header != 0);

  if ( fframe[0] == 0xF0 
        && fframe[1] == 0x7F 
        && fframe[2] == 0x7F 
        && fframe[3] == 0x01 
        && fframe[4] == 0x01 
        && fframe[9] == 0xF7) // Full Frame Message
  {
    raw_mtc[3] = fframe[8];
    raw_mtc[2] = fframe[7];
    raw_mtc[1] = fframe[6];
    raw_mtc[0] = fframe[5] & 0x1F;
    // Frames
    mtc[7] = raw_mtc[3] % 10;
    mtc[6] = ( raw_mtc[3]/10 ) % 10; 
    // Seconds
    mtc[5] = raw_mtc[2] % 10;
    mtc[4] = ( raw_mtc[2] / 10 ) % 10;
    // Minutes
    mtc[3] = raw_mtc[1] % 10;
    mtc[2] = ( raw_mtc[1] / 10 ) % 10;
    // Hours
    mtc[1] = raw_mtc[0] % 10;
    mtc[0] = ( raw_mtc[0] / 10 ) % 10;
    msb_matrix.writeDigitNum(0, mtc[0]);
    msb_matrix.writeDigitNum(1, mtc[1]);
    // Minutes
    msb_matrix.writeDigitNum(3, mtc[2]);
    msb_matrix.writeDigitNum(4, mtc[3]);
    // Seconds
    lsb_matrix.writeDigitNum(0, mtc[4]); 
    lsb_matrix.writeDigitNum(1, mtc[5]);
    // Frames
    lsb_matrix.writeDigitNum(3, mtc[6]);
    lsb_matrix.writeDigitNum(4, mtc[7]);

    mode = (fframe[5] & 0xE0 ) >> 5;
    TimeCodeMode(mode);

    msb_matrix.writeDigitRaw(2,0x02 | 0x04 | 0x08);
    lsb_matrix.writeDigitRaw(2, lsb_pos2);

    msb_matrix.writeDisplay();
    lsb_matrix.writeDisplay();
#ifdef DEBUG
    Serial.print("MTC-FF: ");
    printMtc();
#endif
    return;
  }
}
#endif

void loop() {
#if defined(USBCON)
  midiEventPacket_t rx;
  do {
    rx = MidiUSB.read();
    if (rx.header != 0) {
      if ( rx.byte1 == 0xF1 && rx.byte3 == 0x7F ) { // We have a MidiTimeCode Event
        MidiTimeCode(rx);
      }
      if ( rx.byte1 == 0xF0 && rx.byte2 == 0x7F && rx.byte3 == 0x7F ) { // MTC Full frame message ?
        FullFrame(rx);
      }
    }
  } while (rx.header != 0);
#endif

  KeepAlive();
  
  if (bitRead(tcFlags, tcValid) == 1) {    
    if (bitRead(xtc[6],2) == 1 ) // Test for Drop Frame
    {
#ifdef DFDEBUG
      Serial.print("TC-[df] ");
#endif
      lsb_pos2 = lsb_pos2 | 0x10;
    }       
    else
    {
#ifdef DFDEBUG
      Serial.print("TC-NO DROP FRAME ");
#endif
      lsb_pos2 = lsb_pos2 & 0xF;
    }
    // Hours
    msb_matrix.writeDigitNum(0,xtc[0] & 0x03 );
    msb_matrix.writeDigitNum(1,xtc[1] & 0x0F );
    // Minutes
    msb_matrix.writeDigitNum(3,xtc[2] & 0x07 );
    msb_matrix.writeDigitNum(4,xtc[3] & 0x0F );
    // Seconds
    lsb_matrix.writeDigitNum(0,xtc[4] & 0x07);
    lsb_matrix.writeDigitNum(1,xtc[5] & 0x0F);
    //Frames
    lsb_matrix.writeDigitNum(3,xtc[6] & 0x03);
    lsb_matrix.writeDigitNum(4,xtc[7] & 0x0F);

#ifdef  DEBUG
    xtc[0] = xtc[0] & 0x03;
    xtc[1] = xtc[1] & 0x0f;
    xtc[2] = xtc[2] & 0x07;
    xtc[3] = xtc[3] & 0x0f;
    xtc[4] = xtc[4] & 0x07;
    xtc[5] = xtc[5] & 0x0f;
    xtc[6] = xtc[6] & 0x03;
    xtc[7] = xtc[7] & 0x0f;
    Serial.print("SMPTE: "); 
    for (int i=0 ; i < 8 ; i++)
    {
      Serial.print(xtc[i]);
      if ( i%2 == 1 ) Serial.print(":");
    }
    Serial.println();
#endif

    msb_matrix.writeDigitRaw(2,0x02 | 0x04 | 0x08);
    lsb_matrix.writeDigitRaw(2, lsb_pos2);
    
    msb_matrix.writeDisplay();
    lsb_matrix.writeDisplay();

#ifdef DEBUG_TIMING
    Serial.print("Zero Time ");
    Serial.println(avg_zero_time);
    Serial.print("One Time ");
    Serial.println(avg_one_time);
#endif

    bitClear(tcFlags, tcValid);
  }
}

void shiftRight(uint8_t theArray[], uint8_t theArraySize){
  uint8_t x;
  for (x = theArraySize; x > 0; x--){
    uint8_t xBit = bitRead(theArray[x - 1], 0);
    theArray[x] = theArray[x] >> 1;
    theArray[x] = theArray[x] | (xBit << 7);
  }
  theArray[x] = theArray[x] >> 1;
}

void clearBuffer(uint8_t theArray[], uint8_t theArraySize){
  for (uint8_t x = 0; x < theArraySize - 1; x++){
    theArray[x] = 0;
  }
}
