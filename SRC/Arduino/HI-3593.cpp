//
// HI-3593.cpp: library for HI-3593 shield
// (HOLT-IC dual receiver - single transmitter ARINC 429 chip)
//
// VERSION: V2.0 (added "float on ARINC" coding/decoding)
//

#include "HI-3593.h"

#include <SPI.h>

// ARINC 429 shield pin numbers definition
#define SS     10  // SPI Slave Select
#define OE      9  // Output Enable of voltage translator (TXB0104)
#define MR      8  // HI-3593 Master Reset
#define TxEmpty 7  // HI-3593 Transmitter Empty
#define DIn1    6  // Discrete input #1 (0V / open)
#define DIn0    5  // Discrete input #0 (0V / open)
#define DOut    4  // Discrete output (0V / Open)
#define R1Int   3  // HI-3593 Receiver #1 interrupt
#define R2Int   2  // HI-3593 Receiver #2 interrupt
#define RTOut   1  // RS422 TX Out
#define RRIn    0  // RS422 RX In

//------------------------------------------------------------------------------
void HI3593ShieldInit() {

  // in/out pins configuration
  pinMode(SS,     OUTPUT); // SPI Slave Select
  pinMode(OE,     OUTPUT); // Output Enable of voltage translator (TXB0104)
  pinMode(MR,     OUTPUT); // HI-3593 Master Reset
  pinMode(TxEmpty, INPUT); // HI-3593 Transmitter Empty
  pinMode(DIn1,    INPUT); // Discrete input #1 (0V / open)
  pinMode(DIn0,    INPUT); // Discrete input #0 (0V / open)
  pinMode(DOut,   OUTPUT); // Discrete output (0V / Open)
  pinMode(R1Int,   INPUT); // HI-3593 Receiver #1 interrupt
  pinMode(R2Int,   INPUT); // HI-3593 Receiver #2 interrupt
  pinMode(RTOut,  OUTPUT); // RS422 TX Out
  pinMode(RRIn,    INPUT); // RS422 RX In
 
  // default states on outputs
  digitalWrite(SS,    HIGH);
  digitalWrite(OE,    HIGH);
  digitalWrite(RTOut, HIGH);

  // reset HI-3593
  digitalWrite(MR, HIGH); delay(1); digitalWrite(MR,  LOW);

  // initialize SPI
  SPI.begin();
  SPISettings HI3593SpiSettings(
    10000000, // page 1, features: "10MHz SPI" => nearest is 16MHz/2 = 8MHz
    MSBFIRST, // page 14, fig 5
    SPI_MODE0 // page 14, fig 5
  );
  SPI.beginTransaction(HI3593SpiSettings);

  // set the HI-3593 ACLK division register to accommodate 12MHz external clock
  digitalWrite(SS, LOW);
  SPI.transfer(0x38); // Set ACLK division register (HI-3593 data sheet, page 6)
  SPI.transfer(0x10); // page 9, table 2: 0x0C=12MHz clock; 0x10=16MHz clock
  digitalWrite(SS, HIGH);

  // set the HI-3593 TX control register  
  digitalWrite(SS, LOW);
  SPI.transfer(0x08); // transmit control register, (HI-3593 data sheet, page 4)
  SPI.transfer(0x24); // bit 5 (TMODE): send ARINC words without enable command (page 5)
                      // bit 2 (TPARITY): parity bit generated by the chip (page 5)
  digitalWrite(SS, HIGH);
}
//------------------------------------------------------------------------------
void SetRxCtrlReg(unsigned char RxChannel, unsigned char CtrlWord) {
  digitalWrite(SS, LOW);
  SPI.transfer((RxChannel==0)?0x10:0x24); // write rx1 or rx2 CTRL register (page 5)
  SPI.transfer(CtrlWord);
  digitalWrite(SS, HIGH);
}
//------------------------------------------------------------------------------
void SetLabelFilters(unsigned char RxChannel, const unsigned char* bitarray) {
  digitalWrite(SS, LOW);
  SPI.transfer((RxChannel==0)?0x14:0x28); // receiver #1 or #2 (page 4)
  for (char i=0; i<32; i++) SPI.transfer(bitarray[i]);
  digitalWrite(SS, HIGH);
}
//------------------------------------------------------------------------------
void WriteArincWord(unsigned long w) {
  digitalWrite(SS, LOW);
  SPI.transfer(0x0C); // write A429 word to TX FIFO (page 4)
  for (char i=3; i>=0; i--) SPI.transfer(((unsigned char*)&w)[i]);
  digitalWrite(SS, HIGH);
}
//------------------------------------------------------------------------------
bool RxFifoEmpty(unsigned char RxChannel) {
  digitalWrite(SS, LOW);
  SPI.transfer((RxChannel==0)?0x90:0xB0); // reads RX status register
  unsigned char rsr = SPI.transfer(0);
  digitalWrite(SS, HIGH);
  return (rsr & 1) == 1;
}
//------------------------------------------------------------------------------
unsigned long ReadArincWord(unsigned char RxChannel){
  digitalWrite(SS, LOW);
  SPI.transfer((RxChannel==0)?0xA0:0xC0); // read A429 word from receiver #1 or #2 FIFO (page 4)
  union {unsigned long w; unsigned char b[4];} aw;
  for (char i=3; i>=0; i--) aw.b[i] = SPI.transfer(0);
  digitalWrite(SS, HIGH);
  return aw.w;
}
//------------------------------------------------------------------------------
// execution time: 81µs (including 2µs for calling the function)
unsigned long BuildArincWord(const float &r, const float &d, const unsigned char ssm, const unsigned char sdi, const unsigned char label) {
  float ld = d/r * 262144; // 262144 = 2^18
  bool noSdi = (sdi>3);
  if (noSdi) ld *= 4;
  unsigned long w = ld>0?(long int)(ld+0.5):(long int)(ld-0.5);
  if (noSdi) w = (((unsigned long)ssm)<<29) | ((w<< 8) & 0x1fffffff)                            | label;
  else       w = (((unsigned long)ssm)<<29) | ((w<<10) & 0x1fffffff) | (((unsigned int)sdi)<<8) | label;
  return w;
}
//------------------------------------------------------------------------------
// execution time: ... µs (including ... µs for calling the function)
unsigned long BuildArincWordFloat(const float &data, const bool vld, const unsigned char label) {
  union {float x; unsigned long u;} un; // "unsigned int" => "unsigned long" // V2.01
  if (vld) {
    un.x = 1.000030f * data; // scale factor to balance the truncation error
    return ((un.u >> 1) & 0x7fffff00) | label;
  } else return 0x3fe00000 | label;  // NAN >> 1 (NAN = 0x7fc00000) // V2.01: encoded label
}
//------------------------------------------------------------------------------
// execution time: 22µs (including 2µs for calling the function)
void SplitArincWord(const unsigned long &aw, const bool noSdi, const float &range, float &data, unsigned char &ssm, unsigned char &sdi) {
  data = range/2.147483648e9 * ((signed long)(aw & (noSdi ? 0xffffff00 : 0xfffffc00)) << 3);
  ssm =            (((unsigned char*)&aw)[3] & 0x60) >> 5;
  sdi = noSdi ? 0 : ((unsigned char*)&aw)[1] & 0x03;
}
//------------------------------------------------------------------------------
// execution time: ... µs (including ... µs for calling the function)
void SplitArincWordFloat(const unsigned long &aw, float &data, bool &vld) {
  union {float x; unsigned long u;} un; // "unsigned int" => "unsigned long" // V2.01
  un.u = (aw & 0x7fffff00)<<1;
  vld = !isnan(un.x);  // replaced data by un.x // V2.01 
  data = vld ? un.x : 0.0f;
}
//------------------------------------------------------------------------------
unsigned char cbnSsm(const unsigned char Ssm1, const unsigned char Ssm2) {
  return ((Ssm1==3) && (Ssm2==3)) ? 3 : (((Ssm1==0) || (Ssm2==0)) ? 0 : 1);
}
//------------------------------------------------------------------------------
