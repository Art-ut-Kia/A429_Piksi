/**
 * FILE NAME:                                   galileo.ino
 * 
 * DESCRIPTION: Bridges a Swift Navigation piksi-multi GNSS receiver module 
 *              to a Naveol Nav429 ARINC shield
 *
 * NOTA: Piksi receiver is supposed to operate as a Galileo-only receiver (exo-GPS velocimeter 
 *       to be used for GPS monitoring). Unfortunately, it is impossible to disable GPS from Piksi 
 *       solution. As a workaround, it is configured in full multiconstellation (GPS+GLONAS+GALILEO+BEIDOU), 
 *       so that GPS weight is lowered. Furthermore, SBAS is disabled to remove a common mode failure 
 *       between constellations
 *
 * AUTHOR/YYYYMMDD: JP Pétillon 2021/03/19
 * 
 * VERSION: V2.0 (added velocity covariance matrix + GPS SV's count on ARINC frame: 7 labels)        // V2.0
 */

//#define traceTimstamps   // uncomment to print a sequence of messages timestamps
//#define sendTestPattern  // uncomment to send a test pattern on A429 line rather than actual GNSS data
  #define synchronize      // comment to deactivate jitter smoother

  #ifdef traceTimstamps
      #define MAXSAMPLES 180 // number of messages to print in here above test
  #endif

#include <HI-3593.h> // ARINC Shield support library

#define NRESET A5 // output pin connected to piksi "negative reset" input
#define POSVLD A4 // input pin connected to piksi "position valid" output
#define PVLOUT A0 // output pin for debug
//..............................................................................
void setup() {
    // initialize serial link at piksi default baudrate
    Serial.begin(115200);
    // initializes ARINC 429 shield
    HI3593ShieldInit();
    Serial.println(" ------------------------------------------------------ ");
    Serial.println("|                      A429-PIKSI                      |");
    Serial.println("|  Bridges a Swift-nav PIKSI-MULTI to a Naveol NAV429  |");
    Serial.println("|  Author: jean-paul.petillon@orange.fr                |");
    Serial.println("|  Version: V2.0 dated 2021/03/19                      |");        // V2.0
    Serial.println(" ------------------------------------------------------ ");
    #ifdef traceTimstamps
        Serial.println("NOT THE FLIGHT SW. PRINTS A SEQUENCE OF PIKSI TIMESTAMPS");
    #endif
    #ifdef sendTestPattern
        Serial.println("NOT THE FLIGHT SW. SENDS A TEST PATTERN ON A429 LINE INSTEAD OF PIKSI DATA");
    #endif
    #ifndef synchronize
        Serial.println("JITTER COMPENSATION NOT ACTIVATED");
    #endif
    // release piksi reset pin
    pinMode(NRESET, OUTPUT); digitalWrite(NRESET, HIGH);
    pinMode(POSVLD, INPUT);
    pinMode(PVLOUT, OUTPUT);
    #ifdef traceTimstamps
      Serial.println("Waiting for First Fix...");
      delay(3000); // wait for actual POSVLD release after reset
      while (digitalRead(POSVLD)==LOW);
      Serial.println("First Fix obtained.");
    #endif
}

//..............................................................................
// global (hence remanant) variables
bool preambleRxd = false; // preamble allready received
uint32_t rxTime; // receive timestamp
uint32_t avgRxTime; // averaged rx timestamp
int32_t jitter; // deviation wrt predicted timestamp
struct __attribute__((packed)) {
    uint8_t preamble;
    uint16_t type;
    uint16_t sender;
    uint8_t len;
    uint8_t data[256];
} msg;
int iter = 0;
extern const unsigned char flip[]; // reversed bits labels n° (assigned at the end of this file)
struct __attribute__((packed)) {
    uint8_t  flags;
    uint32_t towMillis;
    uint16_t year;
    uint8_t  month, day, hour, minute, second;
    uint32_t nanos;
} utcTime;
struct __attribute__((packed)) {
    uint32_t towMillis;
    uint16_t gDop, pDop, tDop, hDop, vDop;
    uint8_t  statusFlags;
} dops;
struct __attribute__((packed)) {
    uint32_t towMillis;
    int64_t  lat, lon, height;
    float    covNN, covNE, covND, covEE, covED, covDD;
    uint8_t  svCount, statusFlags;
} llhPosCov;
struct __attribute__((packed)) {
    uint32_t towMillis;
    int32_t  vn, ve, vd;
    float    covNN, covNE, covND, covEE, covED, covDD;
    uint8_t  svCount, statusFlags;
} velNedCov;
struct __attribute__((packed)) {
    uint16_t devVin, cpuVint, cpuVaux, cpuTemp, feTemp;
} devMon;
bool devMonRefreshed = false;
//..............................................................................
// checks the CRC of the message
bool crcGood() {
    return true;
}

#ifdef traceTimstamps
uint32_t timest[MAXSAMPLES];
uint16_t msgtyp[MAXSAMPLES];
uint8_t splcnt = 0;
#endif

//..............................................................................
// cyclic function
void loop() {
    if (Serial.available()) { // something is received
        ((uint8_t*)&msg)[iter++] = Serial.read(); // stores received charecter
        // message decoding state-machine
        if (!preambleRxd) {
            if (msg.preamble==0x55) preambleRxd = true;
            else iter = 0;
        } else if (iter == msg.len+8) {
            #ifdef traceTimstamps
                if (splcnt<MAXSAMPLES) {
                    timest[splcnt] = micros();
                    msgtyp[splcnt] = msg.type;
                    splcnt++;
                } else {
                    Serial.print(MAXSAMPLES); Serial.println(" samples acquired.");
                    Serial.println("timeSTMP MsgID");
                    for (int i=0; i<MAXSAMPLES; i++) {
                        Serial.print(timest[i]);
                        Serial.print(" ");
                        Serial.println(msgtyp[i]);
                    }
                    while (1); // waits forever
                }
            #else
                if (crcGood()) switch (msg.type) { // processes received message
                    // NOTES:
                    // - Observed emission order is: 118,259,520,529,530
                    // - msg #181 is transmitted 1/40 frame, 32ms before msg #259 (may be at start of computations?)
                    case 181: {memcpy(&devMon,    &msg.data, sizeof(devMon)); devMonRefreshed = true; break;} // Device temperature and voltage levels
                    case 259: {memcpy(&utcTime,   &msg.data, sizeof(utcTime));   break;} // UTC Time
                    case 520: {memcpy(&dops,      &msg.data, sizeof(dops));      break;} // Dilution of Precision
                    case 529: {memcpy(&llhPosCov, &msg.data, sizeof(llhPosCov)); break;} // Geodetic 3D Position with its covariance
                    case 530: {memcpy(&velNedCov, &msg.data, sizeof(velNedCov));         // 3D NED Velocity with its covariance
                        // NED Vel + Cov packet is the latest of UART frame from PIKSI => generate ARINC frame
                        #ifdef synchronize
                            rxTime = micros(); // reads actual reception timestamp
                            avgRxTime += 100000; // average rx timestamp prediction
                            jitter = rxTime - avgRxTime;
                            if (abs(jitter)>50000) avgRxTime = rxTime;
                            else avgRxTime += jitter/100; // correction (time constant = 100 periods = 10s)
                        #endif
                        txArincFrame();
                        break;
                    }
                }
            #endif
            // gets ready for next message
            preambleRxd = msg.preamble = iter = 0;
        }
    }
}

//..............................................................................
// ARINC frame transmission (declared inline since it is called only one time)
inline void txArincFrame() {
    // removes jitter
    #ifdef synchronize
        myDelayUS(constrain(20000L - jitter, 0, 40000));
    #endif

    // overwrites GNSS data for testing purpose
    #define ft 0.3048
    #define knot (1852./3600.*1000.) // knot in mm/s
    #define kt2 (knot*knot) // knot² in (mm/s)²                                                     // V2.0
    #define ftPerMin (0.3048/60*1000.) //ft/min in mm/s
    #ifdef sendTestPattern
        velNedCov.vn = 10*knot; velNedCov.ve = -7*knot; velNedCov.vd = 3*ftPerMin;
        velNedCov.covNN =  4.0*kt2; velNedCov.covEE =  9.0*kt2; velNedCov.covDD =  16.0*kt2;        // V2.0
        velNedCov.covNE = -4.0*kt2; velNedCov.covND = -9.0*kt2; velNedCov.covED = -16.0*kt2;        // V2.0
        llhPosCov.lat=0x4046000000000000; // 44.0°
        llhPosCov.lon=0x400CCCCCCCCCCCCD; //  3.6°
        llhPosCov.height=0x4056DC28F5C28F5C; // 300*ft
        utcTime.hour=15; utcTime.minute=33; utcTime.second=04; // 15:22:04
        utcTime.nanos=724362002; //0.724 seconds = 777777777*2^-30 seconds
        utcTime.day=30; utcTime.month=9; utcTime.year=1957; // my birthdate :o)
        dops.hDop=5555, dops.vDop=6666; // 55.55, 66.66
        velNedCov.svCount = 219;
        velNedCov.statusFlags = 17;
        llhPosCov.statusFlags = 13;
        devMon.cpuTemp=3456; devMon.feTemp=6543; // should display 35°C and 65°C (rounded to nearest integer Celsius degree)
    #endif

    bool vld = velNedCov.statusFlags & 0x07; // validité globale
    #define SDI 0b11

    // North, East, Up velocity components
    #define hvelRange (512.*knot)
    #define vvelRange (32768.*ftPerMin)
    WriteArincWord(BuildArincWord(hvelRange, velNedCov.vn, vld?3:1, SDI, flip[0104]));
    WriteArincWord(BuildArincWord(hvelRange, velNedCov.ve, vld?3:1, SDI, flip[0106]));
    WriteArincWord(BuildArincWord(vvelRange,-velNedCov.vd, vld?3:1, SDI, flip[0107])); // PIKSI outputs Vdown, ARINC Vup

    // NED velocity noise covariance matrix                                       // V2.0
    // transmitted as a truncated float as get from Piksi, i.e. in (m/s)^2        // V2.0
    WriteArincWord(BuildArincWordFloat(velNedCov.covNN, vld, flip[0114]));        // V2.0
    WriteArincWord(BuildArincWordFloat(velNedCov.covNE, vld, flip[0116]));        // V2.0
    WriteArincWord(BuildArincWordFloat(velNedCov.covND, vld, flip[0117]));        // V2.0
    WriteArincWord(BuildArincWordFloat(velNedCov.covEE, vld, flip[0124]));        // V2.0
    WriteArincWord(BuildArincWordFloat(velNedCov.covED, vld, flip[0126]));        // V2.0
    WriteArincWord(BuildArincWordFloat(velNedCov.covDD, vld, flip[0127]));        // V2.0

    // latitude, longitude, height
    #define heiRange (131072.*ft)
    int32_t iLat = DAngDeg2int32(llhPosCov.lat), iLon = DAngDeg2int32(llhPosCov.lon);
    WriteArincWord(BuildArincWord(1048576.,     iLat>>11, vld?3:1, NOSDI, flip[0110]));
    WriteArincWord(BuildArincWord(2048.,    iLat & 0x7ff, vld?3:1,   SDI, flip[0120]));
    WriteArincWord(BuildArincWord(1048576.,     iLon>>11, vld?3:1, NOSDI, flip[0111]));
    WriteArincWord(BuildArincWord(2048.,    iLon & 0x7ff, vld?3:1,   SDI, flip[0121]));
    WriteArincWord(BuildArincWord(heiRange, double2float(llhPosCov.height), vld?3:1, NOSDI, flip[0370]));

    // UTC
    // total of 18 bits:                                     5                6                 6         1
    WriteArincWord(BuildArincWord(262144., (uint32_t)utcTime.hour<<13|utcTime.minute<<7|utcTime.second<<1|1, vld?3:1, SDI, flip[0150])); // hours, minutes, seconds
    uint64_t secFrac = ((uint64_t)utcTime.nanos<<30)/1000000000;  // nanoseconds > 2^-30 seconds
    WriteArincWord(BuildArincWord(1048576.,  secFrac>>10, vld?3:1, NOSDI, flip[0140])); // second fraction coarse
    WriteArincWord(BuildArincWord(1024., secFrac & 0x3ff, vld?3:1,   SDI, flip[0141])); // second fraction fine
    uint8_t tday=utcTime.day/10, uday=utcTime.day%10, tmon=utcTime.month/10, umon=utcTime.month%10, tyea=(utcTime.year/10)%10, uyea=utcTime.year%10;
    // total of 19 bits:                             2                  4        1        4       4       4
    WriteArincWord(BuildArincWord(262144., (uint32_t)tday<<17|(uint32_t)uday<<13|tmon<<12|umon<<8|tyea<<4|uyea, vld?0:1, SDI, flip[0260])); // day, month, year

    // DOPS
    WriteArincWord(BuildArincWord(102400., dops.hDop, vld?3:1, SDI, flip[0101])); // HDOP
    WriteArincWord(BuildArincWord(102400., dops.vDop, vld?3:1, SDI, flip[0102])); // VDOP
  
    // GNSS receiver status (SV count, status flags)
    // total of 18 bits:                                      8                          5                                    5
    WriteArincWord(BuildArincWord(131072., (int32_t)velNedCov.svCount << 10 | (velNedCov.statusFlags & 0x1f) << 5 | llhPosCov.statusFlags & 0x1f, vld?0:1, SDI, flip[0273]));
    // GPS SV count                                                                                                                                                                  // V2.0
    WriteArincWord(BuildArincWord(131072., (int32_t)velNedCov.svCount << 10,                                                                      vld?0:1, SDI, flip[0274]));        // V2.0
  
    // System monitor
    if (devMonRefreshed) {
        bool vld = true; // always valid since a message is received from device
        // total of 18 bits:                             9                                     9
        WriteArincWord(BuildArincWord(131072., (int32_t)(0.01* devMon.feTemp+0.5)<<9|(int32_t)(0.01*devMon.cpuTemp+0.5)&0x1ff, vld?0:1, SDI, flip[0357]));
        devMonRefreshed = false;
    }
}

// delayMicoseconds() with 32-bit integer argument
inline void myDelayUS(uint32_t d) {
    uint32_t startTime = micros();
    while (micros()-startTime < d);
}

// convert a double precision angle, in degrees, into a 32-bit integer (with 2^32 <=> 360°)
// double precision (64-bit) argument is stored in an int64_t since arduino has no double type
int32_t DAngDeg2int32(int64_t a) {
    // mantissa (including implicit MSB=1)
    int64_t m = (a & 0xfffffffffffff) | 0x10000000000000; // (52+1) bits
    // debiased exponant (11 bits)
    int32_t e = (int32_t)(((a >> 52) & 0x7ff)-1023);
    // scaled result
    int32_t r = a==0 ? 0 : (int32_t)(((e>0?m<<e:m>>-e)/360)>>20);
    // applies sign (bit 63)
    return a>>63 ? -r : r;
}

// convert a double precision floating point number into a single precision
// 64-bit argument is stored in an int64_t since arduino has no double type
float double2float(int64_t d) {
    // mantissa
    int64_t m = (d & 0xfffffffffffff); // 52 bits
    // debiased exponant (11 bits)
    int32_t e = (int32_t)(((d >> 52) & 0x7ff)-1023);
    // sign (1 bit)
    int32_t s = (int32_t)(d>>63);
    union {float f; int32_t i;} fi;
    fi.i = (s<<31) | ((e+127)<<23) | (int32_t)(m>>(52-23));
    return d==0 ? 0 : fi.f;
}

const unsigned char flip[] = {
    0x00, 0x80, 0x40, 0xC0, 0x20, 0xA0, 0x60, 0xE0, 0x10, 0x90, 0x50, 0xD0, 0x30, 0xB0, 0x70, 0xF0, 
    0x08, 0x88, 0x48, 0xC8, 0x28, 0xA8, 0x68, 0xE8, 0x18, 0x98, 0x58, 0xD8, 0x38, 0xB8, 0x78, 0xF8, 
    0x04, 0x84, 0x44, 0xC4, 0x24, 0xA4, 0x64, 0xE4, 0x14, 0x94, 0x54, 0xD4, 0x34, 0xB4, 0x74, 0xF4, 
    0x0C, 0x8C, 0x4C, 0xCC, 0x2C, 0xAC, 0x6C, 0xEC, 0x1C, 0x9C, 0x5C, 0xDC, 0x3C, 0xBC, 0x7C, 0xFC, 
    0x02, 0x82, 0x42, 0xC2, 0x22, 0xA2, 0x62, 0xE2, 0x12, 0x92, 0x52, 0xD2, 0x32, 0xB2, 0x72, 0xF2, 
    0x0A, 0x8A, 0x4A, 0xCA, 0x2A, 0xAA, 0x6A, 0xEA, 0x1A, 0x9A, 0x5A, 0xDA, 0x3A, 0xBA, 0x7A, 0xFA,
    0x06, 0x86, 0x46, 0xC6, 0x26, 0xA6, 0x66, 0xE6, 0x16, 0x96, 0x56, 0xD6, 0x36, 0xB6, 0x76, 0xF6, 
    0x0E, 0x8E, 0x4E, 0xCE, 0x2E, 0xAE, 0x6E, 0xEE, 0x1E, 0x9E, 0x5E, 0xDE, 0x3E, 0xBE, 0x7E, 0xFE,
    0x01, 0x81, 0x41, 0xC1, 0x21, 0xA1, 0x61, 0xE1, 0x11, 0x91, 0x51, 0xD1, 0x31, 0xB1, 0x71, 0xF1,
    0x09, 0x89, 0x49, 0xC9, 0x29, 0xA9, 0x69, 0xE9, 0x19, 0x99, 0x59, 0xD9, 0x39, 0xB9, 0x79, 0xF9, 
    0x05, 0x85, 0x45, 0xC5, 0x25, 0xA5, 0x65, 0xE5, 0x15, 0x95, 0x55, 0xD5, 0x35, 0xB5, 0x75, 0xF5,
    0x0D, 0x8D, 0x4D, 0xCD, 0x2D, 0xAD, 0x6D, 0xED, 0x1D, 0x9D, 0x5D, 0xDD, 0x3D, 0xBD, 0x7D, 0xFD,
    0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3, 0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3, 
    0x0B, 0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB, 0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB,
    0x07, 0x87, 0x47, 0xC7, 0x27, 0xA7, 0x67, 0xE7, 0x17, 0x97, 0x57, 0xD7, 0x37, 0xB7, 0x77, 0xF7, 
    0x0F, 0x8F, 0x4F, 0xCF, 0x2F, 0xAF, 0x6F, 0xEF, 0x1F, 0x9F, 0x5F, 0xDF, 0x3F, 0xBF, 0x7F, 0xFF
};
