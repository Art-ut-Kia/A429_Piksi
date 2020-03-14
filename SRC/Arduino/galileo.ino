/**
 * galileo.ino: Bridges a Swift Navigation piksi-multi GNSS module to a Naveol Nav429 ARINC shield
 * author: JP Pétillon
 * status: under construction
 */

//..............................................................................
// initialization
void setup() {
    Serial.begin(9600);
}

//..............................................................................
// global (hence remanant) variables
bool preambleRxd = false; // preamble allready received
struct {
    uint8_t preamble;
    uint16_t type;
    uint16_t sender;
    uint8_t len;
    uint8_t data[256];
} msg;
int iter = 0;

//..............................................................................
// checks the CRC of the message
bool crcGood() {
    return true;
}

//..............................................................................
// cyclic function
void loop() {
    while (!Serial.available()); // polls UART until something is received
    ((uint8_t*)&msg)[iter++] = Serial.read(); // stores received charecter

    // message decoding state-machine
    if (!preambleRxd) {
        if (msg.preamble==0x55) preambleRxd = true;
        else iter = 0;
    } else if (iter == msg.len+8) {
        if (crcGood()) { // processes received message
            if (msg.type==0x020E) { // Velocity in NED
                
            }
            if (msg.type==0x00B5) { // Device temperature and voltage levels
              
            }
        }
        // gets ready for next message
        preambleRxd = msg.preamble = iter = 0;
    }
}
