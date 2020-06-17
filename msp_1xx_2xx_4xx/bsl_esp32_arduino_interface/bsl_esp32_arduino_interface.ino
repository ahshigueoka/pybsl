/************************************************************************
 * Initial definitions for this BSL interface
 */
// The ESP32 pin connected to the  TEST pin of the MSP
#define    PIN_TEST     12
// The ESP32 pin connected to the RESET pin of the MSP
#define    PIN_RESET    13
// The ESP32 pin connected to the TX pin of the MSP
#define    RXD2         16
// The ESP32 pin connected to the RX pin of the MSP
#define    TXD2         17
// LED pins used for debugging purposes
#define    LED0         25
#define    LED1         26
#define    LED2         27

/**************************************************************+*********
 * State enum type
 * 
 * Keep track of the operating mode of this ESP32 interface
 * STANDBY   : waiting for a data frame from either the remote
 *            server or the microcontroller
 * SRV_FRAME : received data frame from server
 * SRV_TASK  : performing task received from the server
 * BSL_INVO  : send BSL invocation to the MSP
 * BSL_SYNC  : sending SYNC_CHR to MSP
 * BSL_MODE  : BSL ready
 * BSL_FRAME : received BSL data frame from remote server
 * BSL_TASK  : resending BSL data frame to microcontroller
 * MSP_FRAME : received data frame from microcontroller
 * MSP_TASK  : performing the task from the microcontroller
 */

// States for the server communication
#define INIT        0
#define STANDBY     1
#define BSL_INVO    2
#define BSL_MODE    3
#define BSL_FRAME   4
#define MSP_RDIR    5
#define SRV_RDIR    6

// Size of payload buffer used for communication
#define BUFFERSIZE  255

// Extra header codes used for wireless communication
#define BSL_REQ    0xB0   // Send BSL invocation
#define BSL_EXIT   0xB1   // Exit BSL mode
#define TX_REQ     0x10   // Transmit request

//-----------------------------------------------------------------------
// Special bytes used by BSL
#define HEAD_BSL    0x80
#define SYNC_CHR    0x80
#define DATA_ACK    0x90
#define DATA_NAK    0xA0

/************************************************************************
 * User specified type definitions
 */
typedef struct bslparams {
    byte  hdr;
    byte  cmd;
    byte  l1;
    byte  l2;
    byte* dat;
    byte  ckl;
    byte  ckh;
};

/**************************************************************+*********
 * Global variables
 * 
 */
// Used to keep track of the operations in execution
unsigned int state;

// Control the communication parameters
bslparams params;

// Data buffers for the server and microcontroller communications
byte data_buffer[BUFFERSIZE];

/**************************************************************+*********
 * Setup routine
 * 
 */
void setup(){
    state = INIT;

    params.dat = data_buffer;
    
    pinMode(PIN_TEST, OUTPUT);
    pinMode(PIN_RESET, OUTPUT);

    // LED pins
    pinMode(LED0, OUTPUT);
    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);
    digitalWrite(LED0, LOW);
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);

    // Serial: communication between PC and ESP
    Serial.begin(9600, SERIAL_8N1);
    while(!Serial);
    Serial.flush();

    // Serial2: communication between ESP and MSP
    Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
    while(!Serial2);
    Serial2.println("AAA");
    Serial2.flush();
    state = STANDBY;
}

/**************************************************************+*********
 * Main application
 */
void loop(){
    int retcode = 0;

    assign_leds(state);
    
    //-------------------------------------------------------------------
    // Check requests from server
    if(state == STANDBY) {
        if(Serial.available()) {
            // Start frame conversion
            retcode = get_frame(Serial, &params);
            // Choose operation according to the frame type
            switch(params.hdr) {
                case BSL_REQ:
                    // Try to start the BSL mode
                    state = BSL_INVO;
                    // The acknowledge from the MSP will be sent later
                    break;
                case TX_REQ:
                    // Redirect frame to MSP
                    state = MSP_RDIR;
                    // The acknowledge from the MSP will be sent later
                    break;
                default:
                    // Wrong frame. Send DATA_NAK and stay in standby
                    Serial.write(DATA_NAK);
                    break;
            }
        }
    }

    //-------------------------------------------------------------------
    // Send BSL invocation sequence to the MSP
    if(state == BSL_INVO) {
        // Send the invocation sequence
        bsl_invocation_test(PIN_TEST, PIN_RESET);
        // Send the synchronization character
        Serial2.write(SYNC_CHR);
        // Keep reading until response is DATA_ACK or DATA_NAK
        while(1) {
            while(!Serial2.available());
            retcode = Serial2.read();
            // Redirect acknowledge
            Serial.write(retcode);
            if(retcode == DATA_ACK) {
                // Successfully entered BSL mode
                state = BSL_MODE;
                break;
            }
            else if(retcode == DATA_NAK) {
                // Failed to enter BSL mode
                state = STANDBY;
                break;
            }
        }
    }
}

/************************************************************************
 * void bsl_invocation_tck(byte pin_tck, byte pin_rst)
 * 
 * Send the BSL invocation for microcontrollers with
 * dedicated JTAG pins, that is, using the
 * TCK and RESET pins.
 */
void bsl_invocation_tck(byte pin_tck, byte pin_rst) {
  
    // Initialize pin states
    digitalWrite(pin_tck,  HIGH);
    digitalWrite(pin_rst,  LOW);
    delay(10);
    
    // Send the first TCK pulse
    digitalWrite(pin_tck,  LOW);
    delay(1);
    digitalWrite(pin_tck,  HIGH);
    delay(1);
    
    // Start the second TCK pulse and send a RESET
    digitalWrite(pin_tck,  LOW);
    delay(1);
    digitalWrite(pin_rst, HIGH);
    delay(1);

    // The BSL starts at the rising edge of the second TCK pulse
    digitalWrite(pin_tck, HIGH);
}

/************************************************************************
 * void bsl_invocation_test(byte pin_test, byte pin_rst)
 * 
 * Send the BSL invocation for microcontrollers with
 * shared JTAG pins, that is, using the
 * TEST and RESET pins.
 */
void bsl_invocation_test(byte pin_test, byte pin_rst) {
    // Initialize pin states
    digitalWrite(pin_test,  LOW);
    digitalWrite(pin_rst,  LOW);
    delay(10);
    
    // Send the first TEST pulse
    digitalWrite(pin_test,  HIGH);
    delay(1);
    digitalWrite(pin_test,  LOW);
    delay(1);
    
    // Start the second TEST pulse and send a RESET
    digitalWrite(pin_test,  HIGH);
    delay(1);
    digitalWrite(pin_rst, HIGH);
    delay(1);

    // The BSL starts at the falling edge of the second TEST pulse
    digitalWrite(pin_test, LOW);
}

/************************************************************************
 * int get_frame_server(Stream &port, bslparams *bslptr)
 * 
 * Receive the BSL data frame and fill in the parameters
 * in the struct pointed to by bslptr and the buffer
 * data pointed to by bslptr->dat
 * 
 * Returns:
 *     > 0: if successful. The exact value equals the number
 *          of read bytes.
 *      -1: if could not get the header.
 *      -2: if could not get the command code.
 *      -3: if could not get L1.
 *      -4: if could not get L2.
 *      -5: if could not get all the data from AL to Dn
 *      -6: if could not get CKL
 *      -7: if could not get CKH
 */
int get_frame(Stream &port, bslparams *bslptr) {
    int temp;
    int num_read_bytes = 0;
    
    // Get the header---------------------------------
    while(!port.available());
    temp = port.read();
    // If temp == -1, could not get header
    if(temp == -1) return -1;
    ++num_read_bytes;
    bslptr->hdr = temp & 0xFF;

    // Get the command code --------------------------
    while(!port.available());
    temp = port.read();
    // If temp == -1, could not get the command code
    if(temp == -1) return -2;
    ++num_read_bytes;
    bslptr->cmd = temp & 0xFF;

    // Get the payload size---------------------------
    // get L1
    while(!port.available());
    temp = port.read();
    // If temp == -1, could not get L1
    if(temp == -1) return -3;
    ++num_read_bytes;
    bslptr->l1 = temp & 0xFF;
    
    // get L2
    while(!port.available());
    temp = port.read();
    // If temp == -1, could not get L2
    if(temp == -1) return -4;
    ++num_read_bytes;
    bslptr->l2 = temp & 0xFF;

    // Store the remaining data in the buffer---------
    if(bslptr->l1 > 0) {
        while(port.available() < bslptr->l1);
        temp = port.readBytes(bslptr->dat, bslptr->l1);
        if(temp < bslptr->l1) return -5;
        num_read_bytes += temp;
    }

    // Get the checksum ------------------------------
    // get CKL
    while(!port.available());
    temp = port.read();
    // If temp == -1, could not get CKL
    if(temp == -1) return -6;
    ++num_read_bytes;
    bslptr->ckl = temp & 0xFF;
        
    // get CKH
    while(!port.available());
    temp = port.read();
    // If temp == -1, could not get CKH
    if(temp == -1) return -7;
    ++num_read_bytes;
    bslptr->ckh = temp & 0xFF;

    return num_read_bytes;
}

/************************************************************************
 * void send_frame(HardwareSerial &port, bslparams *bslptr)
 */
int send_frame(HardwareSerial &port, bslparams *bslptr) {
    int nb = 0;
    int data_size;

    data_size = bslptr->l1 - 4;

    nb += port.write(bslptr->hdr);
    nb += port.write(bslptr->cmd);
    nb += port.write(bslptr->l1);
    nb += port.write(bslptr->l2);
    if(data_size > 0)
        nb += port.write(bslptr->dat, data_size);
    nb += port.write(bslptr->ckl);
    nb += port.write(bslptr->ckh);

    return nb;
}

/************************************************************************
 * void show_state(state)
 */
void assign_leds(unsigned int value) {
    if(value & 0x01)
        digitalWrite(LED0, HIGH);
    else
        digitalWrite(LED0, LOW);

    if(value & 0x02)
        digitalWrite(LED1, HIGH);
    else
        digitalWrite(LED1, LOW);

    if(value & 0x04)
        digitalWrite(LED2, HIGH);
    else
        digitalWrite(LED2, LOW);
}
