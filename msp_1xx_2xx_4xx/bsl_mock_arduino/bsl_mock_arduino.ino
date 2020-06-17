/**************************************************************+***************
 * Simulates the behaviour of the BSL activation of MSP430
 * microprocessors.
 */
 
// Pins used for BSL invocation
#define    PIN_TEST     2
#define    PIN_RESET    3

#define    LED0     22
#define    LED1     23
#define    LED2     24
#define    LED3     25

#define    SYNC_CHR    0x80
#define    DATA_ACK    0x90
#define    DATA_NAK    0xA0

#define    BUFFERSIZE   255

/***************************************************************************
 * User defined types
 */
enum State {
    INIT,
    STANDBY,
    TEST1UP,
    TEST1DOWN,
    TEST2UP,
    RESETUP,
    WAITSYNC,
    BSLMODE};

State curstate;

typedef struct bslparams {
    byte  hdr;
    byte  cmd;
    byte   l1;
    byte   l2;
    byte* dat;
    byte  ckl;
    byte  ckh;
};

bslparams params;

byte  data_buffer[BUFFERSIZE];

/****************************************************************************
 * Initializing routine
 */
void setup(void) {
    // Initialize outside BSL mode
    curstate = INIT;

    params.dat = data_buffer;
    
    // Configure debug LED pins
    pinMode(LED0, OUTPUT);
    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);
    pinMode(LED3, OUTPUT);
    
    // Initialize both serial ports:
    Serial.begin(9600);
    Serial.flush();
    Serial1.begin(9600, SERIAL_8N1);
    Serial1.flush();
        
    // Assign interrupts to pins
    pinMode(PIN_TEST , INPUT);
    pinMode(PIN_RESET, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_TEST ), isr_pin_test , CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_RESET), isr_pin_reset, RISING);

    Serial.print("BSL mock ready.\n");

    curstate = STANDBY;
    assign_leds(curstate);
}
/****************************************************************************
 * Main loop
 */
void loop(void) {
    int inByte = 0;

    assign_leds(curstate);

    if(curstate == STANDBY || curstate == BSLMODE) {
        if(Serial.available()) {
            inByte = Serial.read();
            Serial1.write(inByte);
        }
        if(Serial1.available()) {
            inByte = Serial1.read();
            Serial.write(inByte);
        }
    }

    // Waiting for SYNCH_CHR
    if(curstate == WAITSYNC) {
        Serial1.flush();
        Serial.println("Waiting for sync...");
        while(!Serial1.available());
        inByte = Serial1.read();
        Serial.print("Received byte: ");
        Serial.println(inByte, HEX);
        if(inByte == SYNC_CHR) {
            // Send an acknowledge character
            Serial.println("Sending back a DATA_ACK");
            Serial1.write(DATA_ACK);
            Serial.println("Entered BSL mode.");
            curstate = BSLMODE;
        }
        else {
            Serial.println("Sending back a DATA_NAK");
            Serial1.write(DATA_NAK);
            Serial.println("Back to STANDBY");
            curstate = STANDBY;
        }
    }
}
/****************************************************************************
 * Auxiliary functions
 */
void togglePin(int pin) {
    if(digitalRead(pin) == HIGH)
        digitalWrite(pin, LOW );
    else
        digitalWrite(pin, HIGH);
}

//---------------------------------------------------------------------------
void isr_pin_test(void) {
    // Determine whether it is a rising or falling edge
    int rising;
    
    if(digitalRead(PIN_TEST) == HIGH)
        rising = true;
    else
        rising = false;
        
    if(curstate == STANDBY && rising)
        curstate = TEST1UP;
    else if(curstate == TEST1UP && !rising)
        curstate = TEST1DOWN;
    else if(curstate == TEST1DOWN && rising)
        curstate = TEST2UP;
    else if(curstate == RESETUP && !rising)
        curstate = WAITSYNC;
}

//---------------------------------------------------------------------------
void isr_pin_reset(void) {
    if(curstate == TEST2UP)
        curstate = RESETUP;
}

/************************************************************************
 * void show_curcurstate(curstate)
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

    if(value & 0x08)
        digitalWrite(LED3, HIGH);
    else
        digitalWrite(LED3, LOW);
}
