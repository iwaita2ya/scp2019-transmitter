#define DEBUG

#include <mbed.h>
#include <ssd1306.h>
#include <standard_font.h>
#include <bold_font.h>

SSD1306 *oled;
DigitalOut *led;

/**
 * Serial
 */
RawSerial *device; // to devices TODO:Serialに変更してテストしてみる
RawSerial *transmitter; // to transmitter(XBee)
// Circular buffers for serial TX and RX data - used by interrupt routines
const int serialBufferSize = 255;

// might need to increase buffer size for high baud rates
unsigned char rxDeviceBuffer[serialBufferSize+1];
unsigned char rxTransmitterBuffer[serialBufferSize+1];

// Circular buffer pointers. Volatile makes read-modify-write atomic
volatile int rxDeviceInPointer=0;
volatile int rxDeviceOutPointer=0;
volatile int rxTransmitterInPointer=0;
volatile int rxTransmitterOutPointer=0;

// Line buffers for sprintf and sscanf
unsigned char rxDeviceLineBuffer[64];
unsigned char rxTransmitterLineBuffer[64];

InterruptIn *startDevicePin;
InterruptIn *resetDevicePin;

uint8_t isActive;

#ifdef DEBUG
#define DEBUG_PRINT(x)  transmitter->puts(x)
#define DEBUG_PUTC(x)  transmitter->putc(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PUTC(x)
#endif

// Function Prototypes --------------------------------------------------------

// ----- Serial -----
void rxDevice();
void readDeviceLineBuffer();
void readTransmitterLineBuffer();

void rxTransmitter();
void TxTransmitter(); // transmitterでデータ送信時に呼ばれる

// ----- OLED -----
void drawStatus(uint8_t statusByte, float pressureSeaLevel, float currentPressure, uint16_t groundAltitude, uint16_t currentAltitude);

// ----- Commands -----
void changeStatusParachute(uint8_t statusByte);

// Main  ----------------------------------------------------------------------
int main() {
    /**
     * Init Vars
     */
    isActive = 1;

    /**
     * Init Serial
     */
    // device
    device = new RawSerial(p13, p14, 115200); // tx, rx, baud
    device->attach(rxDevice, RawSerial::RxIrq);  // device->MCU

    // transmitter (XBee)
//    transmitter = new Serial(p28, p27, 115200); // tx, rx, baud MEMO:こっちが本番設定
    transmitter = new RawSerial(USBTX, USBRX, 115200); // tx, rx, baud (pc debug)
//    transmitter->attach(rxTransmitter, RawSerial::RxIrq);  // transmitter->MCU
//    transmitter->attach(TxTransmitter, Serial::TxIrq);// MCU->transmitter

    /**
     * Init OLED
     */
    oled = new SSD1306(p11, p8, p6, p7, p5); // CS, RES, DC/MISO, SCK(D0), MOSI/SDA(D1)
    oled->initialise();
    oled->clear();
    oled->set_contrast(255); // 0-255(max)
    // Bold font test
    oled->set_font(bold_font, 8);
    oled->printf("Bold Font\r\n");
    // Standard Font test
    oled->set_font(standard_font, 6);
    oled->printf("Hello World!\r\n");
    oled->printf("123456789012345678901234567890\r\n");
    oled->printf("ABCDEFGHIJKLMNOPQRSTUVWXYZ\r\n");
    // update display
    oled->update();

    /*
     * Init Pins(Buttons)
     */
    startDevicePin = new InterruptIn(p17); // デバイス開始ボタン
    startDevicePin->mode(PullUp);
//    startDevicePin->fall(&changeStatusParachute(0x01));


    /**
     * Init LED
     */
    led = new DigitalOut(LED1);
    led->write(1);

    /**
     * Main Loop
     */
    volatile int intDecoded;

    while(isActive) {

        // data received from device and not read yet?
        if (rxDeviceInPointer != rxDeviceOutPointer) {
            // rxDeviceLineBuffer に読み込む
            readDeviceLineBuffer();

            // prepare data
            uint8_t statusByte = rxDeviceLineBuffer[0];
            intDecoded = (rxDeviceLineBuffer[4] << 24 | rxDeviceLineBuffer[3] << 16 | rxDeviceLineBuffer[2] << 8 | rxDeviceLineBuffer[1]);
            float pressureAtSeaLevel = *((float*)&intDecoded);

            intDecoded = (rxDeviceLineBuffer[8] << 24 | rxDeviceLineBuffer[7] << 16 | rxDeviceLineBuffer[6] << 8 | rxDeviceLineBuffer[5]);
            float currentPressure = *((float*)&intDecoded);

            uint16_t groundAltitude  = (rxDeviceLineBuffer[10] << 8 | rxDeviceLineBuffer[9]);
            uint16_t currentAltitude = (rxDeviceLineBuffer[12] << 8 | rxDeviceLineBuffer[11]);;

            //MEMO: OLEDに表示
            drawStatus(statusByte, pressureAtSeaLevel, currentPressure, groundAltitude, currentAltitude);
        }

//        // data received from transmitter and not read yet?
//        if (rxTransmitterInPointer != rxTransmitterOutPointer) {
//            // rxTransmissionLineBuffer に読み込む
//            readTransmitterLineBuffer();
//
////            // OLEDに表示してみる
////            uint8_t statusByte = rxTransmitterLineBuffer[0];
////
////            intDecoded = (rxTransmitterLineBuffer[4] << 24 | rxTransmitterLineBuffer[3] << 16 | rxTransmitterLineBuffer[2] << 8 | rxTransmitterLineBuffer[1]);
////            float decoded = *((float*)&intDecoded);
////
////            drawStatus(statusByte, decoded);
//            device->puts((char*)rxTransmitterLineBuffer);
//        }

        // update led
        led->write(!led->read());
        wait_ms(500);
    }

    /**
     * Terminate
     */
     delete(device);
     delete(transmitter);
     delete(oled);
     delete(led);
}

// Functions  -----------------------------------------------------------------
/**
 * Serial
 */

// device -> MCU
void rxDevice() {
    // Loop just in case more than one character is in UART's receive FIFO buffer Stop if buffer full
    while ((device->readable()) && (((rxDeviceInPointer + 1) % serialBufferSize) != rxDeviceOutPointer)) {
        rxDeviceBuffer[rxDeviceInPointer] = device->getc();

        // Uncomment to echo back to transmitter to watch data flow
        //DEBUG_PUTC(rxDeviceBuffer[rxDeviceInPointer]);

        rxDeviceInPointer = (rxDeviceInPointer + 1) % serialBufferSize;
    }
}

// Read a line from the large rx buffer (device)
void readDeviceLineBuffer() {

    uint8_t i = 0;

    // Start Critical Section - don't interrupt while changing global buffer variables
    NVIC_DisableIRQ(UART1_IRQn); // UART1 Interrupt

    // Loop reading rx buffer characters until end of line character
    while ((i==0) || rxDeviceInPointer != rxDeviceOutPointer) {
        rxDeviceLineBuffer[i++] = rxDeviceBuffer[rxDeviceOutPointer];
        rxDeviceOutPointer = (rxDeviceOutPointer + 1) % serialBufferSize;
    }

    // End Critical Section
    NVIC_EnableIRQ(UART1_IRQn);
}


// transmitter -> MCU
void rxTransmitter() {

    // pass-through to device
//    device->putc(transmitter->getc());

    //MEMO: とりあえずLCDに表示する
    while ((transmitter->readable()) && (((rxTransmitterInPointer + 1) % serialBufferSize) != rxTransmitterOutPointer)) {
        rxTransmitterBuffer[rxTransmitterInPointer] = transmitter->getc();

        // Uncomment to echo back to transmitter to watch data flow
//        DEBUG_PUTC(rxTransmitterBuffer[rxTransmitterInPointer]);

        rxTransmitterInPointer = (rxTransmitterInPointer + 1) % serialBufferSize;
    }
}

// Read a line from the large rx buffer (transmitter)
void readTransmitterLineBuffer() {

    uint8_t i = 0;

    // Start Critical Section - don't interrupt while changing global buffer variables
    NVIC_DisableIRQ(UART0_IRQn); // UART0 Interrupt(USBTx, USBRx) //FIXME: p27,p28 を使う場合は UART2_IRQn

    // Loop reading rx buffer characters until end of line character
    while ((i==0) || rxTransmitterInPointer != rxTransmitterOutPointer) {
        rxTransmitterLineBuffer[i++] = rxTransmitterBuffer[rxTransmitterOutPointer];
        rxTransmitterOutPointer = (rxTransmitterOutPointer + 1) % serialBufferSize;
    }

    // End Critical Section
    NVIC_EnableIRQ(UART0_IRQn);
}

/*
 * OLED Display
 */

void drawStatus(uint8_t statusByte, float pressureSeaLevel, float currentPressure, uint16_t groundAltitude, uint16_t currentAltitude) {

//    oled->set_display_start_line(0); // draw from the top
    oled->clear();

    oled->printf("ErFnTdOpFaFlStIn\r\n");
    oled->printf(" %d %d %d %d %d %d %d %d\r\n"
            , ((statusByte & 0x80) ? 1 : 0)
            , ((statusByte & 0x40) ? 1 : 0)
            , ((statusByte & 0x20) ? 1 : 0)
            , ((statusByte & 0x10) ? 1 : 0)
            , ((statusByte & 0x08) ? 1 : 0)
            , ((statusByte & 0x04) ? 1 : 0)
            , ((statusByte & 0x02) ? 1 : 0)
            , ((statusByte & 0x01) ? 1 : 0)
    );
    oled->printf("Pss  0m: %d Pa\r\n", (int)(pressureSeaLevel));
    oled->printf("Pss Cur: %d Pa\r\n", (int)(currentPressure));
    oled->printf("Gnd Alt: %u m\r\n", groundAltitude);
    oled->printf("Cur Alt: %u m\r\n", currentAltitude);

    oled->update();
}

/**
 * Parachute のステータスを更新する
 * @param statusByte
 */
void changeStatusParachute(uint8_t statusByte) {
    char commandArray[2];
    commandArray[0]= 0x20;
    commandArray[1]= statusByte;

    // parachute にコマンドを送る
    device->puts(commandArray);
    wait_ms(100);
}