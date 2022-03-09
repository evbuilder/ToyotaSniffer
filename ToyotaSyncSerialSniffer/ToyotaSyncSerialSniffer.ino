/* Toyota Sniffer
 * Two SPI slave ports used for capturing both HTM and MTH data from Toyota Gen3prius and similar aged Hybrids.
 * 
 * Output might be too dense for Uart? we will see. Maybe a third SPI slave will be needed to get data to ESP32 or ESP8266 for WIFI extraction
 */
//I/O  Due  Due
//     Port NET   Sniffer
//PA8: 0    RX    RX from Atmega16U2. Dedicated UART does not clash with USART SPIs. Up to ~5MHz.
//PA9: 1    TX    TX to Atmega16U2. As above
//PA17 TWD0 SDA1  SCLK0 connect both SCLK to Toyota CLK
//PA16 AD0  AD0   SCLK1
//PA10 19   RXD2  MOSI0  Toyota HTM
//PA12 17   RXD1  MOSI1  Toyota MTH
//PB25 2    PWM2  CS0    Might have to frig the Chip Selects, connect them to a GPIO
//PA14 23   PIN23 CS1
//PB27 3    PWM3  GPIO to frig CSx
//PC26 4    PWM4  REQ    Toyota REQ might need to tie to a GPIO interrupt.


/******************* setup ***********************/
void DataReceived(void);
void ReqReceived(void);

void setup() {
  delay(1000);
  Serial.begin(115200);
  Serial.println();
  Serial.print("Starting Sniffer ...");

  Serial1.begin(9600);  // pre-initialize USART0
  Serial2.begin(9600);  // pre-initialize USART1
  USART0config();       // frig for spi slave
  USART1config();       // frig for spi slave
  
  attachInterrupt(digitalPinToInterrupt(4), ReqReceived, FALLING); //zero write pointers
// Interrupts: USART0 17, USART1 18
  attachInterrupt(17, DataReceived, HIGH); //Byte Received

}

uint8_t last_req = false;
uint8_t htm_buffer[256];
uint8_t mth_buffer[256];
volatile uint8_t htm_write_p = 0;
volatile uint8_t htm_read_p  = 0;
volatile uint8_t mth_write_p = 0;
volatile uint8_t mth_read_p  = 0;

/******************* LOOP ***********************/
void loop() {    

}

void DataReceived() {
  htm_buffer[htm_write_p++] = USART0->US_RHR;
  mth_buffer[mth_write_p++] = USART1->US_RHR;
}

void ReqReceived() {
  htm_write_p = 0;
  mth_write_p = 0;
}
/*-------------------------------------------------------------------------
Instance   Peripheral  Signal  SPI Master  SPI Slave  I/O Line  Arduino Pin
---------------------------------------------------------------------------
USART0     A, ID 17    RXD0    MISO        MOSI       PA10      RX1     19
USART0     A, ID 17    TXD0    MOSI        MISO       PA11      TX1     18
USART0     A, ID 17    RTS0    CS          x          PB25      D2       2
USART0     A, ID 17    CTS0    x           CS         PB26      D22     22
USART0     B, ID 17    SCK0    SCK         SCK        PA17      SDA1    70
---------------------------------------------------------------------------
USART1     A, ID 18    RXD1    MISO        MOSI       PA12      RX2     17
USART1     A, ID 18    TXD1    MOSI        MISO       PA13      TX2     16
USART1     A, ID 18    RTS1    CS          x          PA14      D23     23
USART1     A, ID 18    CTS1    x           CS         PA15      D24     24
USART1     A, ID 18    SCK1    SCK         SCK        PA16      A0      54
---------------------------------------------------------------------------
Register                            Page  Arduino          Settings 
USART Mode Register                 839   USART1->US_MR    CLK0, 8_BIT, SPI_SLAVE, CPOL, CPHA
USART Baud Rate Generator Register  855   USART1->US_BRGR  CD
USART Transmit Holding Register     854   USART1->US_THR   Use to write data
USART Receive Holding Register      853   USART1->US_RHR   Use to read data
USART Channel Status Register       849   USART1->US_RHR   Use to control SPI tranfers
-------------------------------------------------------------------------*/
void USART0config() {
  USART0->US_WPMR = 0x55534100;   // Unlock the USART Mode register
  USART0->US_MR |= 0x408CF;       // Set Mode to CLK0=1, 8_BIT, SPI_SLAVE
  USART0->US_BRGR = 21;           // Clock Divider (SCK = 4MHz) <-- NOT USED
  USART0->US_IMR =  (1<<0);       // ENABLE RXRDY INTERRUPT
//  USART0->US_THR = 0;             // preload the output or maybe just suffer the underrun errors

  PIOA->PIO_WPMR = 0x50494F00;    // Unlock PIOA Write Protect Mode Register
  PIOB->PIO_WPMR = 0x50494F00;    // Unlock PIOB Write Protect Mode Register
  PIOB->PIO_ABSR |= (0u << 26);   // CS: Assign A14 I/O to the Peripheral A function
  PIOB->PIO_PDR |= (1u << 26);    // CS: Disable PIO control, enable peripheral control
  PIOA->PIO_ABSR |= (1u << 17);   // SCK: Assign A16 I/O to the Peripheral B function
  PIOA->PIO_PDR |= (1u << 17);    // SCK: Disable PIO control, enable peripheral control
  PIOA->PIO_ABSR |= (0u << 10);   // MOSI: Assign PA13 I/O to the Peripheral A function
  PIOA->PIO_PDR |= (1u << 10);    // MOSI: Disable PIO control, enable peripheral control
  PIOA->PIO_ABSR |= (0u << 11);   // MISO: Assign A12 I/O to the Peripheral A function
  PIOA->PIO_PDR |= (1u << 11);    // MISO: Disable PIO control, enable peripheral control
}
void USART1config() {
  USART1->US_WPMR = 0x55534100;   // Unlock the USART Mode register
  USART1->US_MR |= 0x408CF;       // Set Mode to CLK0=1, 8_BIT, SPI_Slave
  USART1->US_BRGR = 21;           // Clock Divider (SCK = 4MHz) <-- NOT USED

  PIOA->PIO_WPMR = 0x50494F00;    // Unlock PIOA Write Protect Mode Register
  PIOB->PIO_WPMR = 0x50494F00;    // Unlock PIOB Write Protect Mode Register
  PIOA->PIO_ABSR |= (0u << 15);   // CS: Assign A14 I/O to the Peripheral A function
  PIOA->PIO_PDR |= (1u << 15);    // CS: Disable PIO control, enable peripheral control
  PIOA->PIO_ABSR |= (0u << 16);   // SCK: Assign A16 I/O to the Peripheral A function
  PIOA->PIO_PDR |= (1u << 16);    // SCK: Disable PIO control, enable peripheral control
  PIOA->PIO_ABSR |= (0u << 13);   // MOSI: Assign PA13 I/O to the Peripheral A function
  PIOA->PIO_PDR |= (1u << 13);    // MOSI: Disable PIO control, enable peripheral control
  PIOA->PIO_ABSR |= (0u << 12);   // MISO: Assign A12 I/O to the Peripheral A function
  PIOA->PIO_PDR |= (1u << 12);    // MISO: Disable PIO control, enable peripheral control
}
