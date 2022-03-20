/*
ToyotaSyncSerialSniffer
Using ESP32-S2
gets info from uart0 and uart1 and sends to a connected telnet client.
IP address default is 192.168.4.1
Telnet Port 23

loop() too slow for the Toyota Sniffer. Using event handlers instead. Messages come in at 2ms intervals
*/

#include "driver/uart.h"
#include <WiFi.h>

// Set these to your desired credentials.
const char *ssid = "ToyotaSniffer";
const char *password = "password";

WiFiServer server(23);     // set up on telnet
WiFiClient client;

#define HTM_PORT      UART_NUM_1
#define MTH_PORT      UART_NUM_0
#define HTM_RX_PIN    18
#define MTH_RX_PIN    43
#define BUF_SIZE      32768   // enough for 120bytes*200Hz (128*256) = 1 second of data
static QueueHandle_t  htm_queue;
static QueueHandle_t  mth_queue;

volatile bool clientConnected = false;
void uart_event_task(void *);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  uart_config_t ToyotaConfig = {
        .baud_rate = 500000,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
  };
  uart_param_config(HTM_PORT, &ToyotaConfig);
  uart_param_config(MTH_PORT, &ToyotaConfig);
  uart_set_pin(HTM_PORT, UART_PIN_NO_CHANGE, HTM_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_set_pin(MTH_PORT, UART_PIN_NO_CHANGE, MTH_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(HTM_PORT, BUF_SIZE, 0, 200, &htm_queue, 0);
  uart_driver_install(MTH_PORT, BUF_SIZE, 0, 200, &mth_queue, 0);
  uart_set_rx_timeout(HTM_PORT, 5);    // aim is that timeout will break up the messages
  uart_set_rx_timeout(MTH_PORT, 5);
  uart_set_rx_full_threshold(HTM_PORT, 121); // set lower if HW FIFO overflows at high bit rates.
  // You can remove the password parameter if you want the AP to be open.
  WiFi.softAP(ssid, password);
  server.begin();
  xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);
}

void loop() {
  ///////////////////////  static variables //////////////////////////////
  static uint32_t clientMicros;
  static uint32_t lastMicros = 0;
  static int32_t ledState = LOW;
  static uint32_t ledMicros = 0;
  ///////////////////////  loop variables ////////////////////////////////
  uint32_t loopMicros = micros();
  ///////////////////////  flash LED /////////////////////////////////////
  if(loopMicros > ledMicros){
    ledState = ledState == HIGH ? LOW : HIGH;
    digitalWrite(LED_BUILTIN,ledState);
    ledMicros +=500000;
  }
  //////////////////////  unconnected state  /////////////////////////////
  if(!client){
    clientConnected = false;
    client = server.available();              // check for incoming connection
    if(client){
      client.println("Send 'q (enter)' to quit ");
      clientMicros = loopMicros;
      clientConnected = true;
    }
  }
  //////////////////////  connected state  ///////////////////////////////
  if (client) {
    bool quit = false;
    while (client.available()) {             // if there's bytes to read from the client,
      uint8_t inByte=client.read();
      if(inByte=='q') {
        client.stop();
        quit = true;
        break;
      }
    }
  }
  lastMicros = loopMicros;
}

void uart_event_task(void * Unused)
{
  uart_event_t event;
  size_t buffered_size;
  bool exit_condition = false;
  struct {
    uint8_t newline = '\n';
    uint8_t prefix  = ' ';
    uint8_t length  = 0;
    uint8_t msg[128];
    uint8_t more    = '+';
  }inData;

  int data_length = 0;
  uint8_t allowRTOStime = 0;

  //Infinite loop to run main bulk of task
  while (1) {
    if(xQueueReceive(htm_queue, (void * )&event, allowRTOStime++%64 ? 0:1 ))   //check htm queue
    {
      //Handle received event
      switch (event.type) {
      case UART_DATA:
        data_length = uart_read_bytes(HTM_PORT, inData.msg, event.size, 0);
        if(client){
          inData.prefix = 'H';
          inData.length = data_length;
          if(!event.timeout_flag)
            inData.msg[data_length++]='+';
          client.write((uint8_t*)&inData, data_length + 3);
        }
        break;
      case UART_FRAME_ERR:
        if(client){
          client.write("\nhtmFRAME_ERR",13);
        }
        break;
      case UART_FIFO_OVF:  // fifo full? need to sort out rtos interrupt better
        if(client){
          client.write("\nhtmFIFO_OVF" ,12);
        }
        break;
      case UART_BUFFER_FULL: // buffer full, my task isn't keeping up
        if(client){
          client.write("\nhtmBUFF_FULL",13);
          uart_flush(HTM_PORT); //see if this recovers us
        }
        break;
      default:
        if(client){
          client.print("\nhtm,");
          client.println(event.type);
        }
      }
    }
    if(xQueueReceive(mth_queue, (void * )&event, 0))   //check mth queue, don't wait.
    {
      //Handle received event
      switch (event.type) {
      case UART_DATA:
        data_length = uart_read_bytes(MTH_PORT, inData.msg, event.size, 0);
        if(client){
          inData.prefix = 'M';
          inData.length = data_length;
          if(!event.timeout_flag)
            inData.msg[data_length++]='+';
          client.write((uint8_t*)&inData, data_length + 3);
        }
        break;
      case UART_FRAME_ERR:
        if(client){
          client.write("\nmthFRAME_ERR",13);
        }
        break;
      case UART_FIFO_OVF:  // fifo full? need to sort out rtos interrupt better
        if(client){
          client.write("\nhtmFIFO_OVF" ,12);
        }
        break;
      case UART_BUFFER_FULL: // buffer full, my task isn't keeping up
        if(client){
          client.write("\nmthBUFF_FULL",13);
          uart_flush(MTH_PORT); //see if this recovers us
        }
        break;
      default:
        if(client){
          client.print("mth,");
          client.println(event.type);
        }
      }
    }
    //If you want to break out of the loop due to certain conditions, set exit condition to true
    if (exit_condition) {
      break;
    }
  }
   
  //Out side of loop now. Task needs to clean up and self terminate before returning
  vTaskDelete(NULL);
}

/*
 * references:
 * https://github.com/espressif/arduino-esp32/blob/master/tools/sdk/esp32s2/include/driver/include/driver/uart.h
 * https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/uart.html
 * https://github.com/espressif/esp-idf/blob/2f9d47c708f39772b0e8f92d147b9e85aa3a0b19/examples/peripherals/uart/uart_events/main/uart_events_example_main.c
 * https://www.esp32.com/viewtopic.php?t=9939#
 * 
 */
