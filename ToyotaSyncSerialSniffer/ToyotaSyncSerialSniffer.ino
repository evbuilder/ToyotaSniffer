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

#define HTM_PORT      UART_NUM_0
#define MTH_PORT      UART_NUM_1
#define HTM_RX_PIN    43
#define MTH_RX_PIN    18
#define BUF_SIZE      (256*8)
static QueueHandle_t  htm_queue;
static QueueHandle_t  mth_queue;
struct port_t {
  const QueueHandle_t* queueHandle;
  int                  port;
  const char*          str;
};
struct port_t htmPort { &htm_queue, HTM_PORT,"htm"};
struct port_t mthPort { &mth_queue, MTH_PORT,"mth"};
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
  uart_driver_install(HTM_PORT, BUF_SIZE, 0, 20, &htm_queue, 0);
  uart_driver_install(MTH_PORT, BUF_SIZE, 0, 20, &mth_queue, 0);
  uart_set_rx_timeout(HTM_PORT, 5);    // aim is that timeout will break up the messages
  uart_set_rx_timeout(MTH_PORT, 5);
  uart_set_rx_full_threshold(HTM_PORT, 121); // set lower if HW FIFO overflows at high bit rates.
  // You can remove the password parameter if you want the AP to be open.
  WiFi.softAP(ssid, password);
  server.begin();
  xTaskCreate(uart_event_task, "uart_event_task", 2048, &htmPort, 12, NULL);
  xTaskCreate(uart_event_task, "uart_event_task", 2048, &mthPort, 12, NULL);
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

void uart_event_task(void * Handle)
{
  struct port_t *portHandle = (struct port_t*) Handle;
  uart_event_t event;
  size_t buffered_size;
  bool exit_condition = false;
  uint8_t inData[128];
  int data_length = 0;

  //Infinite loop to run main bulk of task
  while (1) {
     
    //Loop will continually block (i.e. wait) on event messages from the event queue
    if(xQueueReceive(*portHandle->queueHandle, (void * )&event, (portTickType)portMAX_DELAY))
    {
      //Handle received event
      switch (event.type) {
      case UART_DATA:
        uart_read_bytes(portHandle->port, inData, event.size, portMAX_DELAY);
        if(client){
          client.print(portHandle->str);
          client.print(',');
          client.print(event.size);
          client.print(',');
          client.print(millis());
          client.print(',');
          for(byte i=0; i<event.size;i++){
            client.print(inData[i]);
            client.print(',');
          }
          if(event.timeout_flag)
            client.println("");
          else
            client.println("...");
        }
        break;
      case UART_FRAME_ERR:
        if(client){
          client.print(portHandle->str);
          client.println(",FRAME ERROR");
        }
        break;
      default:
        if(client){
          client.print(portHandle->str);
          client.print(',');
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
