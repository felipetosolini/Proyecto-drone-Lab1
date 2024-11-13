/* Tested with 16MHz ATmega328p-AP, QuadX
Motors use pins 9,6,5,3 instead of 9,10,11,3
*/

#ifndef NRF24_RX_H_
#define NRF24_RX_H_

#include "config.h"

#if defined(NRF24_RX) || defined(BLUETOOTH_RX)

// Estructura de datos para NRF24
struct RF24Data {
  byte throttle;
  byte yaw;
  byte pitch;
  byte roll;
  byte AUX1;
  byte AUX2;
  byte switches;
};

struct RF24AckPayload {
  float lat;
  float lon;
  int16_t heading;
  int16_t pitch;
  int16_t roll;  
  int32_t alt;
  byte flags;
};

extern RF24Data MyData;
extern RF24AckPayload nrf24AckPayload;
extern int16_t nrf24_rcData[RC_CHANS];

// Estructura de datos para Bluetooth
struct BluetoothData {
  byte throttle;
  byte yaw;
  byte pitch;
  byte roll;
  byte AUX1;
  byte AUX2;
  byte switches;
};

struct BluetoothAckPayload {
  float lat;
  float lon;
  int16_t heading;
  int16_t pitch;
  int16_t roll;  
  int32_t alt;
  byte flags;
};

extern BluetoothData btData;
extern BluetoothAckPayload btAckPayload;
extern int16_t bt_rcData[RC_CHANS];

void NRF24_Init();
void NRF24_Read_RC();
void Bluetooth_Init();
void Bluetooth_Read_RC();

#endif

#endif /* NRF24_RX_H_ */


