#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "my_serial.h"
#include "MultiWii.h"

static volatile uint8_t serialHeadRX[UART_NUMBER], serialTailRX[UART_NUMBER];
static uint8_t serialBufferRX[RX_BUFFER_SIZE][UART_NUMBER];
static volatile uint8_t serialHeadTX[UART_NUMBER], serialTailTX[UART_NUMBER];
static uint8_t serialBufferTX[TX_BUFFER_SIZE][UART_NUMBER];

// *******************************************************
// Para Teensy 2.0, estas funciones emulan la API usada para ProMicro.
// No pueden tener el mismo nombre que en la API de Arduino porque no compilaría en el ProMini.
// *******************************************************
#if defined(TEENSY20)
unsigned char T_USB_Available() {
  int n = Serial.available();
  if (n > 255) n = 255;
  return n;
}
#endif

// *******************************************************
// Transmisor UART basado en interrupciones - usando un buffer circular
// *******************************************************

#if defined(PROMINI) || defined(MEGA)
  #if defined(PROMINI)
  ISR(USART_UDRE_vect_custom) {  // Serial 0 en un PROMINI
  #endif
  #if defined(MEGA)
  ISR(USART0_UDRE_vect_custom) { // Serial 0 en una MEGA
  #endif
    uint8_t t = serialTailTX[0];
    if (serialHeadTX[0] != t) {
      if (++t >= TX_BUFFER_SIZE) t = 0;
      UDR0 = serialBufferTX[t][0];  // Transmite el siguiente byte en el buffer
      serialTailTX[0] = t;
    }
    if (t == serialHeadTX[0]) UCSR0B &= ~(1 << UDRIE0); // Si se transmiten todos los datos, deshabilita la interrupción de transmisor UDRE
  }
#endif

#if defined(MEGA) || defined(PROMICRO)
ISR(USART1_UDRE_vect_custom) {  // Serial 1 en una MEGA o PROMICRO
  uint8_t t = serialTailTX[1];
  if (serialHeadTX[1] != t) {
    if (++t >= TX_BUFFER_SIZE) t = 0;
    UDR1 = serialBufferTX[t][1];  // Transmite el siguiente byte en el buffer
    serialTailTX[1] = t;
  }
  if (t == serialHeadTX[1]) UCSR1B &= ~(1 << UDRIE1);
}
#endif

#if defined(MEGA)
ISR(USART2_UDRE_vect_custom) {  // Serial 2 en una MEGA
  uint8_t t = serialTailTX[2];
  if (serialHeadTX[2] != t) {
    if (++t >= TX_BUFFER_SIZE) t = 0;
    UDR2 = serialBufferTX[t][2];
    serialTailTX[2] = t;
  }
  if (t == serialHeadTX[2]) UCSR2B &= ~(1 << UDRIE2);
}

ISR(USART3_UDRE_vect_custom) {  // Serial 3 en una MEGA
  uint8_t t = serialTailTX[3];
  if (serialHeadTX[3] != t) {
    if (++t >= TX_BUFFER_SIZE) t = 0;
    UDR3 = serialBufferTX[t][3];
    serialTailTX[3] = t;
  }
  if (t == serialHeadTX[3]) UCSR3B &= ~(1 << UDRIE3);
}
#endif

void UartSendData(uint8_t port) {
  #if defined(PROMINI)
    UCSR0B |= (1 << UDRIE0);
  #endif
  #if defined(PROMICRO)
    switch (port) {
      case 0:
        while (serialHeadTX[0] != serialTailTX[0]) {
          if (++serialTailTX[0] >= TX_BUFFER_SIZE) serialTailTX[0] = 0;
          #if !defined(TEENSY20)
            USB_Send(USB_CDC_TX, serialBufferTX[serialTailTX[0]], 1);
          #else
            Serial.write(serialBufferTX[serialTailTX[0]], 1);
          #endif
        }
        break;
      case 1: UCSR1B |= (1 << UDRIE1); break;
    }
  #endif
  #if defined(MEGA)
    switch (port) {
      case 0: UCSR0B |= (1 << UDRIE0); break;
      case 1: UCSR1B |= (1 << UDRIE1); break;
      case 2: UCSR2B |= (1 << UDRIE2); break;
      case 3: UCSR3B |= (1 << UDRIE3); break;
    }
  #endif
}

#if defined(GPS_SERIAL)
bool SerialTXfree(uint8_t port) {
  return (serialHeadTX[port] == serialTailTX[port]);
}
#endif

void SerialOpen(uint8_t port, uint32_t baud) {
  uint8_t h = ((F_CPU / 4 / baud - 1) / 2) >> 8;
  uint8_t l = ((F_CPU / 4 / baud - 1) / 2);
  switch (port) {
    #if defined(PROMINI)
      case 0: UCSR0A = (1 << U2X0); UBRR0H = h; UBRR0L = l; UCSR0B |= (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0); break;
    #endif
    #if defined(PROMICRO)
      #if (ARDUINO >= 100) && !defined(TEENSY20)
        case 0: UDIEN &= ~(1 << SOFE); break;  // Desactiva la interrupción del marco USB de Arduino para evitar jitter
      #endif
      case 1: UCSR1A = (1 << U2X1); UBRR1H = h; UBRR1L = l; UCSR1B |= (1 << RXEN1) | (1 << TXEN1) | (1 << RXCIE1); break;
    #endif
    #if defined(MEGA)
      case 0: UCSR0A = (1 << U2X0); UBRR0H = h; UBRR0L = l; UCSR0B |= (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0); break;
      case 1: UCSR1A = (1 << U2X1); UBRR1H = h; UBRR1L = l; UCSR1B |= (1 << RXEN1) | (1 << TXEN1) | (1 << RXCIE1); break;
      case 2: UCSR2A = (1 << U2X2); UBRR2H = h; UBRR2L = l; UCSR2B |= (1 << RXEN2) | (1 << TXEN2) | (1 << RXCIE2); break;
      case 3: UCSR3A = (1 << U2X3); UBRR3H = h; UBRR3L = l; UCSR3B |= (1 << RXEN3) | (1 << TXEN3) | (1 << RXCIE3); break;
    #endif
  }
}

void SerialEnd(uint8_t port) {
  switch (port) {
    #if defined(PROMINI)
      case 0: UCSR0B &= ~((1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0) | (1 << UDRIE0)); break;
    #endif
    #if defined(PROMICRO)
      case 1: UCSR1B &= ~((1 << RXEN1) | (1 << TXEN1) | (1 << RXCIE1) | (1 << UDRIE1)); break;
    #endif
    #if defined(MEGA)
      case 0: UCSR0B &= ~((1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0) | (1 << UDRIE0)); break;
      case 1: UCSR1B &= ~((1 << RXEN1) | (1 << TXEN1) | (1 << RXCIE1) | (1 << UDRIE1)); break;
      case 2: UCSR2B &= ~((1 << RXEN2) | (1 << TXEN2) | (1 << RXCIE2) | (1 << UDRIE2)); break;
      case 3: UCSR3B &= ~((1 << RXEN3) | (1 << TXEN3) | (1 << RXCIE3) | (1 << UDRIE3)); break;
    #endif
  }
}

// No se considera el desbordamiento del buffer circular (head->tail) para evitar una condición de prueba: los datos se pierden de todos modos si sucede.
void store_uart_in_buf(uint8_t data, uint8_t portnum) {
  uint8_t h = serialHeadRX[portnum];
  if (++h >= RX_BUFFER_SIZE) h = 0;
  if (h == serialTailRX[portnum]) return;  // En este caso, el buffer está lleno y se descarta el nuevo byte
  serialBufferRX[h][portnum] = data;
  serialHeadRX[portnum] = h;
}

#if defined(PROMINI)
ISR(USART_RX_vect_custom) {
  uint8_t c = UDR0;
  store_uart_in_buf(c, 0);
}
#endif

#if defined(PROMICRO)
ISR(USART1_RX_vect_custom) {
  uint8_t c = UDR1;
  store_uart_in_buf(c, 1);
}
#endif

#if defined(MEGA)
ISR(USART0_RX_vect_custom) {
  uint8_t c = UDR0;
  store_uart_in_buf(c, 0);
}

ISR(USART1_RX_vect_custom) {
  uint8_t c = UDR1;
  store_uart_in_buf(c, 1);
}

ISR(USART2_RX_vect_custom) {
  uint8_t c = UDR2;
  store_uart_in_buf(c, 2);
}

ISR(USART3_RX_vect_custom) {
  uint8_t c = UDR3;
  store_uart_in_buf(c, 3);
}
#endif

uint8_t SerialRead(uint8_t port) {
  uint8_t t = serialTailRX[port];
  if (serialHeadRX[port] == t) return 0;  // No hay datos
  if (++t >= RX_BUFFER_SIZE) t = 0;
  serialTailRX[port] = t;
  return serialBufferRX[t][port];
}

uint8_t SerialAvailable(uint8_t port) {
  return (uint8_t)(serialHeadRX[port] - serialTailRX[port]);
}

// SerialSerialize: Envia el byte serializado para un puerto específico
void SerialSerialize(uint8_t port, uint8_t a) {
  // Serializa y agrega el dato al buffer de transmisión
  uint8_t h = serialHeadTX[port];
  if (++h >= TX_BUFFER_SIZE) h = 0;
  while (h == serialTailTX[port]);  // Espera si el buffer está lleno

  serialBufferTX[h][port] = a;  // Almacena el dato serializado
  serialHeadTX[port] = h;

  // Habilitar la transmisión UART según el puerto
  UartSendData(port);
}

// SerialUsedTXBuff: Devuelve el número de bytes usados en el buffer de transmisión
uint8_t SerialUsedTXBuff(uint8_t port) {
  // Si el buffer está vacío, devuelve 0
  return (uint8_t)(serialHeadTX[port] - serialTailTX[port]);
}


