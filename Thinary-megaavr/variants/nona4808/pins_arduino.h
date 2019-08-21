/*
  pins_arduino.h - Pin definition functions for Arduino
  Part of Arduino - http://www.arduino.cc/

  Copyright (c) 2007 David A. Mellis

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA
*/

#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <avr/pgmspace.h>
#include "timers.h"

#define NUM_DIGITAL_PINS            25 // (13 on digital headers + 12 on analog headers)
#define NUM_ANALOG_INPUTS           12
#define NUM_RESERVED_PINS           1  // (RESET)
#define NUM_INTERNALLY_USED_PINS    1 // (2 x Chip select + 2 x UART + 4 x IO + LED_BUILTIN + 1 unused pin)
#define NUM_I2C_PINS                2  // (SDA / SCL)
#define NUM_SPI_PINS                3  // (MISO / MOSI / SCK)
#define NUM_TOTAL_FREE_PINS         (NUM_DIGITAL_PINS)
#define NUM_TOTAL_PINS              (NUM_DIGITAL_PINS + NUM_RESERVED_PINS)
#define ANALOG_INPUT_OFFSET         14

#define EXTERNAL_NUM_INTERRUPTS     48

#define digitalPinHasPWM(p)         ((p) == 2 || (p) == 3 || (p) == 4 || (p) == 20 || (p) == 21)

#define SPI_MUX       (PORTMUX_SPI0_ALT1_gc)
#define PIN_SPI_MISO  (12)
#define PIN_SPI_SCK   (13)
#define PIN_SPI_MOSI  (11)
#define PIN_SPI_SS    (10)

static const uint8_t SS   = PIN_SPI_SS;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

#define PIN_WIRE_SDA  (22)
#define PIN_WIRE_SCL  (23)

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

// Nano Evey Debug USART (not available on headers, only via the EDGB virtual COM port)
// USART2 on mega4808 (alternative pins)
// Mapped to HWSERIAL1 in Serial library
#define HWSERIAL1 (&USART2)
#define HWSERIAL1_DRE_VECTOR (USART2_DRE_vect)
#define HWSERIAL1_DRE_VECTOR_NUM (USART2_DRE_vect_num)
#define HWSERIAL1_RXC_VECTOR (USART2_RXC_vect)
#define HWSERIAL1_MUX (PORTMUX_USART2_DEFAULT_gc)
#define PIN_WIRE_HWSERIAL1_RX (0)
#define PIN_WIRE_HWSERIAL1_TX (1)

// Main USART available on Arduino header pins
// USART0 on mega4808 (alternative pins)
// Mapped to HWSERIAL0 in Serial library
#define HWSERIAL0 (&USART0)
#define HWSERIAL0_DRE_VECTOR (USART0_DRE_vect)
#define HWSERIAL0_DRE_VECTOR_NUM (USART0_DRE_vect_num)
#define HWSERIAL0_RXC_VECTOR (USART0_RXC_vect)
#define HWSERIAL0_MUX (PORTMUX_USART0_DEFAULT_gc)
#define PIN_WIRE_HWSERIAL0_RX (3)
#define PIN_WIRE_HWSERIAL0_TX (2)

#define HWSERIAL2_MUX (PORTMUX_USART0_NONE_gc)
#define HWSERIAL3_MUX (PORTMUX_USART1_NONE_gc)

#define TWI_MUX (PORTMUX_TWI0_ALT1_gc) //PORTMUX_TWI0_ALT1_gc

#define MUX_SPI (SPI_MUX)
#define SPI_INTERFACES_COUNT 1

#define LED_BUILTIN   (13)

#define PIN_A0   (14) // PD0 / AIN0
#define PIN_A1   (15) // PD1 / AIN1
#define PIN_A2   (16) // PD2 / AIN2
#define PIN_A3   (17) // PD3 / AIN3
#define PIN_A4   (18) // PF2 / AIN12
#define PIN_A5   (19) // PF3 / AIN13
#define PIN_A6   (20) // PF4 / AIN14
#define PIN_A7   (21) // PF5 / AIN15

static const uint8_t A0 = PIN_A0;
static const uint8_t A1 = PIN_A1;
static const uint8_t A2 = PIN_A2;
static const uint8_t A3 = PIN_A3;
static const uint8_t A4 = PIN_A4;
static const uint8_t A5 = PIN_A5;
static const uint8_t A6 = PIN_A6;
static const uint8_t A7 = PIN_A7;

#define PINS_COUNT    (40u)

#ifdef ARDUINO_MAIN

// On the Arduino board, digital pins are also used
// for the analog output (software PWM).  Analog input
// pins are a separate set.

// ATMEGA4808 / ARDUINO
//
//                 (4)  (3)  (2)                (RST) (A7)
//                 PA2  PA1  PA0  GND  VDD UPDI  PF6  PF5
//
//                 32   31   30   29   28   27   26   25
//              + ____ ____ ____ ____ ____ ____ ____ ____  +
//   (5) PA3   1|                                         |24  PF4 (A6)
//   (6) PA4   2|                                         |23  PF3 (A5)
//   (7) PA5   3|                                         |22  PF2 (A4)
//   (8) PA6   4|                                         |21  PF1 (RX)
//   (9) PA7   5|                32pin QFN                |20  PF0 (TX)
//  (11) PC0   6|                                         |19  GND
//  (12) PC1   7|                                         |18  AVDD
//  (13) PC2   8|                                         |17  PD7 (25)
//               + ____ ____ ____ ____ ____ ____ ____ ____ +
//                   9   10   11   12   13   14   15   16 
//
//                  PC3  PD0  PD1  PD2  PD3  PD4  PD5  PD6
//                 (10) (A0) (A1) (A2) (A3) (22) (23) (24)

//

const uint8_t PROGMEM digital_pin_to_port[] = {
  PF, // 0 PF0/USART2_Rx
  PF, // 1 PF1/USART2_Tx
  PA, // 2 PA0
  PA, // 3 PA1
  PA, // 4 PA2
  PA, // 5 PA3
  PA, // 6 PA4
  PA, // 7 PA5
  PA, // 8 PA6
  PA, // 9 PA7
  PC, // 10 PC3
  PC, // 11 PC0
  PC, // 12 PC1
  PC, // 13 PC2
  PD, // 14 PD0/AI0
  PD, // 15 PD1/AI1
  PD, // 16 PD2/AI2
  PD, // 17 PD3/AI3
  PF, // 18 PF2/AI12/TWI_SDA
  PF, // 19 PF3/AI13/TWI_SCL
  PF, // 20 PF4/AI14
  PF, // 21 PF5/AI15
  PD, // 22 PD4
  PD, // 23 PD5
  PD, // 24 PD6
  PD, // 25 PD7
};

/* Use this for accessing PINnCTRL register */
const uint8_t PROGMEM digital_pin_to_bit_position[] = {
  PIN1_bp,  // 0 PF1/USART2_Rx
  PIN0_bp,  // 1 PF0/USART2_Tx
  PIN0_bp,  // 2 PA0
  PIN1_bp,  // 3 PA1
  PIN2_bp,  // 4 PA2
  PIN3_bp,  // 5 PA3
  PIN4_bp,  // 6 PA4
  PIN5_bp,  // 7 PA5
  PIN6_bp,  // 8 PA6
  PIN7_bp,  // 9 PA7
  PIN3_bp,  // 10 PC3
  PIN0_bp,  // 11 PC0
  PIN1_bp,  // 12 PC1
  PIN2_bp,  // 13 PC2
  PIN0_bp,  // 14 PD0/AI0
  PIN1_bp,  // 15 PD1/AI1
  PIN2_bp,  // 16 PD2/AI2
  PIN3_bp,  // 17 PD3/AI3
  PIN2_bp,  // 18 PF2/AI12/TWI_SDA
  PIN3_bp,  // 19 PF3/AI13/TWI_SCL
  PIN4_bp,  // 20 PF4/AI14
  PIN5_bp,  // 21 PF5/AI15
  PIN4_bp,  // 22 PD4
  PIN5_bp,  // 23 PD5
  PIN6_bp,  // 24 PD6
  PIN7_bp,  // 25 PD7
};

/* Use this for accessing PINnCTRL register */
const uint8_t PROGMEM digital_pin_to_bit_mask[] = {
  PIN1_bm,  // 0 PF1/USART2_Rx
  PIN0_bm,  // 1 PF0/USART2_Tx
  PIN0_bm,  // 2 PA0
  PIN1_bm,  // 3 PA1
  PIN2_bm,  // 4 PA2
  PIN3_bm,  // 5 PA3
  PIN4_bm,  // 6 PA4
  PIN5_bm,  // 7 PA5
  PIN6_bm,  // 8 PA6
  PIN7_bm,  // 9 PA7
  PIN3_bm,  // 10 PC3
  PIN0_bm,  // 11 PC0
  PIN1_bm,  // 12 PC1
  PIN2_bm,  // 13 PC2
  PIN0_bm,  // 14 PD0/AI0
  PIN1_bm,  // 15 PD1/AI1
  PIN2_bm,  // 16 PD2/AI2
  PIN3_bm,  // 17 PD3/AI3
  PIN2_bm,  // 18 PF2/AI12/TWI_SDA
  PIN3_bm,  // 19 PF3/AI13/TWI_SCL
  PIN4_bm,  // 20 PF4/AI14
  PIN5_bm,  // 21 PF5/AI15
  PIN4_bm,  // 22 PD4
  PIN5_bm,  // 23 PD5
  PIN6_bm,  // 24 PD6
  PIN7_bm,  // 25 PD7
};

const uint8_t PROGMEM digital_pin_to_timer[] = {
  NOT_ON_TIMER,  // 0 PF1/USART2_Rx
  NOT_ON_TIMER,  // 1 PF0/USART2_Tx
  NOT_ON_TIMER,  // 2 PA0
  NOT_ON_TIMER,  // 3 PA1
  NOT_ON_TIMER,  // 4 PA2/TWI_SDA
  NOT_ON_TIMER,  // 5 PA3/TWI_SCL
  NOT_ON_TIMER,  // 6 PA4
  NOT_ON_TIMER,  // 7 PA5
  NOT_ON_TIMER,  // 8 PA6
  NOT_ON_TIMER,  // 9 PA7
  NOT_ON_TIMER,  // 10 PC3/SS
  NOT_ON_TIMER,  // 11 PC0/MOSI
  NOT_ON_TIMER,  // 12 PC1/MISO
  NOT_ON_TIMER,  // 13 PC2/SCK
  TIMERA0,       // 14 PD0/AI0
  TIMERA0,       // 15 PD1/AI1
  TIMERA0,       // 16 PD2/AI2
  NOT_ON_TIMER,  // 17 PD3/AI3
  NOT_ON_TIMER,  // 18 PF2/AI12/TWI_SDA
  NOT_ON_TIMER,  // 19 PF3/AI13/TWI_SCL
  TIMERB0,       // 20 PF4/AI14
  TIMERB1,       // 21 PF5/AI15
  NOT_ON_TIMER,  // 22 PD4
  NOT_ON_TIMER,  // 23 PD5
  NOT_ON_TIMER,  // 24 PD6
  NOT_ON_TIMER,  // 25 PD7
};

const uint8_t PROGMEM analog_pin_to_channel[] = {
  0,
  1,
  2,
  3,
  12,
  13,
  14,
  15
};

#endif

extern const uint8_t analog_pin_to_channel[];
#define digitalPinToAnalogInput(p)  ((p < ANALOG_INPUT_OFFSET) ? pgm_read_byte(analog_pin_to_channel + p) : pgm_read_byte(analog_pin_to_channel + p - ANALOG_INPUT_OFFSET) )

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_MONITOR       Serial
#define SERIAL_PORT_HARDWARE      Serial1
#define SERIAL_PORT_USBVIRTUAL    Serial

#endif
