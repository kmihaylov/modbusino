/*
 * Copyright © 2011 Stéphane Raimbault <stephane.raimbault@gmail.com>
 *
 * License ISC, see LICENSE for more details.

 * This library implements the Modbus protocol.
 * http://libmodbus.org/
 *
 */

#ifndef Modbusino_h
#define Modbusino_h

#include <inttypes.h>
#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
  #include <pins_arduino.h>
#endif

#define MODBUS_BROADCAST_ADDRESS 0

/* Protocol exceptions */
#define MODBUS_EXCEPTION_ILLEGAL_FUNCTION     1
#define MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS 2
#define MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE   3
#define MODBUS_INFORMATIVE_NOT_FOR_US   4
#define MODBUS_INFORMATIVE_RX_TIMEOUT   5

/* As reported in https://github.com/stephane/modbusino/issues/6, the code could
segfault for longer ADU */
#define _MODBUSINO_RTU_MAX_ADU_LENGTH 256

void transmitCompleteCb(HardwareSerial &);

class ModbusinoSlave {
public:
    ModbusinoSlave(uint8_t slave, uint16_t *, uint8_t);
    ~ModbusinoSlave();
    void setRxCallback(void (*callback)(void) = nullptr);
    void setup(long baud);
    int loop(uint16_t *tab_reg, uint16_t nb_reg);
    void onData(Stream& stream, char arrivedChar, unsigned short availableCharsCount);

private:
    int _slave;
    uint8_t req[_MODBUSINO_RTU_MAX_ADU_LENGTH] = {0};
    uint8_t req_index = 0;
    uint16_t *dataPtr = nullptr;
    uint8_t dataRegLen = 0;
    void (*rxCallback)(void) = nullptr;
    void clearBuffer(void);
};

#endif
