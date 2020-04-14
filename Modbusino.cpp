/*
 * Copyright © 2011-2012 Stéphane Raimbault <stephane.raimbault@gmail.com>
 *
 * License ISC, see LICENSE for more details.
 *
 * This library implements the Modbus protocol.
 * http://libmodbus.org/
 *
 */

#include <inttypes.h>

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#include <pins_arduino.h>
#endif
#include "Modbusino.h"

#define _MODBUS_RTU_SLAVE 0
#define _MODBUS_RTU_FUNCTION 1
#define _MODBUS_RTU_PRESET_REQ_LENGTH 6
#define _MODBUS_RTU_PRESET_RSP_LENGTH 2

#define _MODBUS_RTU_CHECKSUM_LENGTH 2

/* Supported function codes */
#define _FC_READ_HOLDING_REGISTERS 0x03
#define _FC_WRITE_MULTIPLE_REGISTERS 0x10

/*
 * Read holding registers (fn code 3) has less bytes than write multiple registers (fn code 16)
 * Address (1 byte)
 * Function (1 byte) Read holding
 * 1st register to read (2 bytes)
 * Nr of registers to read (2 bytes)
 * CRC (2 bytes)
 */
#define _MODBUS_MSG_READ_HOLDING_BYTES 8
#define _MODBUS_MSG_WRITE_MULTIPLE_MIN_BYTES 11
#define _MODBUS_MSG_WRITE_MULTIPLE_LEN_POS 7 //Data bytes length byte
enum { _STEP_FUNCTION = 0x01, _STEP_META, _STEP_DATA };

Timer modbusMessageTimeoutTimer;

static uint16_t crc16(uint8_t *req, uint8_t req_length)
{
    uint8_t j;
    uint16_t crc;

    crc = 0xFFFF;
    while (req_length--) {
        crc = crc ^ *req++;
        for (j = 0; j < 8; j++) {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc = crc >> 1;
        }
    }

    return (crc << 8 | crc >> 8);
}

void uartCallback(uart_t* uart, uint32_t status) {
	if(status & _BV(UIFE)) {
		digitalWrite(RS485_RE_PIN, LOW);
	}
}

ModbusinoSlave::ModbusinoSlave(uint8_t slave, uint16_t *dataArray = nullptr, uint8_t arrLen=0)
{
    if (slave >= 0 & slave <= 247) {
        _slave = slave;
    }
    dataPtr = dataArray;
    dataRegLen = arrLen;
    modbusMessageTimeoutTimer.initializeMs(10,TimerDelegate(&ModbusinoSlave::clearBuffer, this));
}

ModbusinoSlave::~ModbusinoSlave() {
	modbusMessageTimeoutTimer.stop();
	dataPtr = nullptr;
	rxCallback = nullptr;
}

void ModbusinoSlave::setup(long baud)
{
    pinMode(RS485_RE_PIN, OUTPUT);
    digitalWrite(RS485_RE_PIN, !RS485_TX_LEVEL);

    Serial.begin(baud, SERIAL_8N1, SERIAL_FULL);

	Serial.onTransmitComplete(transmitCompleteCb);
	//Serial.setUartCallback(uartCallback);
	Serial.onDataReceived(StreamDataReceivedDelegate(&ModbusinoSlave::onData, this));
	clearBuffer();
}

void ModbusinoSlave::setRxCallback(void (*callback)(void)) {
	rxCallback = callback;
}

void transmitCompleteCb(HardwareSerial &) {
		digitalWrite(RS485_RE_PIN, !RS485_TX_LEVEL);
}

static int check_integrity(uint8_t *msg, uint8_t msg_length)
{
    uint16_t crc_calculated;
    uint16_t crc_received;

    if (msg_length < 2)
        return -1;

    crc_calculated = crc16(msg, msg_length - 2);
    crc_received = (msg[msg_length - 2] << 8) | msg[msg_length - 1];

    /* Check CRC of msg */
    if (crc_calculated == crc_received) {
        return msg_length;
    } else {
        return -1;
    }
}

static int build_response_basis(uint8_t slave, uint8_t function, uint8_t *rsp)
{
    rsp[0] = slave;
    rsp[1] = function;

    return _MODBUS_RTU_PRESET_RSP_LENGTH;
}

static void send_msg(uint8_t *msg, uint8_t msg_length)
{
	uint16_t crc = crc16(msg, msg_length);

	msg[msg_length++] = crc >> 8;
	msg[msg_length++] = crc & 0x00FF;

	// Set RE active, add a one-character delay before and after message
	digitalWrite(RS485_RE_PIN, RS485_TX_LEVEL);
	constexpr uint8_t NUL{0};
	msg[msg_length++] = NUL;
	Serial.write(msg, msg_length);
	// RE is set inactive by Serial transmit-complete callback
}

static uint8_t response_exception(uint8_t slave, uint8_t function,
                                  uint8_t exception_code, uint8_t *rsp)
{
    uint8_t rsp_length;

    rsp_length = build_response_basis(slave, function + 0x80, rsp);

    /* Positive exception code */
    rsp[rsp_length++] = exception_code;

    return rsp_length;
}

static void reply(uint16_t *tab_reg, uint16_t nb_reg, uint8_t *req,
                  uint8_t req_length, uint8_t _slave)
{
    uint8_t slave = req[_MODBUS_RTU_SLAVE];
    uint8_t function = req[_MODBUS_RTU_FUNCTION];
    uint16_t address =
        (req[_MODBUS_RTU_FUNCTION + 1] << 8) + req[_MODBUS_RTU_FUNCTION + 2];
    uint16_t nb =
        (req[_MODBUS_RTU_FUNCTION + 3] << 8) + req[_MODBUS_RTU_FUNCTION + 4];
    uint8_t rsp[_MODBUSINO_RTU_MAX_ADU_LENGTH];
    uint8_t rsp_length = 0;

    if (slave != _slave && slave != MODBUS_BROADCAST_ADDRESS) {
        return;
    }

    if (address + nb > nb_reg) {
        rsp_length = response_exception(
            slave, function, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp);
    } else {
        req_length -= _MODBUS_RTU_CHECKSUM_LENGTH;

        if (function == _FC_READ_HOLDING_REGISTERS) {
            uint16_t i;

            rsp_length = build_response_basis(slave, function, rsp);
            rsp[rsp_length++] = nb << 1;
            for (i = address; i < address + nb; i++) {
                rsp[rsp_length++] = tab_reg[i] >> 8;
                rsp[rsp_length++] = tab_reg[i] & 0xFF;
            }
        } else {
            uint16_t i, j;

            for (i = address, j = 6; i < address + nb; i++, j += 2) {
                /* 6 and 7 = first value */
                tab_reg[i] = (req[_MODBUS_RTU_FUNCTION + j] << 8)
                             + req[_MODBUS_RTU_FUNCTION + j + 1];
            }

            rsp_length = build_response_basis(slave, function, rsp);
            /* 4 to copy the address (2) and the no. of registers */
            memcpy(rsp + rsp_length, req + rsp_length, 4);
            rsp_length += 4;
        }
    }

    send_msg(rsp, rsp_length);
}

void ModbusinoSlave::onData(Stream &stream, char arrivedChar,
                            unsigned short availableCharsCount)
{
    int rc = 0;
    if(modbusMessageTimeoutTimer.isStarted()) {
    	modbusMessageTimeoutTimer.restart();
    } else {
    	modbusMessageTimeoutTimer.startOnce();
    }

    while (availableCharsCount) {
        if (req_index < (_MODBUSINO_RTU_MAX_ADU_LENGTH - 1)) {
            req[req_index] = stream.read();
            req_index++;
            availableCharsCount--;
        } else {
            // clear req, print error
        	availableCharsCount=0;
        }
    }
    debug_hex(INFO, "ADU", req, req_index);
    if (req_index < _MODBUS_MSG_READ_HOLDING_BYTES) {
        return;
    }

    if (req[_MODBUS_RTU_SLAVE] != _slave
        && req[_MODBUS_RTU_SLAVE != MODBUS_BROADCAST_ADDRESS]) {
    	clearBuffer();
    	return;
    }

    if(req[_MODBUS_RTU_FUNCTION] == _FC_READ_HOLDING_REGISTERS) {
    	if(req_index >= _MODBUS_MSG_READ_HOLDING_BYTES) {
    		rc = check_integrity(req, _MODBUS_MSG_READ_HOLDING_BYTES);
            if (rc > 0) {
            	if(dataPtr) {
            		reply(dataPtr, dataRegLen, req, rc, _slave);
            	}
            }
    		return;
    	}
    }

    if(req[_MODBUS_RTU_FUNCTION] == _FC_WRITE_MULTIPLE_REGISTERS) {
    	if(req_index >= (req[_MODBUS_MSG_WRITE_MULTIPLE_LEN_POS-1] + _MODBUS_MSG_WRITE_MULTIPLE_MIN_BYTES - 2)) {
    		rc = check_integrity(req, req[_MODBUS_MSG_WRITE_MULTIPLE_LEN_POS-1] + _MODBUS_MSG_WRITE_MULTIPLE_MIN_BYTES - 2);
            if (rc > 0) {
            	if(dataPtr) {
            		reply(dataPtr, dataRegLen, req, rc, _slave);
            		if(rxCallback) {
            			rxCallback();
            		}
            	}
            }
    		return;
    	}
    }
}

void ModbusinoSlave::clearBuffer() {
	req_index = 0;
	memset(req, 0,_MODBUSINO_RTU_MAX_ADU_LENGTH);
}
