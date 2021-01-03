/*
 * Copyright (c) 2021 by Vadim Kulakov vad7@yahoo.com, vad711
  *
 * This file is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3.0 of the License, or (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
 * GNU General Public License for more details.
 */
#define VERSION F("1.00")

#include "Arduino.h"
#include <avr/wdt.h>
#include <util/atomic.h>
#include <Wire.h>
extern "C" {
	#include "utility/twi.h"
}

#define LED_PD						LED_BUILTIN
#define I2C_FREQ					2500
#define BMS_QTY_MAX					32
#define BMS_SERIAL 					Serial	// if not used - omit
#define BMS_SERIAL_RATE				9600
#define MAIN_LOOP_PERIOD			1		// msec
#define BMS_NO_TEMP					255
const uint8_t BMS_Cmd_Request[] PROGMEM = { 0x55, 0xAA, 0x01, 0xFF, 0x00, 0x00, 0xFF };

//#define MICROART_BMS_READWRITE				// Include code for Microart BMS
#define DEBUG_TO_SERIAL				9600
//#define DebugSerial 				Serial  // when active - UART BMS does not used

#ifdef DEBUG_TO_SERIAL
#ifndef DebugSerial
#include <SoftwareSerial.h>
SoftwareSerial DebugSerial(2, 3); // RX, TX
#else
#undef BMS_SERIAL
#endif
#endif

#ifdef DEBUG_TO_SERIAL
#define DEBUG(s) DebugSerial.print(s)
#define DEBUGN(s) DebugSerial.println(s)
#define DEBUG2(s) { if(bitRead(flags, f_DebugFull)) DebugSerial.print(s); }
#define DEBUG2N(s) { if(bitRead(flags, f_DebugFull)) DebugSerial.println(s); }
char	debug_read_buffer[64];
uint8_t debug_read_idx = 0;
const char dbg_temp[] PROGMEM = "temp";
const char dbg_cells[] PROGMEM = "cells";
const char dbg_period[] PROGMEM = "period";
const char dbg_debug[] PROGMEM = "debug";
const char dbg_I2C_WRITE_BMS[] PROGMEM = "I2C_WRITE_BMS";
const char dbg_I2C_READ_BMS[] PROGMEM = "I2C_READ_BMS";
#else
#define DEBUG(s)
#define DEBUGN(s)
#define DEBUG2(s)
#define DEBUG2N(s)
#endif

struct WORK {
	uint8_t  bms_qty;
	uint8_t  mode;
	uint32_t UART_read_period;		// ms
} work;

struct _EEPROM {
	WORK    work;
} __attribute__ ((packed));

struct _EEPROM EEMEM EEPROM;

enum {
	f_BMS_Ready = 0,
	f_DebugFull
};
uint8_t  flags = 0;					// f_*
uint16_t bms[BMS_QTY_MAX];			// V, hundreds
uint8_t  bms_Q[BMS_QTY_MAX];			// %
uint8_t  bms_idx = 0;
uint8_t  bms_idx_prev = 0;
uint16_t bms_min = 0;				// V, hundreds
uint16_t bms_full = 0;				// V, hundreds
uint8_t  map_mode = 0;
uint8_t  temp = BMS_NO_TEMP;		// C, +50
uint8_t  crc;
uint8_t  error = 0;
uint8_t  read_buffer[74];
uint8_t  read_idx = 0;
uint8_t  i2c_receive[32];
uint8_t  i2c_receive_idx = 0;

// Called in delay()
void yield(void)
{
	sleep_cpu();
	wdt_reset();
}

void Delay100ms(uint8_t ms) {
	while(ms-- > 0) {
		_delay_ms(100); wdt_reset();
	}
}

void FlashLED(uint8_t num, uint8_t ton, uint8_t toff) {
	while (num-- > 0) {
		digitalWrite(LED_PD, HIGH);
		Delay100ms(ton);
		digitalWrite(LED_PD, LOW);
		Delay100ms(toff);
	}
}

void i2c_set_slave_addr(uint8_t addr)
{
	TWAR = (addr << 1) | 1; // +broardcast addr(0)
}

void i2c_write(uint8_t d)
{
	crc += d;
	Wire.write(d);
}

void I2C_Response() {
	crc = 0;
	i2c_write(7);									// size
	i2c_write(5);									// op_code
	i2c_write(bms[bms_idx] & 0xFF);					// Ucell(low), V, hundreds
	i2c_write(bms[bms_idx] >> 8);					// Ucell(high), V, hundreds
	i2c_write(bms_idx == 0 ? temp : BMS_NO_TEMP);	// temp + 50, 255 - none
	i2c_write(bms_Q[bms_idx]);						// Q_Cell, %, I=(Q_Cell/100)*(Ucell/R), R=1
	i2c_write(0);									// prev err
	crc = 0 - crc;
	i2c_write(crc);
	if(++bms_idx == work.bms_qty) bms_idx = 0;
	i2c_set_slave_addr(bms_idx + 1);
}

void I2C_Receive(int howMany) {
	(void)howMany;  // unused
	if(i2c_receive_idx >= sizeof(i2c_receive)) return;
	while(Wire.available()) {
		i2c_receive[i2c_receive_idx++] = Wire.read();
		if(i2c_receive_idx >= sizeof(i2c_receive)) break;
	}
}

#ifdef DEBUG_TO_SERIAL
#ifdef MICROART_BMS_READWRITE
void Show_I2C_error(uint8_t err)
{
	DEBUG(F(" ERROR "));
	if(err == 1) DEBUG(F("LEN"));
	else if(err == 2) DEBUG(F("ADDR NACK"));
	else if(err == 3) DEBUG(F("DATA NACK"));
	else if(err == 4) DEBUG(F("SEND"));
	else DEBUG(err);
}

bool Wait_Microart_BMS_Response(void)
{
	const uint8_t Microart_BMS_I2CCom_JobWR[] PROGMEM = { 5, 4, 132, 136, 19, 216 };
	DEBUGN(F("Wait all BMS response for 1 min... "));
	uint8_t ok = 0;
	uint32_t m = millis();
	while(millis() - m <= 60000UL) {
		Wire.beginTransmission(0); // Broadcast
		for(uint8_t i = 0; i < sizeof(Microart_BMS_I2CCom_JobWR); i++) Wire.write(pgm_read_byte(BMS_Cmd_Request[i]));
		uint8_t err = Wire.endTransmission();
		if(err) {
			DEBUG(F("Broadcast: "));
			Show_I2C_error(err);
			DEBUGN();
			//return false;
		}
		delay(20);
		for(uint8_t bms = 1; bms <= work.bms_qty; bms++) {
			err = Wire.requestFrom(bms, (uint8_t) 8);
			if(err != 8) {
				DEBUG(F("ERROR REQ BMS-")); DEBUG(bms);
				DEBUG(F(". "));	DEBUGN(err);
				delay(40);
				continue;
			}
			uint8_t j = 0;
			crc = 0;
			while(Wire.available() && j < sizeof(debug_read_buffer)) {
				uint8_t b = Wire.read();
				crc += b;
				debug_read_buffer[j++] = b;
				DEBUG2(b); DEBUG2(' ');
			}
			if(crc != 0) {
				DEBUG(F("CRC ERROR BMS-")); DEBUGN(bms);
			} else ok += 1;
			delay(20);
		}
		if(ok == work.bms_qty) {
			DEBUG(F("Scan OK. Time: "));
			DEBUGN(millis() - m);
			return true;
		}
	}
	DEBUG(F("Failed! Max success attempt: ")); DEBUGN(ok);
	return false;
}
#endif

void DebugSerial_read(void)
{
	while(DebugSerial.available()) {
		int16_t r = DebugSerial.read();
		if(r == -1) break;
		debug_read_buffer[debug_read_idx++] = r;
		if(r == '\r' || debug_read_idx == sizeof(debug_read_buffer)-1) {
			debug_read_buffer[debug_read_idx-1] = '\0';
			debug_read_idx = 0;
			char *p = strchr(debug_read_buffer, '=');
			if(p == NULL) break;
			*p = '\0';
			DEBUG(F("CFG: ")); DEBUG(debug_read_buffer); DEBUG('=');
			uint16_t d = strtol(p + 1, NULL, 0);
			if(strncmp_P(debug_read_buffer, dbg_temp, sizeof(dbg_temp)-1) == 0) {
				temp = d + 50;
				DEBUG(d);
			} else if(strncmp_P(debug_read_buffer, dbg_cells, sizeof(dbg_cells)-1) == 0) {
				work.bms_qty = d;
				eeprom_update_block(&work, &EEPROM.work, sizeof(EEPROM.work));
				DEBUG(d);
			} else if(strncmp_P(debug_read_buffer, dbg_period, sizeof(dbg_period)-1) == 0) {
				work.UART_read_period = d;
				eeprom_update_block(&work, &EEPROM.work, sizeof(EEPROM.work));
				DEBUG(d);
			} else if(strncmp_P(debug_read_buffer, dbg_debug, sizeof(dbg_debug)-1) == 0) {
				bitWrite(flags, f_DebugFull, d);
				DEBUG(d);
#ifdef MICROART_BMS_READWRITE
			} else if(strncmp_P(debug_read_buffer, dbg_I2C_WRITE_BMS, sizeof(dbg_I2C_WRITE_BMS)-1) == 0) { // I2C_WRITE_BMSa=x -> a - address, x - byte
				uint8_t addr = strtol(debug_read_buffer + sizeof(dbg_I2C_WRITE_BMS)-1, NULL, 0);
				DEBUG2(addr); DEBUG2(',');
				DEBUG(d); DEBUG(' ');
				if(!Wait_Microart_BMS_Response()) break;
				uint8_t i = 1;
				for(; i <= work.bms_qty; i++) {
					crc = 0;
					Wire.beginTransmission(i);
					i2c_write(5);									// size
					i2c_write(1);									// op_code
					i2c_write(addr & 0xFF);							// Address_low
					i2c_write(addr >> 8);							// Address_high
					i2c_write(d);									// value
					crc = 0 - crc;
					i2c_write(crc);
					uint8_t err = Wire.endTransmission();
					if(err) {
						DEBUG(F("BMS-")); DEBUG(i);
						Show_I2C_error(err);
						break;
					}
					delay(30);
				}
				if(i > work.bms_qty) DEBUG(F("OK"));
			} else if(strncmp_P(debug_read_buffer, dbg_I2C_READ_BMS, sizeof(dbg_I2C_READ_BMS)-1) == 0) { // I2C_READ_BMS=a -> a - address
				DEBUG(d); DEBUGN(':');
				if(!Wait_Microart_BMS_Response()) break;
				uint8_t i = 1;
				for(; i <= work.bms_qty; i++) {
					DEBUG(F("BMS-")); DEBUG(i); DEBUG(F(": "));
					crc = 0;
					Wire.beginTransmission(i);
					i2c_write(5);									// size
					i2c_write(2);									// op_code
					i2c_write(d & 0xFF);							// Address_low
					i2c_write(d >> 8);								// Address_high
					i2c_write(1);									// len
					crc = 0 - crc;
					i2c_write(crc);
					uint8_t err = Wire.endTransmission();
					if(err) {
						DEBUG(F("REQ"));
						Show_I2C_error(err);
						break;
					}
					delay(30);
					d = Wire.requestFrom(i, (uint8_t) 6);
					if(d != 6) {
						DEBUG(F("REQ2 ERROR LEN="));
						DEBUG(d);
						break;
					}
					uint8_t j = 0;
					crc = 0;
					while(Wire.available() && j < sizeof(debug_read_buffer)) {
						uint8_t b = Wire.read();
						crc += b;
						debug_read_buffer[j++] = b;
						DEBUG2(b); DEBUG2(' ');
					}
					if(crc != 0) {
						DEBUG(F("- CRC ERROR!"));
					} else {
						DEBUG2(F(": "));
						DEBUG(debug_read_buffer[4]);
					}
					DEBUGN();
					delay(30);
				}
#endif
			} else if((debug_read_buffer[0] | 0x20) == 'v') { // Vn=x, n={1..bms_qty}, n=0 - all
				if(d) {
					uint8_t i = strtol(debug_read_buffer + 1, NULL, 0);
					ATOMIC_BLOCK(ATOMIC_FORCEON) {
						if(i == 0) {
							for(; i < work.bms_qty; i++) {
								bms[i] = d;
								//if(bms_full && d > bms_full+1) d = bms_full+1;
							}
						} else if(--i < work.bms_qty) bms[i] = d;
					}
					if(!bitRead(flags, f_BMS_Ready)) {
						i2c_set_slave_addr(bms_idx + 1);
						bitSet(flags, f_BMS_Ready);
					}
					DEBUG(d);
				}
			} else if((debug_read_buffer[0] | 0x20) == 'q') { // Qn=x, n={1..bms_qty}
				uint8_t i = strtol(debug_read_buffer + 1, NULL, 0);
				if(--i < work.bms_qty) bms_Q[i] = d;
				DEBUG(d);
			} else DEBUG(F("Unknown!"));
			DEBUG('\n');
			break;
		}
	}
}
#endif

// JK-DZ11-B2A24S active balancer
void BMS_Serial_read(void)
{
#ifdef BMS_SERIAL
	while(BMS_SERIAL.available()) {
		int16_t r = BMS_SERIAL.read();
		if(r == -1) break;
		read_buffer[read_idx++] = r;
		if(read_idx == sizeof(read_buffer)) {
			read_idx = 0;
			if(read_buffer[0] != 0xEB || read_buffer[1] != 0x90) {
				DEBUGN(F("BMS: Header mismatch!"));
				error = 50;
				break;
			}
			uint8_t crc = 0;
			for(uint8_t i = 0; i < sizeof(read_buffer) - 1; i++) crc += read_buffer[i];
			if(crc != read_buffer[sizeof(read_buffer) - 1]) {
				DEBUGN(F("BMS: CRC Error!"));
				error = 50;
				break;
			}
#ifdef DEBUG_TO_SERIAL
			if(bitRead(flags, f_DebugFull)) {
				DEBUG(F("Balance: "));
				if(read_buffer[21]) DEBUGN(F("ON")); else DEBUGN(F("OFF"));
				DEBUG(F("Total, mV: ")); DEBUGN(*(uint16_t*)(read_buffer + 4));
				DEBUG(F("Status: ")); DebugSerial.println(read_buffer[21], HEX);
				DEBUG(F("Current, mA: ")); DEBUGN(*(uint16_t*)(read_buffer + 15));
				DEBUG(F("Temp, C: ")); DEBUGN(*(uint16_t*)(read_buffer + 71));
				DEBUG(F("Process: ")); DEBUG(read_buffer[8]); DEBUG(read_buffer[9]); DEBUG(read_buffer[10]); DEBUGN(read_buffer[11]);
			}
#endif
			for(uint8_t i = 0; i < work.bms_qty; i++) {
				uint16_t v = (read_buffer[23 + i] + 5) / 10; // 0.001 -> 0.01
				ATOMIC_BLOCK(ATOMIC_FORCEON) bms[i] = v;
			}
			temp = read_buffer[71] + 50;
			memset(bms_Q, 0, sizeof(bms_Q));
			uint8_t i = read_buffer[9];
			if(i < work.bms_qty) bms_Q[i] = 100UL * read_buffer[15] / read_buffer[23 + i]; // Q_Cell=100*I/(Ucell/R), R=1
			if(!bitRead(flags, f_BMS_Ready)) {
				i2c_set_slave_addr(bms_idx + 1);
				bitSet(flags, f_BMS_Ready);
			}
			break;
		}
	}
#endif
}

void setup()
{
	wdt_enable(WDTO_2S); // Enable WDT
	sleep_enable();
	PRR = (1<<PRSPI) | (1<<PRADC); // Power off: SPI, ADC
#ifdef BMS_SERIAL
	BMS_SERIAL.begin(BMS_SERIAL_RATE);
#endif
#ifdef DEBUG_TO_SERIAL
	DebugSerial.begin(DEBUG_TO_SERIAL);
	DEBUG(F("\nBMS gate to Microart, v")); DEBUGN(VERSION);
	DEBUGN(F("Copyright by Vadim Kulakov (c) 2021, vad7@yahoo.com"));
#endif
	uint8_t b = eeprom_read_byte((uint8_t*)&EEPROM.work.bms_qty);
	if(b == 0 || b > 32) { // init EEPROM
		memset(&work, 0, sizeof(work));
		work.bms_qty = 16;
		work.mode = 0;
		work.UART_read_period = 1000;
		eeprom_update_block(&work, &EEPROM.work, sizeof(EEPROM.work));
	}
	eeprom_read_block(&work, &EEPROM.work, sizeof(EEPROM.work));
#ifdef DEBUG_TO_SERIAL
	DEBUG(F("Cells: ")); DEBUGN(work.bms_qty);
	DEBUG(F("BMS read period, ms: ")); DEBUGN(work.UART_read_period);
	DEBUGN(F("Commands:"));
	DEBUG((const __FlashStringHelper*)dbg_debug); DEBUGN(F("=x"));
	DEBUG((const __FlashStringHelper*)dbg_period); DEBUGN(F("=x"));
	DEBUG((const __FlashStringHelper*)dbg_cells); DEBUGN(F("=x"));
	DEBUG((const __FlashStringHelper*)dbg_temp); DEBUGN(F("=x"));
	DEBUGN(F("Vn=x\nQn=x"));
#ifdef MICROART_BMS_READWRITE
	DEBUG((const __FlashStringHelper*)dbg_I2C_READ_BMS); DEBUGN(F("=addr"));
	DEBUG((const __FlashStringHelper*)dbg_I2C_WRITE_BMS); DEBUGN(F("addr=x"));
#endif
#endif
	//
	Wire.begin();
#if I2C_FREQ < 30500
	TWSR |= (1<<TWPS1) | (1<<TWPS0); // Prescaler = 64
	TWBR = ((F_CPU / I2C_FREQ) - 16) / (2 * 64);
#else
	Wire.setClock(I2C_FREQ);
#endif
	Wire.onRequest(I2C_Response); // register event
	Wire.onReceive(I2C_Receive); // register event
	i2c_set_slave_addr(0);
	pinMode(LED_PD, OUTPUT);
	FlashLED(4, 1, 1);
}

void loop()
{
	wdt_reset(); sleep_cpu();
#ifdef DEBUG_TO_SERIAL
	if(bitRead(flags, f_DebugFull) && bms_idx_prev != bms_idx) {
		DEBUG(F("I2C_R_"));
		DEBUG(bms_idx_prev + 1);
		DEBUG(F("->"));
		DEBUGN(bms[bms_idx_prev]);
		bms_idx_prev = bms_idx;
	}
#endif
	static uint32_t led_flashing, bms_reading;
	uint32_t m = millis();
	if(m - led_flashing > (error == 0 ? 1500UL : 200UL)) {
		led_flashing = m;
		if(error) error--;
	    *portOutputRegister(digitalPinToPort(LED_PD)) ^= digitalPinToBitMask(LED_PD);
	}
	// Read from UART
	BMS_Serial_read();
	if(m - bms_reading > work.UART_read_period) {
		bms_reading = m;
#ifdef BMS_SERIAL
		read_idx = 0;	// reset read index
		for(uint8_t i = 0; i < sizeof(BMS_Cmd_Request); i++) BMS_SERIAL.write(pgm_read_byte(BMS_Cmd_Request[i]));
#endif
#ifdef DEBUG_TO_SERIAL
		DebugSerial_read();
#endif
	}
	// I2C slave receive
	if(i2c_receive_idx && i2c_receive_idx > i2c_receive[0]) { // i2c write
		DEBUG2(F("I2C_W: "));
		if(i2c_receive[0] >= sizeof(i2c_receive)) {
			DEBUG2(F("LEN ERROR!"));
			i2c_receive_idx = 0;
		} else {
			uint8_t crc = 0;
			for(uint8_t i = 0; i <= i2c_receive[0]; i++) { // +CRC
				crc += i2c_receive[i];
				DEBUG2(i2c_receive[i]);
				DEBUG2(" ");
			}
			if(crc != 0) {
				error = 50;
				DEBUG2(F("- CRC ERROR!"));
			} else if(i2c_receive[1] == 4) { // Broadcast I2CCom_JobWR
				bms_min = i2c_receive[2] + 200;
				bms_full = i2c_receive[3] + 200;
				map_mode = i2c_receive[4];
			}
			if(i2c_receive_idx > i2c_receive[0] + 1) {
				memcpy(i2c_receive, i2c_receive + i2c_receive[0] + 1, i2c_receive_idx -= i2c_receive[0] + 1);
			} else i2c_receive_idx = 0;
		}
		DEBUG2(F("\n"));
	}
	delay(MAIN_LOOP_PERIOD);
}


