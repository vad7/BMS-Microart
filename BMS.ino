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

RJ-11 6P6C:
1 - BMS_DISCHARGE / I2C_SLC_buf_iso (brown-white)
2 - GND								(brown)
3 - TEPM							(blue-white)
4 - BMS_CHARGE / I2C_SDA_buf_iso�	(blue)
5 - +5v_iso							(orange)
6 - not connected on back side, on front side: +12V if enabled (orange-white)

*/

#define VERSION F("1.0")

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
#define WATCHDOG_NO_CONN			30UL	// sec
#define BMS_PAUSE_BETWEEN_READS		150UL	// msec
#define BMS_CHANGE_DELTA_PAUSE_MIN  1800	// sec
#define BMS_CHANGE_DELTA_EQUALIZER  60		// attempts (* ~1 sec)
const uint8_t BMS_Cmd_Request[] PROGMEM = { 0x55, 0xAA, 0x01, 0xFF, 0x00, 0x00, 0xFF };
const uint8_t BMS_Cmd_ChangeDelta[] PROGMEM = { 0x55, 0xAA, 0x01, 0xF2 };

//#define MICROART_BMS_READWRITE				// Include code for Microart BMS
#define DEBUG_TO_SERIAL				115200  // 19200
#define DEBUG_READ_PERIOD			1000UL	// ms
//#define DebugSerial 				Serial  // when active - UART BMS does not used

#ifdef DEBUG_TO_SERIAL
#ifndef DebugSerial
#include <SoftwareSerial.h>
SoftwareSerial DebugSerial(3, 2); // RX = D3, TX = D2
//#include "libraries\AltSoftSerial\AltSoftSerial.h"
//AltSoftSerial DebugSerial;			// pins: RX = D8, TX = D9, unusable = D10
#else
#undef BMS_SERIAL
#endif
#endif

#ifdef DEBUG_TO_SERIAL
#define DEBUG(s) DebugSerial.print(s)
#define DEBUGH(s) DebugSerial.print(s, 16)
#define DEBUGN(s) DebugSerial.println(s)
#define DEBUGIF(d,s) { if(debug >= d) DebugSerial.print(s); }
#define DEBUGIFN(d,s) { if(debug >= d) DebugSerial.println(s); }
char	debug_read_buffer[64];
uint8_t debug_read_idx = 0;
const char dbg_debug[] PROGMEM = "dbg";
const char dbg_cells[] PROGMEM = "cells";
const char dbg_period[] PROGMEM = "period";
const char dbg_round[] PROGMEM = "Vround";
const char dbg_correct[] PROGMEM = "Vcorr";
const char dbg_options[] PROGMEM = "options";
const char dbg_temp_correct[] PROGMEM = "tempcorr";
const char dbg_delta_default[] PROGMEM = "deltadef";
const char dbg_delta_pause[] PROGMEM = "deltapause";
const char dbg_watchdog[] PROGMEM = "watchdog";
const char dbg_vmaxhyst[] PROGMEM = "Vmaxhyst";
const char dbg_seterr[] PROGMEM = "ERR";
const char dbg_I2C_WRITE_BMS[] PROGMEM = "I2C_WRITE_BMS";
const char dbg_I2C_READ_BMS[] PROGMEM = "I2C_READ_BMS";
const char dbg_temp[] PROGMEM = "temp";
const char dbg_delta_change_pause[] PROGMEM = "dchgpause";
#else
#define DEBUG(s)
#define DEBUGN(s)
#define DEBUGIF(d,s)
#define DEBUGIFN(d,s)
#endif

enum {
	round_true = 0,
	round_cut,
	round_up
};

enum { // options bits
	o_average = 0
};

struct WORK {
	uint8_t  bms_qty;
	uint8_t  options;
	uint32_t UART_read_period;		// ms
	uint8_t  round;					// round_*
	int16_t  V_correct;				// mV
	int8_t   temp_correct;			// mV
	uint8_t  watchdog;				// reboot if no data over: 1 - I2C, 2 - BMS
	int16_t  Vmaxhyst;				// *10mV
	uint16_t BalansDeltaPause;		// sec
	uint16_t BalansDeltaDefault;	// Default balans delta voltage, mV
	uint16_t BalansDelta[2];		// mV
	uint8_t  BalansDeltaI[2];		// Current threshold
} work;

struct _EEPROM {
	WORK    work;
} __attribute__ ((packed));

struct _EEPROM EEMEM EEPROM;

enum {
	ERR_BMS_Read = 1,
	ERR_BMS_Config,
	ERR_BMS_Hardware
};
enum {
	f_BMS_Ready = 0
};
uint8_t  flags = 0;					// f_*
int8_t   debug = 0;					// 0 - off, 1 - on, 2 - detailed dump, 3 - full dump, 4 - BMS full
uint16_t bms[BMS_QTY_MAX];			// *10mV
uint8_t  bms_Q[BMS_QTY_MAX];		// %
uint8_t  bms_idx = 0;
uint8_t  bms_idx_prev = 0;
uint32_t bms_loop_time;
uint32_t watchdog_timer = 0;
uint8_t  watchdog_I2C = 0;
uint8_t  watchdog_BMS = 0;
int16_t  bms_min = 0;				// *10mV
int16_t  bms_full = 0;				// *10mV
bool     bms_need_read = true;
uint8_t  map_mode = 0;
uint8_t  temp = BMS_NO_TEMP;		// C, +50
uint8_t  crc;
uint8_t  last_error = 0;
uint8_t  error_alarm_time = 0;
uint8_t  read_buffer[74];
uint8_t  read_idx = 0;
uint8_t  i2c_receive[32];
uint8_t  i2c_receive_idx = 0;
uint32_t bms_last_read_time = 0;
uint16_t delta_active = 0;			// mV
uint16_t delta_new = 0;				// mV
uint16_t delta_next = 0;
uint16_t delta_change_pause = 0;  	// sec
uint8_t  delta_change_equalizer = 0; // attempts
uint8_t  debug_info	= 0b0011;		// b0 - I2C_W1, b1 - I2C_W2

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
	i2c_write(bms_idx == 0 ? last_error : 0);		// prev err
	crc = 0 - crc;
	i2c_write(crc);
	if(++bms_idx == work.bms_qty) bms_idx = 0;
	i2c_set_slave_addr(bms_idx + 1);
	watchdog_I2C = 0;
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
		for(uint8_t i = 0; i < sizeof(Microart_BMS_I2CCom_JobWR); i++) Wire.write(pgm_read_byte(&BMS_Cmd_Request[i]));
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
				DEBUGIF(2,b); DEBUGIF(2,' ');
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
		int r = DebugSerial.read();
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
			if(strncmp_P(debug_read_buffer, dbg_temp_correct, sizeof(dbg_temp_correct)-1) == 0) {
				work.temp_correct = d;
				DEBUG(work.temp_correct);
				eeprom_update_block(&work, &EEPROM.work, sizeof(EEPROM.work));
			} else if(strncmp_P(debug_read_buffer, dbg_temp, sizeof(dbg_temp)-1) == 0) {
				temp = d + 50;
				DEBUG(d);
			} else if(strncmp_P(debug_read_buffer, dbg_cells, sizeof(dbg_cells)-1) == 0) {
				if(d < 2) d = 2;
				work.bms_qty = d;
				DEBUG(d);
				eeprom_update_block(&work, &EEPROM.work, sizeof(EEPROM.work));
			} else if(strncmp_P(debug_read_buffer, dbg_period, sizeof(dbg_period)-1) == 0) {
				work.UART_read_period = d;
				DEBUG(d);
				eeprom_update_block(&work, &EEPROM.work, sizeof(EEPROM.work));
			} else if(strncmp_P(debug_read_buffer, dbg_round, sizeof(dbg_round)-1) == 0) {
				work.round = d;
				DEBUGN(work.round == round_true ? "5/4" : work.round == round_cut ? "cut" : work.round == round_up ? "up" : "?");
				eeprom_update_block(&work, &EEPROM.work, sizeof(EEPROM.work));
			} else if(strncmp_P(debug_read_buffer, dbg_correct, sizeof(dbg_correct)-1) == 0) {
				work.V_correct = d;
				DEBUG(d);
				eeprom_update_block(&work, &EEPROM.work, sizeof(EEPROM.work));
			} else if(strncmp_P(debug_read_buffer, dbg_vmaxhyst, sizeof(dbg_vmaxhyst)-1) == 0) {
				work.Vmaxhyst = d;
				DEBUG(d);
				eeprom_update_block(&work, &EEPROM.work, sizeof(EEPROM.work));
			} else if(strncmp_P(debug_read_buffer, dbg_delta_default, sizeof(dbg_delta_default)-1) == 0) {
				work.BalansDeltaDefault = d;
				DEBUG(d);
				eeprom_update_block(&work, &EEPROM.work, sizeof(EEPROM.work));
			} else if(strncmp_P(debug_read_buffer, dbg_delta_pause, sizeof(dbg_delta_pause)-1) == 0) {
				work.BalansDeltaPause = d;
				DEBUG(d);
				eeprom_update_block(&work, &EEPROM.work, sizeof(EEPROM.work));
			} else if(strncmp_P(debug_read_buffer, dbg_watchdog, sizeof(dbg_watchdog)-1) == 0) {
				work.watchdog = d;
				DEBUG(d);
				eeprom_update_block(&work, &EEPROM.work, sizeof(EEPROM.work));
			} else if(strncmp_P(debug_read_buffer, dbg_options, sizeof(dbg_options)-1) == 0) {
				work.options = d;
				DEBUG(d);
				eeprom_update_block(&work, &EEPROM.work, sizeof(EEPROM.work));
			} else if(strncmp_P(debug_read_buffer, dbg_debug, sizeof(dbg_debug)-1) == 0) {
				debug = d;
				DEBUG(d);
			} else if(strncmp_P(debug_read_buffer, dbg_seterr, sizeof(dbg_seterr)-1) == 0) {
				last_error = d;
				DEBUG(d);
			} else if(strncmp_P(debug_read_buffer, dbg_delta_change_pause, sizeof(dbg_delta_change_pause)-1) == 0) {
				delta_change_pause = d;
				DEBUG(d);
#ifdef MICROART_BMS_READWRITE
			} else if(strncmp_P(debug_read_buffer, dbg_I2C_WRITE_BMS, sizeof(dbg_I2C_WRITE_BMS)-1) == 0) { // I2C_WRITE_BMSa=x -> a - address, x - byte
				uint8_t addr = strtol(debug_read_buffer + sizeof(dbg_I2C_WRITE_BMS)-1, NULL, 0);
				DEBUGIF(2,addr); DEBUGIF(2,',');
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
						DEBUGIF(2,b); DEBUGIF(2,' ');
					}
					if(crc != 0) {
						DEBUG(F("- CRC ERROR!"));
					} else {
						DEBUGIF(2,F(": "));
						DEBUG(debug_read_buffer[4]);
					}
					DEBUGN();
					delay(30);
				}
#endif
			} else if(debug_read_buffer[1] >= '0' && debug_read_buffer[1] <= '3' && (!debug_read_buffer[2] || !debug_read_buffer[3])) {
				if((debug_read_buffer[0] | 0x20) == 'v') { // Vn=x, n={1..bms_qty}, n=0 - all
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
				} else if((debug_read_buffer[0] | 0x20) == 'd') { // Dn=x, n={0..delta array size}
					uint8_t i = strtol(debug_read_buffer + 1, NULL, 0);
					if(i >= sizeof(work.BalansDelta)/sizeof(work.BalansDelta[0])) DEBUG(F("MAX IDX!"));
					else work.BalansDelta[i] = d;
					DEBUG(d);
					eeprom_update_block(&work, &EEPROM.work, sizeof(EEPROM.work));
				} else if((debug_read_buffer[0] | 0x20) == 'i') { // In=x, n={0..delta array size}
					uint8_t i = strtol(debug_read_buffer + 1, NULL, 0);
					if(i >= sizeof(work.BalansDelta)/sizeof(work.BalansDelta[0])) DEBUG(F("MAX IDX!"));
					else work.BalansDeltaI[i] = d;
					DEBUG(d);
					eeprom_update_block(&work, &EEPROM.work, sizeof(EEPROM.work));
				} else DEBUG(F("ERR!"));
			} else DEBUG(F("ERR!"));
			DEBUG('\n');
			break;
		}
	}
}
#endif

// JK-DZ11-B2A24S active balancer
void BMS_Serial_read(void)
{
	while(BMS_SERIAL.available()) {
		int16_t r = BMS_SERIAL.read();
		if(r == -1) break;
		read_buffer[read_idx++] = r;
		if(read_idx == sizeof(read_buffer)) {
			last_error = 0;
			read_idx = 0;
			if(read_buffer[0] != 0xEB || read_buffer[1] != 0x90) {
				DEBUGIFN(1,F("BMS: Header mismatch!"));
				last_error = ERR_BMS_Read;
				error_alarm_time = 50;
				break;
			}
			if(debug == 4) DEBUG(F("BMS Answer: "));
			uint8_t crc = 0;
			for(uint8_t i = 0; i < sizeof(read_buffer) - 1; i++) {
				if(debug == 4) {
					DEBUG(i); DEBUG('='); DEBUGH(read_buffer[i]); DEBUG(' ');
				}
				crc += read_buffer[i];
			}
			if(debug == 4) DEBUG('\n');
			if(crc != read_buffer[sizeof(read_buffer) - 1]) {
				DEBUGIFN(1,F("BMS: CRC Error!"));
				last_error = ERR_BMS_Read;
				error_alarm_time = 50;
				break;
			}
			if(read_buffer[3] == 0xF2) { // Change delta voltage
				delta_active = read_buffer[4]*256 + read_buffer[5];
				if(debug) {
					DEBUG(F("New delta: ")); DEBUGN(delta_active);
				}
				delta_change_pause = 0;
				watchdog_BMS = 0;
				bms_last_read_time = millis();
			} else if(read_buffer[3] == 0xFF) { // Request answer
				if(read_buffer[12] & 0x03) {
					DEBUGIF(1,F("BMS Alarm: "));
					error_alarm_time = 50;
					if(read_buffer[12] & (1<<0)) { // cells num wrong
						last_error = ERR_BMS_Config;
						DEBUGIF(1,F("Cells_Num "));
					}
					if(read_buffer[12] & (1<<1)) { // wire resistance is too large
						last_error = ERR_BMS_Hardware;
						DEBUGIF(1,F("Wire_Resistance "));
					}
	//				if(read_buffer[12] & (1<<2)) { // battery overvoltage
	//					last_error = ERR_BMS_Hardware;
	//					DEBUG(F("Overvoltage"));
	//				}
					DEBUGIF(1,'\n');
				}
	#ifdef DEBUG_TO_SERIAL
				if(debug == 3) {
					DEBUG(F("BMS "));
					if(read_buffer[21]) DEBUG(F("ON")); else DEBUG(F("OFF"));
					uint16_t n = read_buffer[4]*256 + read_buffer[5];
					DEBUG(F(",V:")); DEBUG(n / 100); DEBUG('.'); n %= 100; if(n < 10) DEBUG('0'); DEBUG(n);
					DEBUG(F(",D(mV):")); DEBUG(read_buffer[13]*256 + read_buffer[14]);
					DEBUG(F(",Trg(mV):")); DEBUG(read_buffer[17]*256 + read_buffer[18]);
					DEBUG(F(",Bal(mA):")); DEBUG(read_buffer[15]*256 + read_buffer[16]);
					//DEBUG(F(",T(C):")); DEBUG(read_buffer[72]);
					//DEBUG(F(",W:")); DEBUG(read_buffer[8]); DEBUG(','); DEBUG(read_buffer[9]); DEBUG(','); DEBUG(read_buffer[10]); DEBUG(','); DEBUG(read_buffer[11]);
					DEBUG(F("\n"));
				}
	#endif
				if(read_buffer[8] != work.bms_qty) {
					DEBUGN(F("BMS: Cells num not equal setup!"));
					last_error = ERR_BMS_Config;
					error_alarm_time = 50;
					if(read_buffer[8] > BMS_QTY_MAX) break;
				}
				for(uint8_t i = 0; i < read_buffer[8]; i++) {
					if(i > work.bms_qty - 1) {
						ATOMIC_BLOCK(ATOMIC_FORCEON) bms[i] = 0;
						continue;
					}
					int16_t v = read_buffer[23 + i*2]*256 + read_buffer[24 + i*2];
					if(work.V_correct) {
						v += work.V_correct;
						if(v < 0) v = 0;
					}
					if(work.round == round_true) v += 5;
					else if(work.round == round_up) v += 9;
					v /= 10; // 0.001 -> 0.01
					if(work.Vmaxhyst && bms_full) {
						if(v >= bms_full && v < bms_full + work.Vmaxhyst) v = bms_full - 1;
					}
					if(bitRead(work.options, o_average)) {
						// Медианный фильтр
						static int16_t median1, median2;
						if(median1 == 0) median1 = median2 = v;
						int16_t median3 = v;
						if(median1 <= median2 && median1 <= median3) {
							v = median2 <= median3 ? median2 : median3;
						} else if(median2 <= median1 && median2 <= median3) {
							v = median1 <= median3 ? median1 : median3;
						} else {
							v = median1 <= median2 ? median1 : median2;
						}
						median1 = median2;
						median2 = median3;
						//
					}
					ATOMIC_BLOCK(ATOMIC_FORCEON) bms[i] = v;
				}
				temp = read_buffer[72] + 50 + work.temp_correct;
				delta_active = read_buffer[17]*256 + read_buffer[18];
				memset(bms_Q, 0, sizeof(bms_Q));
				uint8_t i = read_buffer[9];
				if(i < work.bms_qty && read_buffer[11]) bms_Q[i] = 100UL * (read_buffer[15]*256 + read_buffer[16]) / (read_buffer[23 + i*2]*256 + read_buffer[24 + i*2]); // Q_Cell=100*I/(Ucell/R), R=1
				if(!bitRead(flags, f_BMS_Ready)) {
					i2c_set_slave_addr(bms_idx + 1);
					bitSet(flags, f_BMS_Ready);
					bms_loop_time = millis();
				}
				watchdog_BMS = 0;
				bms_last_read_time = millis();
			} else if(debug) {
				DEBUG(F("BMS: Wrong response code: ")); DEBUGN(read_buffer[3]);
			}
			break;
		}
	}
}

void setup()
{
	wdt_enable(WDTO_2S); // Enable WDT
	sleep_enable();
	PRR = (1<<PRSPI) | (1<<PRADC); // Power off: SPI, ADC
	BMS_SERIAL.begin(BMS_SERIAL_RATE);
#ifdef DEBUG_TO_SERIAL
	DebugSerial.begin(DEBUG_TO_SERIAL);
	DEBUG(F("\nBMS gate to Microart, v")); DEBUGN(VERSION);
	DEBUGN(F("Copyright by Vadim Kulakov (c) 2021, vad7@yahoo.com"));
#endif
	uint8_t b = eeprom_read_byte((uint8_t*)&EEPROM.work.bms_qty);
	if(b == 0 || b > 32) { // init EEPROM
		memset(&work, 0, sizeof(work));
		work.bms_qty = 16;
		work.options = (1<<o_average);
		work.UART_read_period = 1;
		work.round = round_true;
		work.Vmaxhyst = 1;
		work.temp_correct = 6;
		work.BalansDeltaDefault = 10;
		work.BalansDeltaPause = 2*60*60;
		work.BalansDeltaI[0] = 6; work.BalansDelta[0] = 20;
		work.BalansDeltaI[1] = 16; work.BalansDelta[1] = 50;
		work.watchdog = 3;
		eeprom_update_block(&work, &EEPROM.work, sizeof(EEPROM.work));
	}
	eeprom_read_block(&work, &EEPROM.work, sizeof(EEPROM.work));
	memset(bms_Q, 0, sizeof(bms_Q));
#ifdef DEBUG_TO_SERIAL
	DEBUG(F("Cells: ")); DEBUG(work.bms_qty); DEBUG(F(" (")); DEBUG((const __FlashStringHelper*)dbg_cells); DEBUGN(F("=X)"));
	DEBUG(F("Watchdog(")); DEBUG(WATCHDOG_NO_CONN); DEBUG(F("s): ")); if(!work.watchdog) DEBUG(F("NONE")); else { if(work.watchdog & 1) DEBUG(F("I2C ")); if(work.watchdog & 2) DEBUG(F("BMS")); }
	DEBUG(F(" (")); DEBUG((const __FlashStringHelper*)dbg_watchdog); DEBUGN(F("=1-I2C,2-BMS,3-all)"));
	DEBUGN(F("BMS slave address: 1"));
	DEBUG(F("BMS read period, ms: "));
	if(work.UART_read_period > 1) DEBUG(work.UART_read_period);
	else if(work.UART_read_period == 1) DEBUG(F("Synch I2C"));
	else DEBUG(F("OFF"));
	DEBUG(F(" (")); DEBUG((const __FlashStringHelper*)dbg_period); DEBUGN(F("=0-off,1-synch,X ms)"));
	DEBUG(F("Options: ")); if(bitRead(work.options, o_average)) DEBUG(F("V-average(1) ")); DEBUG("\n");
	DEBUG(F("BMS voltage round: ")); DEBUG(work.round == round_true ? F("5/4") : work.round == round_cut ? F("cut") : work.round == round_up ? F("up") : F("?"));
	DEBUG(F(" (")); DEBUG((const __FlashStringHelper*)dbg_round); DEBUGN(F("=0-5/4,1-cut,2-up)"));
	DEBUG(F("BMS voltage correct, mV: ")); DEBUG(work.V_correct); DEBUG(F(" (")); DEBUG((const __FlashStringHelper*)dbg_correct); DEBUGN(F("=X)"));
	DEBUG(F("BMS cell max catch, 10mV: ")); if(work.Vmaxhyst) { DEBUG('+'); DEBUG(work.Vmaxhyst); } else DEBUG(F("OFF"));
	DEBUG(F(" (")); DEBUG((const __FlashStringHelper*)dbg_vmaxhyst); DEBUGN(F("=X)"));
	DEBUG(F("BMS Temp correct, C: ")); DEBUG(work.temp_correct); DEBUG(F(" (")); DEBUG((const __FlashStringHelper*)dbg_temp_correct); DEBUGN(F("=X)"));
	DEBUG(F("BMS Balans delta, mV: ")); DEBUG(work.BalansDeltaDefault); DEBUG(F(" (")); DEBUG((const __FlashStringHelper*)dbg_delta_default); DEBUGN(F("=X)"));
	DEBUG(F("BMS Balans delta array, [I>mV]: "));
	for(uint8_t i = 0; i < sizeof(work.BalansDelta)/sizeof(work.BalansDelta[0]); i++) {	DEBUG(work.BalansDeltaI[i]); DEBUG('>'); DEBUG(work.BalansDelta[i]); DEBUG(' '); }
	DEBUGN(F(" (Delta: Dn=X, Current: In=X)"));
	DEBUG(F("BMS Balans delta decrease pause, s: ")); DEBUG(work.BalansDeltaPause);  DEBUG(F(" (")); DEBUG((const __FlashStringHelper*)dbg_delta_pause); DEBUGN(F("=X)"));
	DEBUGN(F("\nCommands:"));
	DEBUG((const __FlashStringHelper*)dbg_debug); DEBUGN(F("=0,1,2,3"));
	DEBUG((const __FlashStringHelper*)dbg_temp); DEBUGN(F("=X"));
	DEBUG((const __FlashStringHelper*)dbg_delta_change_pause); DEBUGN(F("=X"));
	DEBUGN(F("Out: Vn=X (All: n=0)\nQn=X"));
	DEBUG((const __FlashStringHelper*)dbg_seterr); DEBUGN(F("=X"));
#ifdef MICROART_BMS_READWRITE
	DEBUG((const __FlashStringHelper*)dbg_I2C_READ_BMS); DEBUGN(F("=addr"));
	DEBUG((const __FlashStringHelper*)dbg_I2C_WRITE_BMS); DEBUGN(F("addr=X"));
#endif
#endif
	//
	Wire.begin();
	// deactivate internal pullups for twi.
	digitalWrite(SDA, 0);
	digitalWrite(SCL, 0);

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
	static uint32_t led_flashing, bms_reading;
	uint32_t m = millis();
	if(m - watchdog_timer >= 1000UL) { // every 1 sec
		watchdog_timer = m;
		if(work.watchdog & 1) watchdog_I2C++;
		if(work.watchdog & 2) watchdog_BMS++;
		if(watchdog_I2C > WATCHDOG_NO_CONN || watchdog_BMS > WATCHDOG_NO_CONN) {
			DEBUG(F("* WATCHDOG: ")); DEBUG(watchdog_I2C); DEBUG(','); DEBUGN(watchdog_BMS);
			Wire.end();
			while(1) { // reboot
				sleep_cpu();
				*portOutputRegister(digitalPinToPort(LED_PD)) ^= digitalPinToBitMask(LED_PD);
				_delay_ms(100);
			}
		}
		if(delta_change_pause < 0xFFFF) delta_change_pause++;
#ifdef DEBUG_TO_SERIAL
		DebugSerial_read();
#endif
	}
	if(m - led_flashing >= (error_alarm_time == 0 ? 1500UL : 200UL)) {
		led_flashing = m;
		if(error_alarm_time) error_alarm_time--;
	    *portOutputRegister(digitalPinToPort(LED_PD)) ^= digitalPinToBitMask(LED_PD);
	}
	if(bms_idx_prev != bms_idx) {
		if(bms_idx == 0) {
			bms_need_read = true;
#ifdef DEBUG_TO_SERIAL
			if(debug >= 2) {
				DEBUG(F("I2C ms: "));
				DEBUGN(m - bms_loop_time);
				bms_loop_time = m;
			}
#endif
		}
#ifdef DEBUG_TO_SERIAL
		if(debug >= 4) {
			DEBUG(F("I2C_R_"));
			DEBUG(bms_idx_prev + 1);
			DEBUG(F("->"));
			DEBUGN(bms[bms_idx_prev]);
		}
#endif
		bms_idx_prev = bms_idx;
	}

	// Read from UART
	BMS_Serial_read();
	m = millis();
	if(m - bms_last_read_time > BMS_PAUSE_BETWEEN_READS) {
		bms_last_read_time = m;
		if((work.UART_read_period == 1 && bms_need_read) || (work.UART_read_period > 1 && m - bms_reading > work.UART_read_period)) {
			bms_reading = m;
			bms_last_read_time += BMS_PAUSE_BETWEEN_READS;
			bms_need_read = false;
			read_idx = 0;	// reset read index
			for(uint8_t i = 0; i < sizeof(BMS_Cmd_Request); i++) BMS_SERIAL.write(pgm_read_byte(&BMS_Cmd_Request[i]));
		} else if(delta_new) {
			uint8_t crc = 0;
			uint8_t b;
			read_idx = 0;	// reset read index
			for(uint8_t i = 0; i < sizeof(BMS_Cmd_ChangeDelta); i++) {
				BMS_SERIAL.write(b = pgm_read_byte(&BMS_Cmd_ChangeDelta[i]));
				crc += b;
			}
			b = delta_new >> 8;
			BMS_SERIAL.write(b);
			crc += b;
			b = delta_new & 0xFF;
			BMS_SERIAL.write(b);
			crc += b;
			BMS_SERIAL.write(crc);
			delta_new = 0;
		}
	}
	// I2C slave receive
	if(i2c_receive_idx && i2c_receive_idx > i2c_receive[0]) { // i2c write
		DEBUGIF(4,F("I2C_W: "));
		if(i2c_receive[0] >= sizeof(i2c_receive)) {
			DEBUGIFN(0,F("LEN ERROR!"));
			i2c_receive_idx = 0;
		} else {
			uint8_t crc = 0;
			for(uint8_t i = 0; i <= i2c_receive[0]; i++) { // +CRC
				crc += i2c_receive[i];
				DEBUGIF(4,i2c_receive[i]);
				DEBUGIF(4," ");
			}
			if(crc != 0) {
				error_alarm_time = 50;
				DEBUGIFN(0,F("- CRC ERROR!"));
			} else if(i2c_receive[1] == 4) { // Broadcast I2CCom_JobWR
				bms_min = i2c_receive[2] + 200;
				bms_full = i2c_receive[3] + 200;
				map_mode = i2c_receive[4];
				if(debug == 2 || bitRead(debug_info, 0)) {
					bitClear(debug_info, 0);
					DEBUG(F("I2C_W: Min=")); DEBUG(bms_min); DEBUG(F(",Max=")); DEBUG(bms_full); DEBUG(F(",Mode=")); DEBUGN(map_mode);
				}
				if(work.UART_read_period == 1 && bms[0] == 0) bms_need_read = true;
			} else if(i2c_receive[1] == 6) { // Broadcast I2CCom_JobWR_MPPT
//				uint8_t A = i2c_receive[8];
//				if(debug == 2 || bitRead(debug_info, 1)) {
//					bitClear(debug_info, 1);
//					DEBUG(F("I2C_W: I=")); DEBUGN(A);
//				}
				if(delta_change_pause > BMS_CHANGE_DELTA_PAUSE_MIN && !delta_new && delta_active) {
					uint8_t A = i2c_receive[8];
					int8_t i = sizeof(work.BalansDelta)/sizeof(work.BalansDelta[0])-1;
					for(; i >= 0; i--) if(A >= work.BalansDeltaI[i]) break;
					uint16_t d = i >= 0 ? work.BalansDelta[i] : work.BalansDeltaDefault;
					if(d == delta_active || (d < delta_active && delta_change_pause <= work.BalansDeltaPause)) d = 0;
					if(d && d == delta_next) {
						if(++delta_change_equalizer > BMS_CHANGE_DELTA_EQUALIZER) {
							delta_change_equalizer = 0;
							delta_new = d;
						}
					} else {
						delta_change_equalizer = 0;
						delta_next = d;
					}
					if(delta_new) {
						if(debug >= 1) { DEBUG(F("D_NEW: ")); DEBUGN(delta_new); }
						delta_change_pause = BMS_CHANGE_DELTA_PAUSE_MIN > 60 ? BMS_CHANGE_DELTA_PAUSE_MIN - 60 : 60;
					}
				}
			}
			if(i2c_receive_idx > i2c_receive[0] + 1) {
				memcpy(i2c_receive, i2c_receive + i2c_receive[0] + 1, i2c_receive_idx -= i2c_receive[0] + 1);
			} else i2c_receive_idx = 0;
			watchdog_I2C = 0;
		}
		DEBUGIF(4,F("\n"));
	}
	delay(MAIN_LOOP_PERIOD);
}


