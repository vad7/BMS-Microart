/*
 * Copyright (c) 2020 by Vadim Kulakov vad7@yahoo.com, vad711
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

#include "Arduino.h"
#include <avr/wdt.h>
#include <util/atomic.h>
#include <Wire.h>
extern "C" {
	#include "utility/twi.h"
}

#define VERSION F("1.00")
#define DEBUG_TO_SERIAL

#define BMS_QTY						16
#define I2C_FREQ					2500
//#define

#define KEY1_PD						2
#define LED_PD						LED_BUILTIN
#define KEYS_INIT					PORTC |= (1<<PC2) // Pull-up, to GND: LEFT - 10kOm, OK - 30kOm, RIGHT - 56kOm
#define KEY_TWICE_PRESS_TIMEOUT		3		// sec
#define KEY_FAST_PRESSING_TIME		1		// sec
#define KEY_RELEASE_TIMEOUT			200		// *100 msec
#define MAIN_LOOP_PERIOD			1		// msec
#define BMS_NO_TEMP					255


#ifdef DEBUG_TO_SERIAL
#define DEBUG(s) Serial.print(s)
#define DEBUGH(h) Serial.print(h, HEX)
#define DEBUGN(s) Serial.println(s)
#else
#define DEBUG(s)
#define DEBUGH(h)
#define DEBUGN(s)
#endif

struct WORK {
	uint8_t  mode;
	uint32_t BMS_read_period;		// ms
} work;

struct _EEPROM {
	WORK    work;
} __attribute__ ((packed));

struct _EEPROM EEMEM EEPROM;

enum {
	f_BMS_Ready = 0
};
uint8_t  flags = 0;					// f_*
uint16_t bms[BMS_QTY];				// V, hundreds
uint8_t  bms_idx = 0;
uint8_t  bms_idx_prev = 0;
uint16_t bms_min = 0;				// V, hundreds
uint16_t bms_full = 0;				// V, hundreds
uint8_t  map_mode = 0;
uint8_t  temp = BMS_NO_TEMP;		// C, +50
uint8_t  crc;
uint8_t  error = 0;
char     read_buffer[32];
uint8_t  i2c_receive[32];
uint8_t  i2c_receive_idx = 0;
uint8_t  read_idx = 0;

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

void WaitKeysRelease(void)
{
	uint8_t tm = KEY_RELEASE_TIMEOUT;
	while(digitalRead(KEY1_PD) && --tm) delay(100);
	delay(100);
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
	i2c_write(bms[bms_idx] & 0xFF);					// V
	i2c_write(bms[bms_idx] >> 8);					// V
	i2c_write(bms_idx == 1 ? temp : BMS_NO_TEMP);	// temp
	i2c_write(0);									// Q%
	i2c_write(0);									// err
	crc = 0 - crc;
	i2c_write(crc);
	if(++bms_idx == BMS_QTY) bms_idx = 0;
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

void BMS_read(void)
{
	while(Serial.available()) {
		int16_t r = Serial.read();
		if(r == -1) break;
		read_buffer[read_idx++] = (char)r;
		if(r == '\r' || read_idx == sizeof(read_buffer)-1) {
			read_buffer[read_idx-1] = '\0';
			read_idx = 0;
			uint16_t d;
			DEBUG(F("Read: "));
			if(read_buffer[0] == 'T') {
				DEBUG(F("T "));
				d = atoi(read_buffer + 1);
				temp = d + 50;
			} else {
				d = atoi(read_buffer);
				if(d) {
					ATOMIC_BLOCK(ATOMIC_FORCEON) for(uint8_t i = 0; i < BMS_QTY; i++) bms[i] = d;
					if(!bitRead(flags, f_BMS_Ready)) {
						i2c_set_slave_addr(bms_idx + 1);
						bitSet(flags, f_BMS_Ready);
					}
				}
			}
			DEBUGN(d);
			break;
		}
	}
}

void setup()
{
#if defined(__AVR_ATmega8__)
	MCUCR |= (1<<SE); // Idle sleep enable
#else // ATmega48/P, ATmega88/P, ATmega168/P, ATmega328/P
	SMCR = (1<<SE); // Idle sleep enable
#endif
	wdt_enable(WDTO_2S); // Enable WDT
	// Setup keys
	KEYS_INIT;
	// Setup classes
#ifdef DEBUG_TO_SERIAL
	Serial.begin(250000);
#endif
	DEBUG(F("BMS gate to Microart. v.")); DEBUGN(VERSION);

	if(eeprom_read_byte((uint8_t*)&EEPROM.work.mode) == 255) { // init EEPROM
		memset(&work, 0, sizeof(work));
		work.mode = 0;
		work.BMS_read_period = 1000;
		eeprom_update_block(&work, &EEPROM.work, sizeof(EEPROM.work));
	}
	eeprom_read_block(&work, &EEPROM.work, sizeof(EEPROM.work));
	DEBUG(F("Read UART period, ms: ")); DEBUGN(work.BMS_read_period);

	//
	Wire.begin();
	Wire.setClock(I2C_FREQ);
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
	if(bms_idx_prev != bms_idx) {
		DEBUG(F("R_"));
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
	if(m - bms_reading > work.BMS_read_period) {
		bms_reading = m;
		BMS_read();
	}
	// I2C slave receive
	if(i2c_receive_idx && i2c_receive_idx > i2c_receive[0]) { // i2c write
		DEBUG(F("W: "));
		if(i2c_receive[0] >= sizeof(i2c_receive)) {
			DEBUG(F("LEN ERROR!"));
			i2c_receive_idx = 0;
		} else {
			uint8_t crc = 0;
			for(uint8_t i = 0; i <= i2c_receive[0]; i++) { // +CRC
				crc += i2c_receive[i];
				DEBUGH(i2c_receive[i]);
				DEBUG(" ");
			}
			if(crc != 0) {
				error = 50;
				DEBUG("- CRC ERROR!");
			} else if(i2c_receive[0] == 4) { // I2CCom_JobWR
				bms_min = i2c_receive[2] + 200;
				bms_full = i2c_receive[3] + 200;
				map_mode = i2c_receive[4];
			}
			if(i2c_receive_idx > i2c_receive[0] + 1) {
				memcpy(i2c_receive, i2c_receive + i2c_receive[0] + 1, i2c_receive_idx -= i2c_receive[0] + 1);
			} else i2c_receive_idx = 0;
		}
		DEBUG("\n");
	}
	//delay(MAIN_LOOP_PERIOD);
}


