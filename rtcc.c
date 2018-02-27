#include "rtcc.h"
#include "hal.h"
#include "hw_conf.h"

#define I2C_ADDRESS 0x51
#define HEX_TO_BCD(x) ((x / 10) << 4) | (x % 10)

static Time time;

void rtcc_init(void)
{
    uint8_t tx[2];
    uint8_t rx[2];
    tx[0] = 0x25; //Oscillator register
    tx[1] = 0x01;
    i2cMasterTransmitTimeout(&I2C_DEV, I2C_ADDRESS, tx, 2, rx, 0, MS2ST(10));
    tx[0] = 0x26;//Battery switch register
    tx[1] = 0x06;
    i2cMasterTransmitTimeout(&I2C_DEV, I2C_ADDRESS, tx, 2, rx, 0, MS2ST(10));
    tx[0] = 0x27;//Pin_IO register
    tx[1] = 0x02;
    i2cMasterTransmitTimeout(&I2C_DEV, I2C_ADDRESS, tx, 2, rx, 0, MS2ST(10));
    tx[0] = 0x29;//INTA_enable
    tx[1] = 0x90;//Interrupt activated as a flag (permanent signal) by alarm1
    i2cMasterTransmitTimeout(&I2C_DEV, I2C_ADDRESS, tx, 2, rx, 0, MS2ST(10));
    tx[0] = 0x2B;//Flags
    tx[1] = 0x00;//Clear all flags
    i2cMasterTransmitTimeout(&I2C_DEV, I2C_ADDRESS, tx, 2, rx, 0, MS2ST(10));
    tx[0] = 0x10;//Alarm enables
    tx[1] = 0x00;//Diasables all alarms
    i2cMasterTransmitTimeout(&I2C_DEV, I2C_ADDRESS, tx, 2, rx, 0, MS2ST(10));
}

void rtcc_update(void)
{
    i2cAcquireBus(&I2C_DEV);
    uint8_t tx[1];
    uint8_t rx[1];
    tx[0] = 0x01;//Current time second
    i2cMasterTransmitTimeout(&I2C_DEV, I2C_ADDRESS, tx, 1, rx, 1, MS2ST(10));
    time.second = ((rx[0] >> 4) & 0x07) * 10 + (rx[0] & 0x0F);
    tx[0] = 0x02;//Current time minutes
    i2cMasterTransmitTimeout(&I2C_DEV, I2C_ADDRESS, tx, 1, rx, 1, MS2ST(10));
    time.minute = ((rx[0] >> 4) & 0x07) * 10 + (rx[0] & 0x0F);
    tx[0] = 0x03;//Current time hours
    i2cMasterTransmitTimeout(&I2C_DEV, I2C_ADDRESS, tx, 1, rx, 1, MS2ST(10));
    time.hour = ((rx[0] >> 4) & 0x03) * 10 + (rx[0] & 0x0F);
    tx[0] = 0x04;//Current time days
    i2cMasterTransmitTimeout(&I2C_DEV, I2C_ADDRESS, tx, 1, rx, 1, MS2ST(10));
    time.day = ((rx[0] >> 4) & 0x03) * 10 + (rx[0] & 0x0F);
    tx[0] = 0x06;////Current time months
    i2cMasterTransmitTimeout(&I2C_DEV, I2C_ADDRESS, tx, 1, rx, 1, MS2ST(10));
    time.month = ((rx[0] >> 4) & 0x01) * 10 + (rx[0] & 0x0F);
    tx[0] = 0x07;//Years
    i2cMasterTransmitTimeout(&I2C_DEV, I2C_ADDRESS, tx, 1, rx, 1, MS2ST(10));
    time.year = (rx[0] >> 4) * 10 + (rx[0] & 0x0F);
    i2cReleaseBus(&I2C_DEV);
}

Time rtcc_get_time(void)
{
    return time;
}

Time rtcc_set_time(sec, min, hours, weekday, day, month, year);
{
	//When reading or writing the time it is very important to make a read or write access in one go
    // Before setting the time, the STOP bit should be set and the prescalers should be cleared 
	// (see Section 8.16 “Stop_enable register” on page 60).
	
	// An example of setting the time: 14 hours, 23 minutes and 19 seconds :
	// I2C START condition
	// I2C slave address + write (A2h)
	// register address (2Eh)
	// write data (set STOP, 01h)
	// write data (clear prescaler, A4h)
	// write data (100th seconds, 00h)
	// write data (Hours, 14h)
	// write data (Minutes, 23h)
	// write data (Seconds, 19h)
	
	// I2C START condition
	// I2C slave address + write (A2h)
	// register address (2Eh)
	// write data (clear STOP, 00h). Time starts counting from this point
	// I2C STOP condition

	i2cAcquireBus(&I2C_DEV);
    uint8_t tx[10];
    uint8_t rx[1];
	
	tx[0] = 0xA2; // write slave
	tx[1] = 0x2E;//Stop enable register
	tx[2] = 0x01; //set stop bit
	i2cMasterTransmitTimeout(&I2C_DEV, I2C_ADDRESS, tx, 3, rx, 0, MS2ST(10));
	tx[0] = 0x2F;//resset register
	tx[1] = 0xA4; //clear prescaler
	i2cMasterTransmitTimeout(&I2C_DEV, I2C_ADDRESS, tx, 2, rx, 0, MS2ST(10));
	tx[0] = 0x00;
	tx[1] = 100thSecBCD;
	tx[2] = hoursBCD;
	tx[3] = minBCD;
	tx[4] = secBD;
	i2cMasterTransmitTimeout(&I2C_DEV, I2C_ADDRESS, tx, 5, rx, 0, MS2ST(10));
	i2cReleaseBus(&I2C_DEV);
	
	i2cAcquireBus(&I2C_DEV);
	tx[0] = 0xA2; // write slave
	tx[1] = 0x2E;//Stop enable register
	tx[2] = 0x00; //clear stop bit
    i2cMasterTransmitTimeout(&I2C_DEV, I2C_ADDRESS, tx, 3, rx, 0, MS2ST(10));
	i2cReleaseBus(&I2C_DEV);
	
	
}

void rtcc_enable_alarm(void)
{
    uint8_t numDays;
    uint8_t month = time.day == 1 ? time.month : (time.month + 1) % 12;
    if (month == 1 || month == 3 || month == 5 || month == 8 || month == 10 || month == 12)
        numDays = 31;
    else if (month == 2)
        numDays = time.year % 4 == 0 ? 29 : 28;
    else
        numDays = 30;
    uint8_t day = time.day == 1 ? numDays : (time.day - 1 > numDays ? numDays : (time.day - 1));
    uint8_t tx[1];
    uint8_t rx[1];
    tx[0] = 0x0B;//Day_alarm1
    tx[1] = HEX_TO_BCD(day);
    i2cMasterTransmitTimeout(&I2C_DEV, I2C_ADDRESS, tx, 2, rx, 0, MS2ST(10));
    tx[0] = 0x10;//Alarm enables
    tx[1] = 0x08;//Enables alarm1
    i2cMasterTransmitTimeout(&I2C_DEV, I2C_ADDRESS, tx, 2, rx, 0, MS2ST(10));
}
