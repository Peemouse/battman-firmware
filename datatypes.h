#ifndef _DATATYPES_H_
#define _DATATYPES_H_

#include "ch.h"

typedef enum
{
    PACKET_CONNECT = 0x00,
    PACKET_CONSOLE = 0x01,
    PACKET_GET_DATA = 0x02,
    PACKET_GET_CELLS = 0x03,
    PACKET_ERASE_NEW_FW = 0x04,
    PACKET_WRITE_NEW_FW = 0x05,
    PACKET_JUMP_BOOTLOADER = 0x06,
    PACKET_CONFIG_SET_FIELD = 0x07,
    PACKET_CONFIG_GET_FIELD = 0x08,
    PACKET_CONFIG_SET_ALL = 0x09,
    PACKET_CONFIG_GET_ALL = 0x0A
} PacketID;

// typedef enum
// {
//     CAN_PACKET_SWITCHOFF,
// 	CAN_PACKET_INFINITY_SET_CURRENT:
// 	CAN_PACKET_INFINITY_STATUS,
// 	CAN_PACKET_VESC_STOP,
// 	CAN_PACKET_VESC_GETMCCONF,
// } CANPacketID;

typedef enum
{
    EVENT_SWITCH = 0x00,
    EVENT_CHARGER = 0x01,
    EVENT_USB = 0x02,
    EVENT_RTCC  = 0x04
} PowerOnEvent;

typedef enum
{
    FAULT_NONE = 0x00,
    FAULT_CELL_UV = 0x01,
    FAULT_CELL_OV = 0x02,
	FAULT_BATTERY_UV = 0x04,
	FAULT_BATTERY_OV = 0x08,
    FAULT_OVERCURRENT = 0x10,
    FAULT_BATTERY_TEMP = 0x20,
    FAULT_BOARD_TEMP = 0x40,
    FAULT_TURN_ON_SHORT = 0x80
} Fault;

typedef struct
{
	Fault fault_code;
    float busVoltage;
	float current;
	float batteryTemp;
	float boardTemp;
	bool isDischarging;
	bool isCharging;
	uint8_t second_fault;
	uint8_t minute_fault;
	uint8_t hour_fault;
	uint8_t day_fault;
	uint8_t month_fault;
	uint8_t year_fault;
} Fault_data;

typedef enum
{
    WARNING_NONE = 0x00,
    WARNING_CELL_LOW = 0x01,
    WARNING_CELL_HIGH = 0x02,
	WARNING_BATTERY_UV = 0x04,
	WARNING_BATTERY_OV = 0x08,
    WARNING_OVERCURRENT = 0x10,
    WARNING_BATTERY_TEMP = 0x20,
	WARNING_BOARD_TEMP = 0x40,
	WARNING_LTC6803_TEMP = 0x80,
	WARNING_LTC6803_ERROR = 0x100
	} Warning;

typedef enum
{
    CURRENT_CONTROL,
    FULL_CURRENT,
    BYPASS_CC,
    BYPASS_CCCV
} ChargeMode;

typedef enum
{
    STANDBY,
    DISCHARGING,
    CHARGING,
    PRECHARGING
} PowerStatus;


typedef struct __attribute__((__packed__))
{
    volatile uint8_t CANDeviceID;
    volatile uint8_t numCells;
    volatile float fullCellVoltage;
    volatile float emptyCellVoltage;
	//volatile float CellOVCutoff;
    volatile float packCapacity;
    volatile float lowVoltageCutoff;
    volatile float lowVoltageWarning;
    volatile float highVoltageCutoff;
    volatile float highVoltageWarning;
    volatile float maxCurrentCutoff;
    volatile float maxContinuousCurrent;
    volatile uint8_t continuousCurrentCutoffTime;
    volatile uint8_t continuousCurrentCutoffWarning;
    volatile float maxChargeCurrent;
    volatile float chargeVoltage;
    volatile float chargeCurrent;
    volatile uint16_t turnOnDelay;
    volatile uint16_t shutdownDelay;
    volatile uint8_t chargeMode;
    volatile float chargeCurrentGain_P;
    volatile float chargeCurrentGain_I;
    volatile uint16_t prechargeTimeout;
    volatile float balanceStartVoltage;
    volatile float balanceDifferenceThreshold;
    volatile bool chargerDisconnectShutdown;
	volatile float tempBoardWarning; //Added
	volatile float tempBoardCutoff; //Added
	volatile float tempBattWarning; //Added
	volatile float tempBattCutoff; //Added
	//volatile float tempLTC6803BalCutoff; //Hardcoded
	volatile bool isBattTempSensor; //Added
	volatile bool enBuzzer; //Added
} Config;

typedef struct
{
    uint8_t second;
    uint8_t minute;
    uint8_t hour;
    uint8_t day;
    uint8_t month;
    uint8_t year;
} Time;

typedef struct //To complete
{
    float busVoltage;
	float current;
	float batteryTemp;
	float boardTemp;
	PowerStatus power_status;
	Fault fault_code;
} bms_values;

#endif /* _DATATYPES_H_ */
