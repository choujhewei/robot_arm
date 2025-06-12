/*
 * Mx106v2.h
 *
 *  Created on: 2021�~6��6��
 *      Author: LDSC-yabi
 */

#ifndef Mx106v2_h
#define Mx106v2_h
#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define USE_THREE_STATE_GATE 1

//------------------------------------------------------------------------------
// Define Dynamixel Hex code table
//------------------------------------------------------------------------------
// EEPROM AREA                        address
#define EEPROM_MODEL_NUMBER_L           0x00
#define EEPROM_MODEL_NUMBER_H           0x01
#define EEPROM_MODEL_INFO_1             0x02
#define EEPROM_MODEL_INFO_2             0x03
#define EEPROM_MODEL_INFO_3             0x04
#define EEPROM_MODEL_INFO_4             0x05
#define EEPROM_VERSION                  0x06
#define EEPROM_ID                       0x07
#define EEPROM_BAUDRATE                 0x08

#define EEPROM_OPERATION_MODE           0x0B

#define EEPROM_MAX_VOLTAGE_LIMIT_1      0x20
#define EEPROM_MAX_VOLTAGE_LIMIT_2      0x21
#define EEPROM_MIN_VOLTAGE_LIMIT_1      0x22
#define EEPROM_MIN_VOLTAGE_LIMIT_2      0x23

#define EEPROM_CURRENT_LIMIT_1          0x26
#define EEPROM_CURRENT_LIMIT_2          0x27

#define EEPROM_MAX_VELOCITY_LIMIT_1     0x2C
#define EEPROM_MAX_VELOCITY_LIMIT_2     0x2D
#define EEPROM_MAX_VELOCITY_LIMIT_3     0x2E
#define EEPROM_MAX_VELOCITY_LIMIT_4     0x2F

#define EEPROM_MAX_POSITION_LIMIT_1     0x30
#define EEPROM_MAX_POSITION_LIMIT_2     0x31
#define EEPROM_MAX_POSITION_LIMIT_3     0x32
#define EEPROM_MAX_POSITION_LIMIT_4     0x33
#define EEPROM_MIN_POSITION_LIMIT_1     0x34
#define EEPROM_MIN_POSITION_LIMIT_2     0x35
#define EEPROM_MIN_POSITION_LIMIT_3     0x36
#define EEPROM_MIN_POSITION_LIMIT_4     0x37
#define EEPROM_SHUTDOWN                 0x3F

// RAM AREA
#define RAM_TORQUE_ENABLE               0x40
#define RAM_LED                         0x41

#define RAM_STATUS_RETURN_LEVEL         0x44

#define RAM_VELOCITY_I_GAIN_L           0x4C
#define RAM_VELOCITY_I_GAIN_H           0x4D
#define RAM_VELOCITY_P_GAIN_L           0x4E
#define RAM_VELOCITY_P_GAIN_H           0x4F

#define RAM_POSITION_D_GAIN_L           0x50
#define RAM_POSITION_D_GAIN_H           0x51
#define RAM_POSITION_I_GAIN_L           0x52
#define RAM_POSITION_I_GAIN_H           0x53
#define RAM_POSITION_P_GAIN_L           0x54
#define RAM_POSITION_P_GAIN_H           0x55

#define RAM_GOAL_CURRENT_1              0x66
#define RAM_GOAL_CURRENT_2              0x67

#define RAM_GOAL_VELOCITY_1             0x68
#define RAM_GOAL_VELOCITY_2             0x69
#define RAM_GOAL_VELOCITY_3             0x6A
#define RAM_GOAL_VELOCITY_4             0x6B

#define RAM_MOVING_VELOCITY_1           0x70
#define RAM_MOVING_VELOCITY_2           0x71
#define RAM_MOVING_VELOCITY_3           0x72
#define RAM_MOVING_VELOCITY_4           0x73

#define RAM_GOAL_POSITION_1             0x74
#define RAM_GOAL_POSITION_2             0x75
#define RAM_GOAL_POSITION_3             0x76
#define RAM_GOAL_POSITION_4             0x77

#define RAM_REALTIME_TICK_L             0x78
#define RAM_REALTIME_TICK_H             0x79

#define RAM_PRESENT_CURRENT_1           0x7E
#define RAM_PRESENT_CURRENT_2           0x7F

#define RAM_PRESENT_VELOCITY_1          0x80
#define RAM_PRESENT_VELOCITY_2          0x81
#define RAM_PRESENT_VELOCITY_3          0x82
#define RAM_PRESENT_VELOCITY_4          0x83

#define RAM_PRESENT_POSITION_1          0x84
#define RAM_PRESENT_POSITION_2          0x85
#define RAM_PRESENT_POSITION_3          0x86
#define RAM_PRESENT_POSITION_4          0x87

#define RAM_PRESENT_VOLTAGE             0x90

#define RAM_PRESENT_TEMPERATURE         0x92

//------------------------------------------------------------------------------
// Instruction commands Set
//------------------------------------------------------------------------------

#define COMMAND_PING                    0x01
#define COMMAND_READ_DATA               0x02
#define COMMAND_WRITE_DATA              0x03
#define COMMAND_REG_WRITE_DATA          0x04
#define COMMAND_ACTION                  0x05
#define COMMAND_RESET                   0x06
#define COMMAND_REBOOT                  0x08
#define COMMAND_STATUS_RETURN           0x55
#define COMMAND_SYNC_READ               0x82
#define COMMAND_SYNC_WRITE              0x83
#define COMMAND_BULK_READ               0x92
#define COMMAND_BULK_WRITE              0x93

//-------------------------------------------------------------------------------------------------------------------------------
//Instruction packet lengths
#define READ_ONE_BYTE_LENGTH            0x01
#define READ_TWO_BYTE_LENGTH            0x02
#define READ_FOUR_BYTE_LENGTH           0x04

//------------------------------------------------------------------------------
// Specials
//------------------------------------------------------------------------------
#define VELOCITY                        0x01
#define POSITION                        0x03

#define NONE                            0x00
#define READ                            0x01
#define ALL                             0x02

#define BROADCAST_ID                    0xFE

#define HEADER1                         0xFF
#define HEADER2                         0xFF
#define HEADER3                         0xFD
#define RESERVED                        0x00

#define TIMEOUT                         1      // in millis()

#define INST_REBOOT                     0x08

extern int32_t dynamixel_init_position[19];
extern int32_t dynamixel_position[19];
extern int32_t dynamixel_velocity[19];
extern int32_t dynamixel_relative_position[19];
extern int16_t dynamixel_current[19];

extern int32_t dynamixel_cmd[19];

extern uint8_t dynamixel_Ready;
extern uint8_t Is_dynamixel_GetData;
extern uint8_t Packet_Return;
extern uint32_t Status_packet_length;
extern uint8_t Status_Packet_Array[25];
//-------------------------------------------------------------------------------------------------------------------------------
// Declare function
//-------------------------------------------------------------------------------------------------------------------------------
void UART4_DMA_Config();
void USART6_DMA_Config();
void readStatusPacket_pos_DMA(int32_t* position);

// EEPROM AREA
uint8_t OperatingMode(uint8_t ID, uint8_t OPERATION_MODE);

// RAM AREA
uint8_t TorqueEnable(uint8_t ID, _Bool Status);
uint8_t StatusReturnLevel(uint8_t ID, uint8_t level);
uint8_t Velocity_PI(uint8_t ID, uint16_t P, uint16_t I);
uint8_t Position_PID(uint8_t ID, uint16_t P, uint16_t I, uint16_t D);
uint8_t Velocity(uint8_t ID, int32_t Speed);
uint8_t PositionWithVelocity(uint8_t ID, int32_t Position, int32_t Moving_Velocity);
uint8_t Position(uint8_t ID, int32_t position);
int32_t ReadPosition(uint8_t ID);
uint8_t Read_TorqueEnable(uint8_t ID);
uint8_t Reboot(uint8_t ID);
uint8_t readOperatingMode(uint8_t ID);
void MX_DMA_RX_UART4_Init(void);
void MX_DMA_RX_USART6_Init(void);
void Read_Operating_Mode(uint8_t ID);
void SyncRead_Position(uint8_t n, uint8_t *ID_list);
void SyncWrite_DisableDynamixels(uint8_t n, uint8_t *ID_list);
void SyncWrite_EnableDynamixels(uint8_t n, uint8_t *ID_list);
void SyncWrite_StatusReturnLevel(uint8_t n, uint8_t *ID_list, uint8_t level);
void SyncWrite_Velocity(uint8_t n, uint8_t *ID_list, int32_t *cmd);
void SyncWrite_VelocityProfile(uint8_t n, uint8_t *ID_list, int32_t vel_profile);
void SyncWrite_Position(uint8_t n, uint8_t *ID_list, int32_t *cmd);
void SyncWrite_PositionWithVelocityProfile(uint8_t n, uint8_t *ID_list, int32_t *cmd, int32_t vel_profile);
void PING(uint8_t id);
void readStatusPacket_PING(int32_t* position);
void SyncLED_Disable(uint8_t n, uint8_t *ID_list);
void SyncLED_Enable(uint8_t n, uint8_t *ID_list);

//--------------------------------------------------------------------------------------------------------------------------------
// For Robot
void InitializeDynamixelSetting();
void InitializeDynamixelPosition();
void ChangeOperatingMode();
void SetDynamixelPID();
void ReadMotorPosition();

#ifdef __cplusplus
}
#endif
#endif
