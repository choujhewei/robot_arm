/*
 * Mx106v2.c
 *
 *  Created on: 2021�~6��6��
 *      Author: LDSC-yabi
 */

#include "Mx106v2.h"
//#include "Mx106v2_hand.h"
#include "Mx106v2_CRC.h"
#include <stdio.h>

// Motor initial angle
// This is important. Sometimes it may seriously influence the performance
int32_t dynamixel_init_position[19] =
	{ 0, 1792, 2304, 1792, 1792, 2304, 2048, 1792, 1792, 2304, 2304, 2304, 2048, 2048, 2048, 2048, 2048, 2048, 2048 }; //12 3310  3470
//          1     2     3     4     5     6     7     8     9    10    11    12    13    14    15    16    17    18

int32_t dynamixel_position[19] = { 0 };
int32_t dynamixel_velocity[19] = { 0 };
int32_t dynamixel_relative_position[19] = { 0 }; // relative to initial angle
int16_t dynamixel_current[19] = { 0 };

int32_t dynamixel_cmd[19] = { 0 };

uint8_t dynamixel_Ready = 1;
uint8_t Is_dynamixel_GetData = 0;
uint32_t Status_packet_length = 0;
uint8_t Packet_Return = 1;

uint8_t Status_Return_Level = ALL;     // Status packet return states ( NON , READ , ALL )

uint8_t Instruction_Packet_Array[200] = { HEADER1, HEADER2, HEADER3, RESERVED }; // Array to hold instruction packet data
uint8_t Status_Packet_Array[25] = { 0 };                    // Array to hold returned status packet data

// Printing the packet might clean the data in UART, we need to copy packet into debug_packet
// after reading all data in packet.
uint8_t debug_Instruction_Packet_Array[35] = { 0 };  // Array to debug instruction packet data
uint8_t debug_Status_Packet_Array[15] = { 0 };       // Array to debug status packet data

uint16_t crc = 0;

void UART4_DMA_Config() {
	// TX
	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_4, LL_USART_DMA_GetRegAddr(UART4));
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_4, (uint32_t)Instruction_Packet_Array);
	LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_4, Instruction_Packet_Array[5] + 7);

	LL_USART_EnableDMAReq_TX(UART4);

	LL_DMA_ClearFlag_TC4(DMA1);
	LL_USART_ClearFlag_TC(UART4);

	LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_4);
	LL_USART_EnableIT_TC(UART4);

	// RX
	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_2, LL_USART_DMA_GetRegAddr(UART4));
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_2, (uint32_t)Status_Packet_Array);
	LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_4, Status_packet_length + 4);

	LL_USART_EnableDMAReq_RX(UART4);

	LL_DMA_ClearFlag_TC2(DMA1);

	LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_2);

}



//------------------------------------------------------------------------------
// Private Methods
//------------------------------------------------------------------------------
void debugInstructionframe(void) {
	for(int i = 0; i < 15; i++) {
		debug_Instruction_Packet_Array[i] = Instruction_Packet_Array[i];
	}
	for(int i = 0; i < 15; i++)
		printf("%x, ", debug_Instruction_Packet_Array[i]);
	printf("\r\nyou transmit!\r\n");
}

void debugStatusframe(void) {
	for(int i = 0; i < 15; i++) {
		debug_Status_Packet_Array[i] = Status_Packet_Array[i];
	}
	for(int i = 0; i < 15; i++)
		printf("%x, ", debug_Status_Packet_Array[i]);
	printf("\r\nyou recieved!\r\n");
}

void transmitInstructionPacket(void) {  // Transmit instruction packet to Dynamixel
	dynamixel_Ready = 0;
#if USE_THREE_STATE_GATE == 1
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4);
	printf("1\r\n");
#else
	LL_USART_SetTransferDirection(UART4, LL_USART_DIRECTION_TX);
	printf("2\r\n");
#endif
	printf("3\r\n");
	LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_4, Instruction_Packet_Array[5]+7); // +7 includes : FF FF FD 00 ID LEN1 LEN2
	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_4);

}

//void receiveStatusPacket(void) {
//    // 切換為接收模式
//    LL_USART_SetTransferDirection(UART4, LL_USART_DIRECTION_RX);
//
//    // 清除 DMA 狀態，確保能接收新的數據
//    LL_DMA_ClearFlag_TC2(DMA1);
//    LL_DMA_ClearFlag_HT2(DMA1);
//    LL_DMA_ClearFlag_TE2(DMA1);
//
//    // 設置 DMA 接收長度（根據讀取的數據量來決定）
//    LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_4, 20);
//
//    // 啟動 DMA RX
//    LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_4);
//}
void readStatusPacket(void) {
	for(uint8_t i = 0; i < Status_packet_length + 4; i++) {
		while(LL_USART_IsActiveFlag_RXNE(UART4) == RESET) {
		}
		Status_Packet_Array[i] = LL_USART_ReceiveData8(UART4);
	}
	dynamixel_Ready = 1;
}

void readStatusPacket_pos_DMA(int32_t* position) {
	if(Status_Packet_Array[8] == 0) {   //確認錯誤碼
		position[Status_Packet_Array[4]] = Status_Packet_Array[12] << 24 | Status_Packet_Array[11] << 16 | Status_Packet_Array[10] << 8
			| Status_Packet_Array[9];
	}
}
void readStatusPacket_PING(int32_t* position){
	position[Status_Packet_Array[4]]=Status_Packet_Array[9];
}
//-------------------------------------------------------------------------------------------------------------------------------
// EEPROM AREA
uint8_t OperatingMode(uint8_t ID, uint8_t OPERATION_MODE) {
// Set Operation Mode: Current Mode 0x00, Velocity Mode 0x01, Position Mode 0x03
	while(dynamixel_Ready != 1) {
	}
	Instruction_Packet_Array[4] = ID;
	Instruction_Packet_Array[5] = 0x06;
	Instruction_Packet_Array[6] = 0x00;
	Instruction_Packet_Array[7] = COMMAND_WRITE_DATA;
	Instruction_Packet_Array[8] = EEPROM_OPERATION_MODE;
	Instruction_Packet_Array[9] = 0x00;
	Instruction_Packet_Array[10] = OPERATION_MODE;

	crc = update_crc(Instruction_Packet_Array, Instruction_Packet_Array[5] + 5);

	Instruction_Packet_Array[11] = crc & 0x00FF;
	Instruction_Packet_Array[12] = (crc >> 8) & 0x00FF;

	Status_packet_length = 7; // ID(1) + LEN(2) + INS(1) + ERR(1) + CRC(2)

	if(ID == 0XFE || Status_Return_Level != ALL) {
		Packet_Return = 0;
		transmitInstructionPacket();
		return (0x00);
	}
	else {
		Packet_Return = 1;
		transmitInstructionPacket();
		readStatusPacket();
		if(Status_Packet_Array[8] == 0)
			return (0x00);
		else
			return (Status_Packet_Array[8] | 0xF000);
	}
}

//-------------------------------------------------------------------------------------------------------------------------------
// RAM AREA

uint8_t TorqueEnable(uint8_t ID, _Bool Status) {
	/*
	 Must Enable it before any motion(Velocity or Position)
	 When it is enabled, EEROM will be locked.
	 */
	while(dynamixel_Ready != 1) {
	}
	Instruction_Packet_Array[4] = ID;
	Instruction_Packet_Array[5] = 0x06;
	Instruction_Packet_Array[6] = 0x00;
	Instruction_Packet_Array[7] = COMMAND_WRITE_DATA;
	Instruction_Packet_Array[8] = RAM_TORQUE_ENABLE;
	Instruction_Packet_Array[9] = 0x00;
	Instruction_Packet_Array[10] = Status;

	crc = update_crc(Instruction_Packet_Array, Instruction_Packet_Array[5] + 5);

	Instruction_Packet_Array[11] = crc & 0x00FF;
	Instruction_Packet_Array[12] = (crc >> 8) & 0x00FF;

	Status_packet_length = 7; // ID(1) + LEN(2) + INS(1) + ERR(1) + CRC(2)

	if(ID == 0XFE || Status_Return_Level != ALL) {
		Packet_Return = 0;
		transmitInstructionPacket();
		return (0x00);
	}
	else {
		Packet_Return = 1;
		transmitInstructionPacket();
		readStatusPacket();
		if(Status_Packet_Array[8] == 0)
			return (0x00);
		else
			return (Status_Packet_Array[8] | 0xF000);
	}
}

uint8_t StatusReturnLevel(uint8_t ID, uint8_t level) {
	while(dynamixel_Ready != 1) {
	}
	Instruction_Packet_Array[4] = ID;
	Instruction_Packet_Array[5] = 0x06;
	Instruction_Packet_Array[6] = 0x00;
	Instruction_Packet_Array[7] = COMMAND_WRITE_DATA;
	Instruction_Packet_Array[8] = RAM_STATUS_RETURN_LEVEL;
	Instruction_Packet_Array[9] = 0x00;
	Instruction_Packet_Array[10] = level;

	crc = update_crc(Instruction_Packet_Array, Instruction_Packet_Array[5] + 5);

	Instruction_Packet_Array[11] = crc & 0x00FF;
	Instruction_Packet_Array[12] = (crc >> 8) & 0x00FF;

	Status_packet_length = 7; // ID(1) + LEN(2) + INS(1) + ERR(1) + CRC(2)

	Status_Return_Level = level;

	if(ID == 0XFE || Status_Return_Level != ALL) {
		Packet_Return = 0;
		transmitInstructionPacket();
		return (0x00);
	}
	else {
		Packet_Return = 1;
		transmitInstructionPacket();
		readStatusPacket();
		if(Status_Packet_Array[8] == 0)
			return (0x00);
		else
			return (Status_Packet_Array[8] | 0xF000);
	}
}

uint8_t Velocity_PI(uint8_t ID, uint16_t P, uint16_t I) {
	while(dynamixel_Ready != 1) {
	}
	Instruction_Packet_Array[4] = ID;
	Instruction_Packet_Array[5] = 0x09;
	Instruction_Packet_Array[6] = 0x00;
	Instruction_Packet_Array[7] = COMMAND_WRITE_DATA;
	Instruction_Packet_Array[8] = RAM_VELOCITY_I_GAIN_L;
	Instruction_Packet_Array[9] = 0x00;
	Instruction_Packet_Array[10] = (uint8_t)(I);
	Instruction_Packet_Array[11] = (uint8_t)((I & 0xFF00) >> 8);
	Instruction_Packet_Array[12] = (uint8_t)(P);
	Instruction_Packet_Array[13] = (uint8_t)((P & 0xFF00) >> 8);

	crc = update_crc(Instruction_Packet_Array, Instruction_Packet_Array[5] + 5);

	Instruction_Packet_Array[14] = crc & 0x00FF;
	Instruction_Packet_Array[15] = (crc >> 8) & 0x00FF;

	Status_packet_length = 7; // ID(1) + LEN(2) + INS(1) + ERR(1) + CRC(2)

	if(ID == 0XFE || Status_Return_Level != ALL) {
		Packet_Return = 0;
		transmitInstructionPacket();
		return (0x00);
	}
	else {
		Packet_Return = 1;
		transmitInstructionPacket();
		readStatusPacket();
		if(Status_Packet_Array[8] == 0)
			return (0x00);
		else
			return (Status_Packet_Array[8] | 0xF000);
	}
}

uint8_t Position_PID(uint8_t ID, uint16_t P, uint16_t I, uint16_t D) {
	while(dynamixel_Ready != 1) {
	}
	Instruction_Packet_Array[4] = ID;
	Instruction_Packet_Array[5] = 0x0B;
	Instruction_Packet_Array[6] = 0x00;
	Instruction_Packet_Array[7] = COMMAND_WRITE_DATA;
	Instruction_Packet_Array[8] = RAM_POSITION_D_GAIN_L;
	Instruction_Packet_Array[9] = 0x00;
	Instruction_Packet_Array[10] = (uint8_t)(D);
	Instruction_Packet_Array[11] = (uint8_t)((D & 0xFF00) >> 8);
	Instruction_Packet_Array[12] = (uint8_t)(I);
	Instruction_Packet_Array[13] = (uint8_t)((I & 0xFF00) >> 8);
	Instruction_Packet_Array[14] = (uint8_t)(P);
	Instruction_Packet_Array[15] = (uint8_t)((P & 0xFF00) >> 8);

	crc = update_crc(Instruction_Packet_Array, Instruction_Packet_Array[5] + 5);

	Instruction_Packet_Array[16] = crc & 0x00FF;
	Instruction_Packet_Array[17] = (crc >> 8) & 0x00FF;

	Status_packet_length = 7; // ID(1) + LEN(2) + INS(1) + ERR(1) + CRC(2)

	if(ID == 0XFE || Status_Return_Level != ALL) {
		Packet_Return = 0;
		transmitInstructionPacket();
		return (0x00);
	}
	else {
		Packet_Return = 1;
		transmitInstructionPacket();
		readStatusPacket();
		if(Status_Packet_Array[8] == 0)
			return (0x00);
		else
			return (Status_Packet_Array[8] | 0xF000);
	}
}

uint8_t Velocity(uint8_t ID, int32_t Speed) {
	/*
	 units = 0.229 rpm
	 max velocity: 48 rpm (-210~210)
	 */
	while(dynamixel_Ready != 1) {
	}
	Instruction_Packet_Array[4] = ID;
	Instruction_Packet_Array[5] = 0x09;
	Instruction_Packet_Array[6] = 0x00;
	Instruction_Packet_Array[7] = COMMAND_WRITE_DATA;
	Instruction_Packet_Array[8] = RAM_GOAL_VELOCITY_1;
	Instruction_Packet_Array[9] = 0x00;
	Instruction_Packet_Array[10] = (uint8_t)(Speed & 0x000000FF);
	Instruction_Packet_Array[11] = (uint8_t)((Speed >> 8) & 0x000000FF);
	Instruction_Packet_Array[12] = (uint8_t)((Speed >> 16) & 0x000000FF);
	Instruction_Packet_Array[13] = (uint8_t)((Speed >> 24) & 0x000000FF);

	crc = update_crc(Instruction_Packet_Array, Instruction_Packet_Array[5] + 5);

	Instruction_Packet_Array[14] = crc & 0x00FF;
	Instruction_Packet_Array[15] = (crc >> 8) & 0x00FF;

	Status_packet_length = 7; // ID(1) + LEN(2) + INS(1) + ERR(1) + CRC(2)

	if(ID == 0XFE || Status_Return_Level != ALL) {
		Packet_Return = 0;
		transmitInstructionPacket();
		return (0x00);
	}
	else {
		Packet_Return = 1;
		transmitInstructionPacket();
		readStatusPacket();
		if(Status_Packet_Array[8] == 0)
			return (0x00);
		else
			return (Status_Packet_Array[8] | 0xF000);
	}
}

uint8_t PositionWithVelocity(uint8_t ID, int32_t Position, int32_t Moving_Velocity) {
	/*
	 units = 0.088 degree
	 position range: 0~360 (0~4095)
	 */
	while(dynamixel_Ready != 1) {
	}
	Instruction_Packet_Array[4] = ID;
	Instruction_Packet_Array[5] = 0x0D;
	Instruction_Packet_Array[6] = 0x00;
	Instruction_Packet_Array[7] = COMMAND_WRITE_DATA;
	Instruction_Packet_Array[8] = RAM_MOVING_VELOCITY_1;
	Instruction_Packet_Array[9] = 0x00;
	Instruction_Packet_Array[10] = (uint8_t)(Moving_Velocity);
	Instruction_Packet_Array[11] = (uint8_t)((Moving_Velocity >> 8) & 0x000000FF);
	Instruction_Packet_Array[12] = (uint8_t)((Moving_Velocity >> 16) & 0x000000FF);
	Instruction_Packet_Array[13] = (uint8_t)((Moving_Velocity >> 24) & 0x000000FF);
	Instruction_Packet_Array[14] = (uint8_t)(Position);
	Instruction_Packet_Array[15] = (uint8_t)((Position >> 8) & 0x000000FF);
	Instruction_Packet_Array[16] = (uint8_t)((Position >> 16) & 0x000000FF);
	Instruction_Packet_Array[17] = (uint8_t)((Position >> 24) & 0x000000FF);

	crc = update_crc(Instruction_Packet_Array, Instruction_Packet_Array[5] + 5);

	Instruction_Packet_Array[18] = crc & 0x00FF;
	Instruction_Packet_Array[19] = (crc >> 8) & 0x00FF;

	Status_packet_length = 7; // ID(1) + LEN(2) + INS(1) + ERR(1) + CRC(2)

	if(ID == 0XFE || Status_Return_Level != ALL) {
		Packet_Return = 0;
		transmitInstructionPacket();
		return (0x00);
	}
	else {
		Packet_Return = 1;
		transmitInstructionPacket();
		readStatusPacket();
		if(Status_Packet_Array[8] == 0)
			return (0x00);
		else
			return (Status_Packet_Array[8] | 0xF000);
	}
}

uint8_t Position(uint8_t ID, int32_t position) {
	while(dynamixel_Ready != 1) {
	}
	Instruction_Packet_Array[4] = ID;
	Instruction_Packet_Array[5] = 0x09;
	Instruction_Packet_Array[6] = 0x00;
	Instruction_Packet_Array[7] = COMMAND_WRITE_DATA;
	Instruction_Packet_Array[8] = RAM_GOAL_POSITION_1;
	Instruction_Packet_Array[9] = 0x00;
	Instruction_Packet_Array[10] = (uint8_t)(position & 0xFF);
	Instruction_Packet_Array[11] = (uint8_t)((position >> 8) & 0x000000FF);
	Instruction_Packet_Array[12] = (uint8_t)((position >> 16) & 0x000000FF);
	Instruction_Packet_Array[13] = (uint8_t)((position >> 24) & 0x000000FF);

	crc = update_crc(Instruction_Packet_Array, Instruction_Packet_Array[5] + 5);

	Instruction_Packet_Array[14] = crc & 0x00FF;
	Instruction_Packet_Array[15] = (crc >> 8) & 0x00FF;

	Status_packet_length = 7; // ID(1) + LEN(2) + INS(1) + ERR(1) + CRC(2)

	if(ID == 0XFE || Status_Return_Level != ALL) {
		Packet_Return = 0;
		transmitInstructionPacket();
		return (0x00);
	}
	else {
		Packet_Return = 1;
		transmitInstructionPacket();
		readStatusPacket();
		if(Status_Packet_Array[8] == 0)
			return (0x00);
		else
			return (Status_Packet_Array[8] | 0xF000);
	}
}

int32_t ReadPosition(uint8_t ID) {
	while(dynamixel_Ready != 1) {
	}
	Instruction_Packet_Array[4] = ID;
	Instruction_Packet_Array[5] = 0x07;
	Instruction_Packet_Array[6] = 0x00;
	Instruction_Packet_Array[7] = COMMAND_READ_DATA;
	Instruction_Packet_Array[8] = RAM_PRESENT_POSITION_1;
	Instruction_Packet_Array[9] = 0x00;
	Instruction_Packet_Array[10] = READ_FOUR_BYTE_LENGTH;
	Instruction_Packet_Array[11] = 0x00;

	crc = update_crc(Instruction_Packet_Array, Instruction_Packet_Array[5] + 5);

	Instruction_Packet_Array[12] = crc & 0x00FF;
	Instruction_Packet_Array[13] = (crc >> 8) & 0x00FF;

	Status_packet_length = 11; // ID(1) + LEN(2) + INS(1) + ERR(1) + PARA(4) + CRC(2)
	Packet_Return = 1;
	transmitInstructionPacket();
	readStatusPacket();

	if(Status_Packet_Array[8] == 0) {               // If there is no status packet error return value
		return (Status_Packet_Array[12] << 24 | Status_Packet_Array[11] << 16 | Status_Packet_Array[10] << 8 | Status_Packet_Array[9]); // Return present position value
	}
	else {
		return (Status_Packet_Array[8]);            // If there is a error Returns error value
	}
}

//-------------------------------------------------------------------------------------------------------------------------------
// Special Command
void PING(){
	while(dynamixel_Ready != 1) {
		}

		Instruction_Packet_Array[4] = 0xFE;
		Instruction_Packet_Array[5] = 0x03;
		Instruction_Packet_Array[6] = 0x00;
		Instruction_Packet_Array[7] = 0x01;

		crc = update_crc(Instruction_Packet_Array, Instruction_Packet_Array[5] + 5);

		Instruction_Packet_Array[8] = crc & 0x00FF;
		Instruction_Packet_Array[9] = (crc >> 8) & 0x00FF;

		Status_packet_length = 10; // ID(1) + LEN(2) + INS(1) + ERR(1) + PARA(4) + CRC(2)
		Packet_Return = 2;
		Is_dynamixel_GetData = 0;

		transmitInstructionPacket();
}
void SyncRead_Position(uint8_t n, uint8_t *ID_list) {
	while(dynamixel_Ready != 1) {
	}

	Instruction_Packet_Array[4] = 0xFE;
	Instruction_Packet_Array[5] = n + 7;
	Instruction_Packet_Array[6] = 0x00;
	Instruction_Packet_Array[7] = COMMAND_SYNC_READ;
	Instruction_Packet_Array[8] = RAM_PRESENT_POSITION_1;
	Instruction_Packet_Array[9] = 0x00;
	Instruction_Packet_Array[10] = READ_FOUR_BYTE_LENGTH;
	Instruction_Packet_Array[11] = 0x00;
	for(uint8_t i = 1; i <= n; i++) {
		Instruction_Packet_Array[i + 11] = ID_list[i - 1];
	}
	crc = update_crc(Instruction_Packet_Array, Instruction_Packet_Array[5] + 5);

	Instruction_Packet_Array[n + 12] = crc & 0x00FF;
	Instruction_Packet_Array[n + 13] = (crc >> 8) & 0x00FF;

	Status_packet_length = 11; // ID(1) + LEN(2) + INS(1) + ERR(1) + PARA(4) + CRC(2)
	Packet_Return = n;
	Is_dynamixel_GetData = 0;

	transmitInstructionPacket();
}

void SyncWrite_DisableDynamixels(uint8_t n, uint8_t *ID_list) {
	while(dynamixel_Ready != 1) {
	}

	Instruction_Packet_Array[4] = 0xFE;
	Instruction_Packet_Array[5] = 2 * n + 7;  // total data frame length (L+1)*n+7 (L is data length)
	Instruction_Packet_Array[6] = 0x00;
	Instruction_Packet_Array[7] = COMMAND_SYNC_WRITE;
	Instruction_Packet_Array[8] = RAM_TORQUE_ENABLE;
	Instruction_Packet_Array[9] = 0x00;
	Instruction_Packet_Array[10] = 0x01; // write data length(L)
	Instruction_Packet_Array[11] = 0x00;
	for(uint8_t i = 1; i <= n; i++) {
		Instruction_Packet_Array[2 * i + 10] = ID_list[i - 1];
		Instruction_Packet_Array[2 * i + 11] = 0x00;
	}
	crc = update_crc(Instruction_Packet_Array, Instruction_Packet_Array[5] + 5);

	Instruction_Packet_Array[2 * n + 12] = crc & 0x00FF;
	Instruction_Packet_Array[2 * n + 13] = (crc >> 8) & 0x00FF;

	Packet_Return = 0;
	transmitInstructionPacket();
}

void SyncWrite_EnableDynamixels(uint8_t n, uint8_t *ID_list) {
	while(dynamixel_Ready != 1) {
	}
	Instruction_Packet_Array[4] = 0xFE;
	Instruction_Packet_Array[5] = 2 * n + 7;  // total data frame length (L+1)*n+7 (L is data length)
	Instruction_Packet_Array[6] = 0x00;
	Instruction_Packet_Array[7] = COMMAND_SYNC_WRITE;
	Instruction_Packet_Array[8] = RAM_TORQUE_ENABLE;
	Instruction_Packet_Array[9] = 0x00;
	Instruction_Packet_Array[10] = 0x01; // write data length(L)
	Instruction_Packet_Array[11] = 0x00;
	for(uint8_t i = 1; i <= n; i++) {
		Instruction_Packet_Array[2 * i + 10] = ID_list[i - 1];
		Instruction_Packet_Array[2 * i + 11] = 0x01;
	}
	crc = update_crc(Instruction_Packet_Array, Instruction_Packet_Array[5] + 5);

	Instruction_Packet_Array[2 * n + 12] = crc & 0x00FF;
	Instruction_Packet_Array[2 * n + 13] = (crc >> 8) & 0x00FF;

	Packet_Return = 0;
	transmitInstructionPacket();
}

void SyncWrite_StatusReturnLevel(uint8_t n, uint8_t *ID_list, uint8_t level) {
	while(dynamixel_Ready != 1) {
	}

	Instruction_Packet_Array[4] = 0xFE;
	Instruction_Packet_Array[5] = 2 * n + 7;  // total data frame length (L+1)*n+7 (L is data length)
	Instruction_Packet_Array[6] = 0x00;
	Instruction_Packet_Array[7] = COMMAND_SYNC_WRITE;
	Instruction_Packet_Array[8] = RAM_STATUS_RETURN_LEVEL;
	Instruction_Packet_Array[9] = 0x00;
	Instruction_Packet_Array[10] = 0x01; // write data length(L)
	Instruction_Packet_Array[11] = 0x00;
	for(uint8_t i = 1; i <= n; i++) {
		Instruction_Packet_Array[2 * i + 10] = ID_list[i - 1];
		Instruction_Packet_Array[2 * i + 11] = level;
	}
	crc = update_crc(Instruction_Packet_Array, Instruction_Packet_Array[5] + 5);

	Instruction_Packet_Array[2 * n + 12] = crc & 0x00FF;
	Instruction_Packet_Array[2 * n + 13] = (crc >> 8) & 0x00FF;

	Status_Return_Level = level;
	Packet_Return = 0;
	transmitInstructionPacket();
}

void SyncWrite_Velocity(uint8_t n, uint8_t *ID_list, int32_t *cmd) {
	while(dynamixel_Ready != 1) {
	}

	Instruction_Packet_Array[4] = 0xFE;
	Instruction_Packet_Array[5] = 5 * n + 7;  // total data frame length (L+1)*n+7 (L is data length)
	Instruction_Packet_Array[6] = 0x00;
	Instruction_Packet_Array[7] = COMMAND_SYNC_WRITE;
	Instruction_Packet_Array[8] = RAM_GOAL_VELOCITY_1;
	Instruction_Packet_Array[9] = 0x00;
	Instruction_Packet_Array[10] = 0x04; // write data length (L)
	Instruction_Packet_Array[11] = 0x00;
	for(uint8_t i = 1; i <= n; i++) {
		Instruction_Packet_Array[5 * i + 7] = ID_list[i - 1];
		Instruction_Packet_Array[5 * i + 8] = (uint8_t)(cmd[ID_list[i - 1]] & 0x000000FF);
		Instruction_Packet_Array[5 * i + 9] = (uint8_t)((cmd[ID_list[i - 1]] >> 8) & 0x000000FF);
		Instruction_Packet_Array[5 * i + 10] = (uint8_t)((cmd[ID_list[i - 1]] >> 16) & 0x000000FF);
		Instruction_Packet_Array[5 * i + 11] = (uint8_t)((cmd[ID_list[i - 1]] >> 24) & 0x000000FF);
	}
	crc = update_crc(Instruction_Packet_Array, Instruction_Packet_Array[5] + 5);

	Instruction_Packet_Array[5 * n + 12] = crc & 0x00FF;
	Instruction_Packet_Array[5 * n + 13] = (crc >> 8) & 0x00FF;

	Packet_Return = 0;
	transmitInstructionPacket();
}

void SyncWrite_VelocityProfile(uint8_t n, uint8_t *ID_list, int32_t vel_profile) {
	while(dynamixel_Ready != 1) {
	}

	Instruction_Packet_Array[4] = 0xFE;
	Instruction_Packet_Array[5] = 5 * n + 7;  // total data frame length (L+1)*n+7 (L is data length)
	Instruction_Packet_Array[6] = 0x00;
	Instruction_Packet_Array[7] = COMMAND_SYNC_WRITE;
	Instruction_Packet_Array[8] = RAM_MOVING_VELOCITY_1;
	Instruction_Packet_Array[9] = 0x00;
	Instruction_Packet_Array[10] = 0x04; // write data length (L)
	Instruction_Packet_Array[11] = 0x00;
	for(uint8_t i = 1; i <= n; i++) {
		Instruction_Packet_Array[5 * i + 7] = ID_list[i - 1];
		Instruction_Packet_Array[5 * i + 8] = (uint8_t)(vel_profile & 0x000000FF);
		Instruction_Packet_Array[5 * i + 9] = (uint8_t)((vel_profile >> 8) & 0x000000FF);
		Instruction_Packet_Array[5 * i + 10] = (uint8_t)((vel_profile >> 16) & 0x000000FF);
		Instruction_Packet_Array[5 * i + 11] = (uint8_t)((vel_profile >> 24) & 0x000000FF);
	}
	crc = update_crc(Instruction_Packet_Array, Instruction_Packet_Array[5] + 5);

	Instruction_Packet_Array[5 * n + 12] = crc & 0x00FF;
	Instruction_Packet_Array[5 * n + 13] = (crc >> 8) & 0x00FF;

	Packet_Return = 0;
	transmitInstructionPacket();
}

void SyncWrite_Position(uint8_t n, uint8_t *ID_list, int32_t *cmd) {
	while(dynamixel_Ready != 1) {
	}

	Instruction_Packet_Array[4] = 0xFE;
	Instruction_Packet_Array[5] = 5 * n + 7;  // total data frame length (L+1)*n+7 (L is data length)
	Instruction_Packet_Array[6] = 0x00;
	Instruction_Packet_Array[7] = COMMAND_SYNC_WRITE;
	Instruction_Packet_Array[8] = RAM_GOAL_POSITION_1;
	Instruction_Packet_Array[9] = 0x00;
	Instruction_Packet_Array[10] = 0x04; // write data length (L)
	Instruction_Packet_Array[11] = 0x00;
	for(uint8_t i = 1; i <= n; i++) {
		Instruction_Packet_Array[5 * i + 7] = ID_list[i - 1];
		Instruction_Packet_Array[5 * i + 8] = (uint8_t)(cmd[ID_list[i - 1]] & 0x000000FF);
		Instruction_Packet_Array[5 * i + 9] = (uint8_t)((cmd[ID_list[i - 1]] >> 8) & 0x000000FF);
		Instruction_Packet_Array[5 * i + 10] = (uint8_t)((cmd[ID_list[i - 1]] >> 16) & 0x000000FF);
		Instruction_Packet_Array[5 * i + 11] = (uint8_t)((cmd[ID_list[i - 1]] >> 24) & 0x000000FF);
	}
	crc = update_crc(Instruction_Packet_Array, Instruction_Packet_Array[5] + 5);

	Instruction_Packet_Array[5 * n + 12] = crc & 0x00FF;
	Instruction_Packet_Array[5 * n + 13] = (crc >> 8) & 0x00FF;

	Packet_Return = 0;
	transmitInstructionPacket();
}

void SyncWrite_PositionWithVelocityProfile(uint8_t n, uint8_t *ID_list, int32_t *cmd, int32_t vel_profile) {
	while(dynamixel_Ready != 1) {
	}

	Instruction_Packet_Array[4] = 0xFE;
	Instruction_Packet_Array[5] = 9 * n + 7;  // total data frame length (L+1)*n+7 (L is data length)
	Instruction_Packet_Array[6] = 0x00;
	Instruction_Packet_Array[7] = COMMAND_SYNC_WRITE;
	Instruction_Packet_Array[8] = RAM_MOVING_VELOCITY_1;
	Instruction_Packet_Array[9] = 0x00;
	Instruction_Packet_Array[10] = 0x08; // write data length (L)
	Instruction_Packet_Array[11] = 0x00;
	for(uint8_t i = 1; i <= n; i++) {
		Instruction_Packet_Array[9 * i + 3] = ID_list[i - 1];
		Instruction_Packet_Array[9 * i + 4] = (uint8_t)(vel_profile & 0x000000FF);
		Instruction_Packet_Array[9 * i + 5] = (uint8_t)((vel_profile >> 8) & 0x000000FF);
		Instruction_Packet_Array[9 * i + 6] = (uint8_t)((vel_profile >> 16) & 0x000000FF);
		Instruction_Packet_Array[9 * i + 7] = (uint8_t)((vel_profile >> 24) & 0x000000FF);
		Instruction_Packet_Array[9 * i + 8] = (uint8_t)(cmd[ID_list[i - 1]] & 0x000000FF);
		Instruction_Packet_Array[9 * i + 9] = (uint8_t)((cmd[ID_list[i - 1]] >> 8) & 0x000000FF);
		Instruction_Packet_Array[9 * i + 10] = (uint8_t)((cmd[ID_list[i - 1]] >> 16) & 0x000000FF);
		Instruction_Packet_Array[9 * i + 11] = (uint8_t)((cmd[ID_list[i - 1]] >> 24) & 0x000000FF);
	}
	crc = update_crc(Instruction_Packet_Array, Instruction_Packet_Array[5] + 5);

	Instruction_Packet_Array[9 * n + 12] = crc & 0x00FF;
	Instruction_Packet_Array[9 * n + 13] = (crc >> 8) & 0x00FF;

	Packet_Return = 0;
	transmitInstructionPacket();
}

// =============================    For robot    ==========================

void InitializeDynamixelSetting() {
	uint8_t ID_list[2] = { 1,2 };
//	uint8_t ID_list2[6] = { 13, 14, 15, 16, 17, 18 };
	SyncWrite_StatusReturnLevel(2, ID_list, 1);
	//SyncWrite_StatusReturnLevel_hand(6, ID_list2, 1);
}

void SetDynamixelPID() {
	Position_PID(1, 2000, 100, 100);
	Position_PID(2, 2000, 100, 100);
	Position_PID(6, 1000, 100, 100);

	Position_PID(7, 2000, 100, 100);
	Position_PID(8, 2000, 100, 100);
	Position_PID(12, 1000, 100, 100);

//	Position_PID_hand(13, 5, 0, 0);
//	Position_PID_hand(14, 5, 0, 0);
//	Position_PID_hand(15, 5, 0, 0);
//
//	Position_PID_hand(16, 5, 0, 0);
//	Position_PID_hand(17, 5, 0, 0);
//	Position_PID_hand(18, 5, 0, 0);

	/* Dong */
	Velocity_PI(1, 125, 1750);
	Velocity_PI(2, 125, 1750);
	Velocity_PI(3, 845, 3392);
	Velocity_PI(4, 845, 3392);
	Velocity_PI(5, 845, 3392);
	Velocity_PI(6, 125, 1750);

	Velocity_PI(7, 125, 1750);
	Velocity_PI(8, 125, 1750);
	Velocity_PI(9, 845, 3392);
	Velocity_PI(10, 845, 3392);
	Velocity_PI(11, 845, 3392);
	Velocity_PI(12, 125, 1750);


	// by Ziegler�VNichols method
//	Velocity_PI(1, 125, 1750);
//	Velocity_PI(2, 125, 1750);
//
//	Velocity_PI(7, 125, 1750);
//	Velocity_PI(8, 125, 1750);
//
//	Velocity_PI(3, 600, 2500);
//	Velocity_PI(4, 600, 2500);
//	Velocity_PI(5, 600, 2500);
//
//	Velocity_PI(9, 600, 2500);
//	Velocity_PI(10, 600, 2500);
//	Velocity_PI(11, 600, 2500);
////
////	// Yaw control //
//	Velocity_PI(6, 600, 2500);
//	Velocity_PI(12, 600, 2500);
	// Yaw control //


}

void InitializeDynamixelPosition() {

	/*Pelvis on right sole*/
	/*P_rsole - P_pelvis resolved in pelvis frame is [0 0 -0.35 0 0 0] */
//	//Right (+- in parentheses converts pel2sole_model to match MX_motor +-)
		dynamixel_cmd[1] = dynamixel_init_position[1] + 13.7338f * DEG2POS;//(-) -13.7338
		dynamixel_cmd[2] = dynamixel_init_position[2] - 29.1063f * DEG2POS;//(+) -29.1063
		dynamixel_cmd[3] = dynamixel_init_position[3] - 55.0326f * DEG2POS;//(-) 55.0326
		dynamixel_cmd[4] = dynamixel_init_position[4] + 25.9263f * DEG2POS;//(-) -25.9263
		dynamixel_cmd[5] = dynamixel_init_position[5] + 13.7338f * DEG2POS;//(+) 13.7338
		dynamixel_cmd[6] = dynamixel_init_position[6] - 0.0000f  * DEG2POS;//(-) 0
	//Left (+- in parentheses converts MX_motor +- to match model pel2sole)
		dynamixel_cmd[7] = dynamixel_init_position[7]   + 13.7338f * DEG2POS;//(-) -13.7338
		dynamixel_cmd[8] = dynamixel_init_position[8]   + 29.1063f * DEG2POS;//(-) -29.1063
		dynamixel_cmd[9] = dynamixel_init_position[9]   + 55.0326f * DEG2POS;//(+) 55.0326
		dynamixel_cmd[10] = dynamixel_init_position[10] - 25.9263f * DEG2POS;//(+) -25.9263
		dynamixel_cmd[11] = dynamixel_init_position[11] + 13.7338f * DEG2POS;//(+) 13.7338
		dynamixel_cmd[12] = dynamixel_init_position[12] - 0.0000f  * DEG2POS;//(-) 0.

	/*Pelvis on both sole center */
	/*P_rsole - P_pelvis resolved in pelvis frame is [0 -0.06 -0.35 0 0 0] */

	//Right (+- in parentheses converts pel2sole_model to match MX_motor +-)
//			dynamixel_cmd[1] = dynamixel_init_position[1] + 0 * DEG2POS;//(-) 0
//			dynamixel_cmd[2] = dynamixel_init_position[2] - 32.2002f * DEG2POS;//(+) -32.2002
//			dynamixel_cmd[3] = dynamixel_init_position[3] - 61.0257f * DEG2POS;//(-) 61.0257
//			dynamixel_cmd[4] = dynamixel_init_position[4] + 28.8255f * DEG2POS;//(-) -28.8255
//			dynamixel_cmd[5] = dynamixel_init_position[5] + 0 * DEG2POS;//(+) 0
//			dynamixel_cmd[6] = dynamixel_init_position[6] - 0  * DEG2POS;//(-) 0
//		//Left (+- in parentheses converts MX_motor +- to match model pel2sole)
//			dynamixel_cmd[7] = dynamixel_init_position[7]   + 0 * DEG2POS;//(-) 0
//			dynamixel_cmd[8] = dynamixel_init_position[8]   + 32.2002f * DEG2POS;//(-) -32.2002
//			dynamixel_cmd[9] = dynamixel_init_position[9]   + 61.2057f * DEG2POS;//(+) 61.0257
//			dynamixel_cmd[10] = dynamixel_init_position[10] - 28.8255f * DEG2POS;//(+) -28.8255
//			dynamixel_cmd[11] = dynamixel_init_position[11] + 0 * DEG2POS;//(+) 0
//			dynamixel_cmd[12] = dynamixel_init_position[12] - 0  * DEG2POS;//(-) 0



	dynamixel_cmd[13] = dynamixel_init_position[13] - 5.0f * DEG2POS ;
	dynamixel_cmd[14] = dynamixel_init_position[14] - 85.0f * DEG2POS;
	dynamixel_cmd[15] = dynamixel_init_position[15] + 30.0f * DEG2POS ;

	dynamixel_cmd[16] = dynamixel_init_position[16] + 5.0f * DEG2POS ;
	dynamixel_cmd[17] = dynamixel_init_position[17] + 85.0f * DEG2POS;
	dynamixel_cmd[18] = dynamixel_init_position[18] - 30.0f * DEG2POS ;


	//Change to Position Mode (motor 1-12)
	uint8_t ID_list[12] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 };
	//uint8_t ID_list2[6] = { 13, 14, 15, 16, 17, 18 };
	SyncWrite_DisableDynamixels(12, ID_list);
//	SyncWrite_DisableDynamixels_hand(6, ID_list2);
	LL_mDelay(1);
	for(uint8_t id = 1; id <= 12; ++id) {
		OperatingMode(id, POSITION);
		LL_mDelay(1);
	}
//	for(uint8_t id = 13; id <= 18; ++id) {
//		OperatingMode_hand(id, POSITION);
//		LL_mDelay(1);
//	}
	SyncWrite_EnableDynamixels(12, ID_list);
//	SyncWrite_EnableDynamixels_hand(6, ID_list2);
	LL_mDelay(1);

	SyncWrite_VelocityProfile(12, ID_list, 30);
//	SyncWrite_VelocityProfile_hand(6, ID_list2, 30);
	SyncWrite_Position(12, ID_list, dynamixel_cmd);
//	SyncWrite_Position_hand(6, ID_list2, dynamixel_cmd);

	//Change to Position Mode (motor 13-18)
//	uint8_t ID_list2[6] = { 13, 14, 15, 16, 17, 18 };
//	SyncWrite_DisableDynamixels(6, ID_list2);
//	LL_mDelay(1);
//	for(uint8_t id = 13; id <= 18; ++id) {
//		OperatingMode(id, POSITION);
//		LL_mDelay(1);
//	}
//	SyncWrite_EnableDynamixels(6, ID_list2);
//	LL_mDelay(1);
//
//	SyncWrite_VelocityProfile(6, ID_list2, 30);
//	SyncWrite_Position(6, ID_list2, dynamixel_cmd);
	LL_mDelay(3000);

}

void ChangeOperatingMode() {
	// NOTE : Switching Operating Mode will reset gains(PID, Feedfoward) properly to the selected Operating Mode.
	// The profile generator and limits will also be reset.
	// Profile Velocity(112), Profile Acceleration(108) : Reset to ��0��
	// Goal PWM(100) : Reset to PWM Limit(36)

	uint8_t ID_list[12] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 };
	SyncWrite_DisableDynamixels(12, ID_list);
	for(int id = 1; id < 6; ++id) {
		OperatingMode(id, VELOCITY);
		LL_mDelay(1);
	}
	for(int id = 7; id < 12; ++id) {
		OperatingMode(id, VELOCITY);
		LL_mDelay(1);
	}

	// Yaw control //
	OperatingMode(6, VELOCITY);
	LL_mDelay(1);
	OperatingMode(12, VELOCITY);
	LL_mDelay(1);
	// Yaw control //

	SyncWrite_EnableDynamixels(12, ID_list);

//	uint8_t ID_list2[6] = { 13, 14, 15, 16, 17, 18 };
//	SyncWrite_PositionWithVelocityProfile_hand(6, ID_list2, dynamixel_cmd, 0);
}

