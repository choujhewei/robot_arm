/*
 * motor_control.cpp
 *
 *  Created on: Mar 31, 2025
 *      Author: jeffr
 */
#include "main.h"
#include "mainpp.h"
#include "motor_control.h"
#include "Mx106v2.h"

void SendMotorCommand(float wR[2]) {

		dynamixel_cmd[1] = -wR[0]*RADs2VEL; // - to match pel2sole FK and J
		dynamixel_cmd[2] = +wR[1]*RADs2VEL; // + to match pel2sole FK and J
//		dynamixel_cmd[3] = -wR[3]*RADs2VEL; // - to match pel2sole FK and J
//		dynamixel_cmd[4] = -wR[2]*RADs2VEL; // - to match pel2sole FK and J
//		dynamixel_cmd[5] = +wR[1]*RADs2VEL; // + to match pel2sole FK and J
//		dynamixel_cmd[6] = -wR[0]*RADs2VEL; // - to match pel2sole FK and J
//
//		dynamixel_cmd[7] = -wL[5]*RADs2VEL; // - to match pel2sole FK and J
//		dynamixel_cmd[8] = -wL[4]*RADs2VEL; // - to match pel2sole FK and J
//		dynamixel_cmd[9] = +wL[3]*RADs2VEL; // + to match pel2sole FK and J
//		dynamixel_cmd[10] = +wL[2]*RADs2VEL;// + to match pel2sole FK and J
//		dynamixel_cmd[11] = +wL[1]*RADs2VEL;// + to match pel2sole FK and J
//		dynamixel_cmd[12] = -wL[0]*RADs2VEL;// - to match pel2sole FK and J

	// Limitation
//	if(right_joint_angle[4] <= 0.08727f) {
//		if(dynamixel_cmd[3] < 0) {
//			dynamixel_cmd[3] = 0;
//		}
//	}
//	if(left_joint_angle[4] <= 0.08727f) {
//		if(dynamixel_cmd[9] < 0) {
//			dynamixel_cmd[9] = 0;
//		}
//	}
//	for(uint8_t i = 1; i <= 12; i++) {
//		if(abs(dynamixel_cmd[i]) >= 210) {
//			if(dynamixel_cmd[i] >= 0) {
//				dynamixel_cmd[i] = 210;
//			}
//			else {
//				dynamixel_cmd[i] = -210;
//			}
//		}
//	}
//
//	if(current_index > CMD_LEN){
//	dynamixel_cmd[1] = 0; // - to match pel2sole FK and J
//	dynamixel_cmd[2] = 0; // + to match pel2sole FK and J
//	dynamixel_cmd[3] = 0; // - to match pel2sole FK and J
//	dynamixel_cmd[4] = 0; // - to match pel2sole FK and J
//	dynamixel_cmd[5] = 0; // + to match pel2sole FK and J
//	dynamixel_cmd[6] = 0; // - to match pel2sole FK and J
//
//	dynamixel_cmd[7] = 0; // - to match pel2sole FK and J
//	dynamixel_cmd[8] = 0; // - to match pel2sole FK and J
//	dynamixel_cmd[9] = 0; // + to match pel2sole FK and J
//	dynamixel_cmd[10] = 0;// + to match pel2sole FK and J
//	dynamixel_cmd[11] = 0;// + to match pel2sole FK and J
//	dynamixel_cmd[12] = 0;// - to match pel2sole FK and J
//	}

	// Yaw control
	uint8_t ID_list[1] = { 1 };
	SyncWrite_Velocity(1, ID_list, dynamixel_cmd);
	// Yaw control
}


