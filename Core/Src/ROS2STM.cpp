/*
 * ROS2STM.cpp
 *
 *  Created on: Jun 25, 2025
 *      Author: jeffr
 */

#include "main.h"
#include "ROS2STM.h"
#include "stm32f4xx_it.h"
#include "Mx106v2.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void parse_and_control(char *input) {
    char temp_buffer[RX_BUFFER_SIZE];
    strcpy(temp_buffer, input);

    char *segments[50];
    int count = 0;

    char *start = strchr(temp_buffer, '[');
    while (start && count < 50) {
        char *end = strchr(start, ']');
        if (!end) break;

        *end = '\0';
        segments[count++] = start + 1;
        start = strchr(end + 1, '[');
    }

//    printf("Parsed %d segments\n", count);

    for (int i = 0; i < count; i++) {
//        printf("Segment[%d]: %s\n", i, segments[i]);

        char *token;
        char *rest = segments[i];

        token = strtok(rest, ",");
        if (!token) {
            printf("First token missing\n");
            continue;
        }

        char hand = token[0];
        float angle[6];

        for (int j = 0; j < 6; j++) {
            token = strtok(NULL, ",");
            if (!token) {
                printf("Angle %d missing in segment %d\n", j + 1, i);
                return;
            }
            angle[j] = atof(token);
        }

        int32_t cmd[7]={0,0,0,0,0,0,0};
        for (int j = 0; j < 6; j++) {
            cmd[j+1] = (int32_t)((angle[j])+180);
        }
        int32_t cmd_mod[7] = {0,(int32_t)(cmd[1]/0.088f),(int32_t)((-cmd[2]+360)/0.088f),(int32_t)((-cmd[3]+360)/0.088f),(int32_t)((-cmd[4]+360)/0.088f),(int32_t)((-cmd[5]+360)/0.088f),(int32_t)(cmd[6]/0.088f)};
        printf("CMD: %ld %ld %ld %ld %ld %ld %ld \r\n",
                      cmd_mod[0], cmd_mod[1], cmd_mod[2], cmd_mod[3], cmd_mod[4], cmd_mod[5], cmd_mod[6]);
        uint8_t ID_list[5];
        for (int j = 0; j < 5; j++) {
            ID_list[j] = (hand == 'R') ? j + 1 : j + 7;
        }

//        printf("Hand: %c | IDs: %d %d %d %d %d %d | CMD: %ld %ld %ld %ld %ld \r\n",
//               hand,
//               ID_list[0], ID_list[1], ID_list[2], ID_list[3], ID_list[4], ID_list[5],
//               cmd[0], cmd[1], cmd[2], cmd[3], cmd[4]);
        int32_t vel = 5;
        SyncWrite_PositionWithVelocityProfile(6, ID_list, cmd_mod, vel);
    }
}


