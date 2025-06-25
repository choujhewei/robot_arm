/*
 * ps4_parser.cpp
 *
 *  Created on: Jun 25, 2025
 *      Author: jeffr
 */

#include "main.h"
#include "ps4_parser.h"
#include <stdio.h>

void process_ps4_packet(uint8_t *packet) {
    if (packet[0] != 0xAA || packet[8] != 0x55) return;

    int8_t lx = (int8_t)packet[1];
    int8_t ly = (int8_t)packet[2];
    int8_t rx = (int8_t)packet[3];
    int8_t ry = (int8_t)packet[4];
    uint16_t buttons = packet[5] | (packet[6] << 8);
    uint8_t checksum = (lx + ly + rx + ry + packet[5] + packet[6]) & 0xFF;

    if (checksum != packet[7]) return;
    char debug_msg[64];
    sprintf(debug_msg, "LX=%d LY=%d RX=%d RY=%d BTN=0x%04X\r\n", lx, ly, rx, ry, buttons);
    for (char *p = debug_msg; *p; p++) {
        while (!LL_USART_IsActiveFlag_TXE(USART2));
        LL_USART_TransmitData8(USART2, *p);
    }
}

void loop_check_uart5(uint8_t *rx_buffer, volatile uint8_t *packet_ready) {
    if (*packet_ready) {
        *packet_ready = 0;
        process_ps4_packet(rx_buffer);
    }
}


