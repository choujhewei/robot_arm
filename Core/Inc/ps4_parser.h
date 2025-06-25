/*
 * ps4_parser.h
 *
 *  Created on: Jun 25, 2025
 *      Author: jeffr
 */

#ifndef INC_PS4_PARSER_H_
#define INC_PS4_PARSER_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void process_ps4_packet(uint8_t *packet);
void loop_check_uart5(uint8_t *rx_buffer, volatile uint8_t *packet_ready);

#ifdef __cplusplus
}
#endif

#endif /* INC_PS4_PARSER_H_ */
