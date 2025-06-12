/*
 * Mx106v2_CRC.h
 *
 *  Created on: 2021�~6��6��
 *      Author: LDSC-yabi
 */

#ifndef INC_MX106V2_CRC_H_
#define INC_MX106V2_CRC_H_

#include "main.h"

extern uint16_t crc_table[256];

extern uint16_t update_crc(uint8_t* data_blk_ptr, uint16_t data_blk_size);

#endif /* INC_MX106V2_CRC_H_ */
