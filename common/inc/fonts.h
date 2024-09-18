/*
 * fonts.h
 *
 *  Created on: Sep 18, 2024
 *      Author: maciej
 */

#ifndef MY_DRIVERS_ST7735_INC_FONTS_H_
#define MY_DRIVERS_ST7735_INC_FONTS_H_

#include <stdint.h>

typedef struct {
    const uint8_t width;
    uint8_t height;
    const uint16_t *data;
} FontDef;


extern FontDef Font_7x10;
extern FontDef Font_11x18;
extern FontDef Font_16x26;

#endif /* MY_DRIVERS_ST7735_INC_FONTS_H_ */
