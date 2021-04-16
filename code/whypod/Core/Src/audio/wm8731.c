#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "stm32f4xx_hal.h"
#include "main.h"

#define WM8731_ADDR		(0x1A << 1)
//#define WM8731_ADDR		0x1A

extern I2C_HandleTypeDef hi2c1;
extern I2S_HandleTypeDef hi2s3;


#define AUDIO_BUFFER_SIZE		2200
static int16_t audio_data[2 * AUDIO_BUFFER_SIZE];


static void wm8731_write_reg(uint8_t reg, uint16_t value) {
	uint16_t tmp = 0;
	uint8_t data[2];

	tmp = ((uint16_t)reg << 9) | value;
	data[0] = (tmp & 0xFF00) >> 8;
	data[1] = tmp & 0x00FF;

	HAL_I2C_Master_Transmit(&hi2c1, WM8731_ADDR, data, 2, HAL_MAX_DELAY);
}


static void fill_audio_buffer (void) {
    for (int i = 0; i < AUDIO_BUFFER_SIZE; i++) {
        int16_t value = (int16_t)(32000.0 * sin(2.0 * M_PI * i / 22.0));
        audio_data[i * 2] = value;
        audio_data[i * 2 + 1] = value;
    }
}


void wm8731_set_in_volume(int vol) {
    // -23 <= vol <= 8
    const unsigned involume = 0x17 + vol;
    wm8731_write_reg(0x00, 0x100 | (involume & 0x1f)); // Left line in, unmute
}

void wm_8731_set_out_volume(int voldB) {
    // -73 <= voldB <= 6
    const unsigned volume = 121 + voldB;
    wm8731_write_reg(0x02, 0x100 | (volume & 0x7f)); // Left headphone
    wm8731_write_reg(0x03, 0x100 | (volume & 0x7f)); // Right headphone
}

void wm8731_init(void) {
	wm8731_write_reg(0x0f, 0b000000000); // Reset!
	wm8731_set_in_volume(0);
    wm_8731_set_out_volume(-10);
    wm8731_write_reg(0x04, 0b000010010); // Analog path - select DAC, no bypass
#ifdef WM8731_HIGHPASS
    wm8731_write_reg(0x05, 0b000000000); // Digital path - disable soft mute
#else
    wm8731_write_reg(0x05, 0b000000001); // Digital path - disable soft mute and highpass
#endif
    wm8731_write_reg(0x06, 0b000000000); // Power down control - enable everything
    wm8731_write_reg(0x07, 0b000000010); // Interface format - 16-bit I2S
    wm8731_write_reg(0x09, 0b000000001); // Active control - engage!

    fill_audio_buffer();
    HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t*)audio_data, 2 * AUDIO_BUFFER_SIZE);
}


void wm8731_handle (void) {

}
