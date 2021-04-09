/*
 * buttons.h
 *
 *  Created on: 7 kwi 2021
 *      Author: marek
 */

#ifndef APPLICATION_USER_BUTTONS_H_
#define APPLICATION_USER_BUTTONS_H_

#include <stdio.h>
#include <stdint.h>
#include "main.h"

#ifdef	__cplusplus
extern "C" {
#endif

typedef struct button {
	GPIO_TypeDef *port;
    uint16_t pin;
    uint32_t timer;
    uint8_t state;
    void (*push_proc)(void);
    void (*long_proc)(void);
} button_t;

void button_init(button_t *btn, GPIO_TypeDef *port, uint16_t pin, void (*push_proc)(void), void (*long_proc)(void));
void button_handle(button_t *btn);

void rotary_irq (void);
int16_t rotary_handle (void);

void pwr_btn_callback (void);
void key1_callback (void);
void key2_callback (void);
void key3_callback (void);
void volup_callback (void);
void voldn_callback (void);
void rot_push_callback (void);

#endif /* APPLICATION_USER_BUTTONS_H_ */
