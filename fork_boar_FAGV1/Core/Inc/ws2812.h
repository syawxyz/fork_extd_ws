/*
 * ws2812.h
 *
 *  Created on: Oct 4, 2025
 *      Author: syawal26
 */

#ifndef INC_WS2812_H_
#define INC_WS2812_H_

#include "main.h"

// Buffer allocated will be twice this
#define BUFFER_SIZE 24

// LED on/off counts.  PWM timer is running 125 counts.  LED_CNT need to be set to the total counts in the PWM.
#define LED_OFF 1 * 104 / 3 - 1  // A bit less than 1/ #89 is counter periode
#define LED_ON 2 * 104 / 3 + 2   // A bit more than 2/3 #89 is counter periode
// Increase reset cycles to ensure >50us latch on longer strips
#define LED_RESET_CYCLES 32          // Full 24-bit cycles

#define GL 0 // Green LED
#define RL 1 // Red LED
#define BL 2 // Blue LED

typedef enum {
    WS2812_Ok,
    WS2812_Err,
    WS2812_Mem
} ws2812_resultTypeDef;

// Statemachine states
typedef enum {
    LED_RES = 0,    // Reset latch cycle
    LED_IDL = 1,    // Idle doing nothing except waiting for is_dirty
    LED_DAT = 2     // Transferring led data - one led at the time
} ws2812_stateTypeDef;

typedef struct {
    TIM_HandleTypeDef *timer;               // Timer running the PWM - MUST run at 800 kHz
    uint32_t channel;                       // Timer channel
    uint16_t dma_buffer[BUFFER_SIZE * 2];   // Fixed size DMA buffer
    uint16_t leds;                          // Number of LEDs on the string
    uint8_t *led;                           // Front buffer: currently displayed LED RGB values
    uint8_t *back_led;                      // Back buffer: where writes occur before commit
    uint8_t *write_led;                     // Alias to current write target (normally back_led)
    ws2812_stateTypeDef led_state;          // LED Transfer state machine
    uint16_t led_cnt;                       // Counts through the leds starting from zero up to "leds" (use 16-bit for >=256 LEDs)
    uint8_t res_cnt;                        // Counts reset cycles when in reset state
    uint8_t is_dirty;                       // Indicates to the call back that the led color values have been updated
    uint8_t zero_halves;                    // Counts halves send during reset
    uint32_t dma_cbs;                       // Just used for statistics
    uint32_t dat_cbs;                       // Also used for statistics
    uint8_t pending_swap;                   // Defer front/back swap until safe boundary
} ws2812_handleTypeDef;

ws2812_resultTypeDef ws2812_init(ws2812_handleTypeDef *ws2812, TIM_HandleTypeDef *timer, uint32_t channel, uint16_t leds);

void ws2812_update_buffer(ws2812_handleTypeDef *ws2812, uint16_t *dma_buffer_pointer);

// Set all led values to zero
ws2812_resultTypeDef zeroLedValues(ws2812_handleTypeDef *ws2812);

// Set a single led value
ws2812_resultTypeDef setLedValue(ws2812_handleTypeDef *ws2812, uint16_t led, uint8_t color, uint8_t value);

// Set values of all 3 leds
ws2812_resultTypeDef setLedValues(ws2812_handleTypeDef *ws2812, uint16_t led, uint8_t r, uint8_t g, uint8_t b);

// Begin a frame update: subsequent setLedValue(s) write to back buffer
void ws2812_begin_frame(ws2812_handleTypeDef *ws2812);

// Commit the frame atomically by swapping front/back buffers
void ws2812_commit_frame(ws2812_handleTypeDef *ws2812);

#endif // _WS2812_H
/*
 * vim: ts=4 nowrap
 */

