/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Brennan Reed
 * SNHU CS 350
 * 11/27/22
 * Milestone Three - Using timers and interrupts to create Morse code in LEDs
 */

/*
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>

/* Driver configuration */
#include "ti_drivers_config.h"

#include <ti/drivers/Timer.h>

/* Global variables */
int16_t btnFlagL = 0;
int16_t btnFlagR = 0;

volatile unsigned char TimerFlag = 0;
unsigned short cnt = 0;

void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
    // Sufficient time has elapsed
    TimerFlag = 1;
}

void initTimer(void)
{
    Timer_Handle timer0;
    Timer_Params params;

    Timer_init();
    Timer_Params_init(&params);
    params.period = 500000;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    timer0 = Timer_open(CONFIG_TIMER_0, &params);

     if (timer0 == NULL) {
         /* Failed to initialized timer */
         while (1) {}
     }

     if (Timer_start(timer0) == Timer_STATUS_ERROR) {
         /* Failed to start timer */
         while (1) {}
     }
}

enum MC_States {MC_START, MC_WAIT, MC_SOS, MC_OK} MC_State;

/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn0(uint_least8_t index)
{
    /* Raises flag so that state can toggle to SOS*/
    btnFlagL = 1;
}

/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_1.
 *  This may not be used for all boards.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn1(uint_least8_t index)
{
    /* Raises flag so that state can toggle to OK */
    btnFlagR = 1;
}

void morseCodeStateMachine()
{
    /*Switch statement to do the STATE changes */
    switch (MC_State) {

    case MC_START:
        MC_State = MC_SOS;
        break;

    case MC_WAIT:   /* Checks for which flag to transition to OK or SOS */
        if (btnFlagL == 1) {
            MC_State = MC_SOS;
        } else if (btnFlagR == 1) {
            MC_State = MC_OK;
        }
        break;

    case MC_SOS:    /* If count is zero AND a flag is raised transitions to WAIT */
        if (!(cnt == 0)) {
            MC_State = MC_SOS;
        } else if (cnt == 0 && (btnFlagL == 1 || btnFlagR == 1)) {
            MC_State = MC_WAIT;
        }
        break;

    case MC_OK:     /* If count is zero AND a flag is raised transitions to WAIT */
        if (!(cnt == 0)) {
            MC_State = MC_OK;
        } else if (cnt == 0 && (btnFlagL == 1 || btnFlagR == 1)) {
            MC_State = MC_WAIT;
        }
        break;

    default:
        MC_State = MC_START;
        break;
    }

    /* Switch statement to implement the FUNCTIONALITY for each STATE (e.g., flashing LEDs, etc.) */
    switch (MC_State) {

    case MC_SOS:    /* Uses a counting variable to blink/dash SOS and reset flag and cnt at end */
        if (cnt < 5) {                                                      // S
            GPIO_toggle(CONFIG_GPIO_LED_0);
            cnt++;
        } else if (cnt >= 5 && cnt < 8) {                                   // break between chars
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
            cnt++;
        } else if (cnt >= 8 && cnt < 11) {                                  // O dash
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
            cnt++;
        } else if (cnt >= 11 && cnt < 12) {                                 // O blink
            GPIO_toggle(CONFIG_GPIO_LED_1);
            cnt++;
        } else if (cnt >= 12 && cnt < 15) {                                  // O dash
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
            cnt++;
        } else if (cnt >= 15 && cnt < 16) {                                  // O blink
            GPIO_toggle(CONFIG_GPIO_LED_1);
            cnt++;
        } else if (cnt >= 16 && cnt < 19) {                                  // O dash
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
            cnt++;
        } else if (cnt >= 19 && cnt < 22) {                                  // break between chars
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
            cnt++;
        } else if (cnt >= 22 && cnt < 27) {                                  // S
            GPIO_toggle(CONFIG_GPIO_LED_0);
            cnt++;
        } else if (cnt >= 27 && cnt < 34) {                                  //  end break between "words"
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
            cnt++;
        } else {
            cnt = 0;
            btnFlagL = 0;
        }
        break;

    case MC_OK:     /* Uses a counting variable to spell out OK and reset flag and cnt at end */
        if (cnt < 3) {                                                 // O dash
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
            cnt++;
        } else if (cnt >= 3 && cnt < 4) {                              // O blink
            GPIO_toggle(CONFIG_GPIO_LED_1);
            cnt++;
        } else if (cnt >= 4 && cnt < 7) {                              // O dash
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
            cnt++;
        } else if (cnt >= 7 && cnt < 8) {                              // O blink
            GPIO_toggle(CONFIG_GPIO_LED_1);
            cnt++;
        } else if (cnt >= 8 && cnt < 11) {                              // O dash
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
            cnt++;
        } else if (cnt >= 11 && cnt < 14) {                             // break between chars
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
            cnt++;
        } else if (cnt >= 14 && cnt < 17) {                             // K dash
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
            cnt++;
        } else if (cnt >= 17 && cnt < 18) {                             // K dot
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
            cnt++;
        } else if (cnt >= 18 && cnt < 20) {                             // K dot
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
            GPIO_toggle(CONFIG_GPIO_LED_0);
            cnt++;
        } else if (cnt >= 20 && cnt < 21) {                             // K dot
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            cnt++;
        } else if (cnt >= 21 && cnt < 24) {                             // K dash
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
            cnt++;
        } else if (cnt >= 24 && cnt < 31) {                             //  end break between "words"
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
            cnt++;
        } else {
            cnt = 0;
            btnFlagR = 0;
        }
        break;

    default:
        break;
    }
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Call driver init functions */
    GPIO_init();
    initTimer();

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_LED_1, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Turn on user LED */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    /*
     *  If more than one input pin is available for your device, interrupts
     *  will be enabled on CONFIG_GPIO_BUTTON1.
     */
    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1) {
        /* Configure BUTTON1 pin */
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

        /* Install Button callback */
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
    }

    /* Initializes STATE variable */
    MC_State = MC_START;

    /* Main loop */
    while (1) {
        /* Calls State Machine function */
        morseCodeStateMachine();
        /* While not done with timer */
        while(!TimerFlag){}
        /* Resets timer to FALSE */
        TimerFlag = 0;
    }

    return (NULL);
}
