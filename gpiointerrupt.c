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
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>

/* Driver configuration */
#include "ti_drivers_config.h"
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/Timer.h>

// UART Global Variables
char output[64];
int bytesToSend;

// Driver Handles - Global variables
UART_Handle uart;

#define DISPLAY(x) UART_write(uart, &output, x);

void initUART(void)
{
    UART_Params uartParams;
// Init the driver
    UART_init();
// Configure the driver
    UART_Params_init(&uartParams);

    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.baudRate = 115200;
// Open the driver
    uart = UART_open(CONFIG_UART_0, &uartParams);
    if (uart == NULL)
    {
        /* UART_open() failed */
        while (1)
            ;
    }
}

// I2C Global Variables
static const struct
{
    uint8_t address;
    uint8_t resultReg;
    char *id;
}

sensors[3] = { { 0x48, 0x0000, "11X" }, { 0x49, 0x0000, "116" }, { 0x41,
                                                                     0x0001,
                                                                     "006" } };
uint8_t txBuffer[1];
uint8_t rxBuffer[2];
I2C_Transaction i2cTransaction;

// Driver Handles - Global variables
I2C_Handle i2c;
// call initUART() before calling initI2C
void initI2C(void)
{
    int8_t i, found;
    I2C_Params i2cParams;
    DISPLAY(snprintf(output, 64, "Initializing I2C Driver - "))
// Init the driver
    I2C_init();
// Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
// Open the driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2c == NULL)
    {
        DISPLAY(snprintf(output, 64, "Failed\n\r"))
        while (1)
            ;
    }
    DISPLAY(snprintf(output, 32, "Passed\n\r"))
// Boards were shipped with different sensors.
// Try to determine which sensor we have.
// Scan through the possible sensor addresses
    /* Common I2C transaction setup */
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;
    found = false;
    for (i = 0; i < 3; ++i){
        i2cTransaction.slaveAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;
        DISPLAY(snprintf(output, 64, "Is this %s? ", sensors[i].id))
        if (I2C_transfer(i2c, &i2cTransaction)){
            DISPLAY(snprintf(output, 64, "Found\n\r"))
            found = true;
            break;
        }
        DISPLAY(snprintf(output, 64, "No\n\r"))
    }
    if (found) {
        DISPLAY(snprintf(output, 64, "Detected TMP%s I2C address: %x\n\r",
                         sensors[i].id, i2cTransaction.slaveAddress))
    }
    else {
        DISPLAY(snprintf(output, 64,
                         "Temperature sensor not found, contact professor\n\r"))
    }
}

int16_t readTemp(void) {
    int16_t temperature = 0;
    i2cTransaction.readCount = 2;
    if (I2C_transfer(i2c, &i2cTransaction)) {
        /*
         * * Extract degrees C from the received data;
         * * see TMP sensor datasheet
         * */
        temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
        temperature *= 0.0078125;
        /*
         * * If the MSB is set '1', then we have a 2's complement
         * * negative value which needs to be sign extended
         * */
        if (rxBuffer[0] & 0x80) {
            temperature |= 0xF000;
        }
    }
    else {
        DISPLAY(snprintf(output, 64,
                         "Error reading temperature sensor (%d)\n\r",
                         i2cTransaction.status))
        DISPLAY(snprintf(
                output,
                64,
                "Please power cycle your board by unplugging USB and plugging back in.\n\r"))

    }
    return temperature;
}

// Driver Handles - Global variables
Timer_Handle timer0;
volatile unsigned char TimerFlag = 0;
unsigned long setpoint = 25; // Start the thermostat at 25.
unsigned char direction = 0; // 0 is nothing, 1 is up, 2 is down.
unsigned char wasPushed = 0;

void timerCallback(Timer_Handle myHandle, int_fast16_t status) {
    // check if button was pressed and set thermostat.
    if (direction == 1) {
        setpoint++;
        printf("Thermostat now set to: %d\n\r", setpoint);
        wasPushed = 0;
        direction = 0;
    }
    else if (direction == 2) {
        setpoint--;
        printf("Thermostat now set to: %d\n\r", setpoint);
        wasPushed = 0;
        direction = 0;
    }

    TimerFlag = 1;
}

unsigned long timerPeriod = 1000;
void initTimer(void) {
    Timer_Params params;
// Init the driver
    Timer_init();
// Configure the driver
    Timer_Params_init(&params);
    params.period = timerPeriod;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;
// Open the driver
    timer0 = Timer_open(CONFIG_TIMER_0, &params);
    if (timer0 == NULL) {
        /* Failed to initialized timer */
        while (1) {
        }
    }
    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
        /* Failed to start timer */
        while (1) {
        }
    }
}

/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn0(uint_least8_t index) {
    printf("Down button was pushed.\n\r");
    wasPushed = 2;
}

/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_1.
 *  This may not be used for all boards.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn1(uint_least8_t index) {
    printf("Up button was pushed.\n\r");
    wasPushed = 1;
}

void changeThermostat() {
    direction = wasPushed;
}

enum LED_States {
    LED_START, LED_ON, LED_OFF
} LED_State;
unsigned char heat = 0;

void updateLED(temperature) {
    switch (LED_State) {
    case LED_START:
        LED_State = LED_OFF;
        break;
    case LED_ON:
        if (temperature >= setpoint) {
            LED_State = LED_OFF;
            heat = 0;
        }
        else if (temperature < setpoint) {
            LED_State = LED_ON;
            heat = 1;
        }
        break;
    case LED_OFF:
        if (temperature >= setpoint) {
            LED_State = LED_OFF;
            heat = 0;
        }
        else if (temperature < setpoint) {
            LED_State = LED_ON;
            heat = 1;
        }
        break;
    }

    switch (LED_State) {
    case LED_START:
        break;
    case LED_ON:
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
        break;
    case LED_OFF:
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
        break;
    }

}

/*
 *  ======== mainThread ========
 */
void* mainThread(void *arg0) {
    /* Call driver init functions */
    GPIO_init();

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0,
                   GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

//    /* Turn on user LED */
//    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

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
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1,
        GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

        /* Install Button callback */
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
    }

    // Set length of time that must pass before desired actions
    unsigned long changeThermostat_elapsedTime = 2000;
    unsigned long readTemp_elapsedTime = 5000;
    unsigned long sendTemp_elapsedTime = 1000000;
    // track how long since last board reset.
    unsigned long microseconds = 0;

    initUART();
    initI2C();
    initTimer();
    LED_State = LED_START;

    while (1) {
        // Every 200ms check button flags
        if (changeThermostat_elapsedTime >= 2000) {
            changeThermostat();
            changeThermostat_elapsedTime = 0;
        }

        // Every 500 ms read temp and update LED
        if (readTemp_elapsedTime >= 5000) {
            updateLED(readTemp());
            readTemp_elapsedTime = 0;
        }

        // Every second, output the following to the UART
        if (sendTemp_elapsedTime >= 1000000) {
            DISPLAY(snprintf(output, 64, "<%02d, %02d, %d, %04d>\n\r", readTemp(), setpoint, heat, microseconds / 1000000))
            sendTemp_elapsedTime = 0;
        }

        // configure the timer period
        while (!TimerFlag) {
        }
        TimerFlag = 0;
        changeThermostat_elapsedTime += timerPeriod;
        readTemp_elapsedTime += timerPeriod;
        sendTemp_elapsedTime += timerPeriod;
        microseconds += timerPeriod;
    }

}
