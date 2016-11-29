/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#include "fsl_debug_console.h"
#include "board.h"
#include "fsl_adc16.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_gpio.h"
#include "math.h"
#include "fsl_mma.h"
#include "fsl_i2c.h"
#include "fsl_tpm.h"
#include "fsl_common.h"
#include "fsl_port.h"
#include "fsl_uart.h"



/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_UART UART2
#define DEMO_UART_CLKSRC UART2_CLK_SRC

#define DEMO_ADC16_BASE ADC0
#define DEMO_ADC16_CHANNEL_GROUP 0U
#define DEMO_ADC16_USER_CHANNEL 1U /* PTE16, A0-ADC0_SE1, J4-2 on FRDM-KL27Z. */

#define BOARD_LED_GPIO BOARD_LED_RED_GPIO
#define BOARD_LED_GPIO_PIN BOARD_LED_RED_GPIO_PIN

#define BOARD_LED_GPIO BOARD_LED_GREEN_GPIO
#define BOARD_LED_GPIO_PIN BOARD_LED_GREEN_GPIO_PIN
#define BOARD_LED_GPIO BOARD_LED_BLUE_GPIO
#define BOARD_LED_GPIO_PIN BOARD_LED_BLUE_GPIO_PIN



#define BOARD_TIMER_BASEADDR TPM2
#define BOARD_FIRST_TIMER_CHANNEL 0U
#define BOARD_SECOND_TIMER_CHANNEL 1U
/* Get source clock for TPM driver */
#define BOARD_TIMER_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_McgIrc48MClk)
#define TIMER_CLOCK_MODE 1U
/* I2C source clock */
#define ACCEL_I2C_CLK_SRC I2C0_CLK_SRC
#define I2C_BAUDRATE 100000U

#define I2C_RELEASE_SDA_PORT PORTD
#define I2C_RELEASE_SCL_PORT PORTD
#define I2C_RELEASE_SDA_GPIO GPIOD
#define I2C_RELEASE_SDA_PIN 6U
#define I2C_RELEASE_SCL_GPIO GPIOD
#define I2C_RELEASE_SCL_PIN 7U
#define I2C_RELEASE_BUS_COUNT 100U
/* Upper bound and lower bound angle values */
#define ANGLE_UPPER_BOUND 85U
#define ANGLE_LOWER_BOUND 5U
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void BOARD_I2C_ReleaseBus(void);
/*******************************************************************************
 * Variables
 ******************************************************************************/
int lectura_adc=0;

uint8_t txbuff[] = "Uart polling example\r\nBoard will send back received characters\r\n";
uint8_t rxbuff[20] = {0};
//int aceleracion_x=0;



i2c_master_handle_t g_MasterHandle;
/* MMA8451 device address */
const uint8_t g_accel_address[] = {0x1CU, 0x1DU, 0x1EU, 0x1FU};
/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */
void bluetooth(int msg)
{
    uint8_t ch=msg;
    uart_config_t config;

    BOARD_InitBth();
    BOARD_BootClockRUN();

    /*
     * config.baudRate_Bps = 115200U;
     * config.parityMode = kUART_ParityDisabled;
     * config.stopBitCount = kUART_OneStopBit;
     * config.txFifoWatermark = 0;
     * config.rxFifoWatermark = 1;
     * config.enableTx = false;
     * config.enableRx = false;
     */
    UART_GetDefaultConfig(&config);
    config.baudRate_Bps = BOARD_DEBUG_UART_BAUDRATE;
    config.enableTx = true;
    config.enableRx = true;

    UART_Init(DEMO_UART, &config, CLOCK_GetFreq(DEMO_UART_CLKSRC));

    //UART_WriteBlocking(DEMO_UART, txbuff, sizeof(txbuff) - 1);


        //UART_ReadBlocking(DEMO_UART, &ch, 1);

        UART_WriteBlocking(DEMO_UART, &ch, 1);



}

void inicializar_leds(void)
{
      gpio_pin_config_t Redled_config = {kGPIO_DigitalOutput, 1U, };
	  gpio_pin_config_t Blueled_config = {kGPIO_DigitalOutput, 1U, };
	  gpio_pin_config_t Greenled_config = {kGPIO_DigitalOutput, 1U, };
	  BOARD_InitPins();
	  BOARD_InitLEDs();
	  BOARD_BootClockRUN();
	  BOARD_InitDebugConsole();

	  /* Add your code here */
	  PRINTF("\r\n GPIO Driver example\r\n");
	  PRINTF("\r\n The LED is taking turns to shine.\r\n");
	  GPIO_PinInit(BOARD_LED_RED_GPIO, BOARD_LED_RED_GPIO_PIN, &Redled_config);
	  GPIO_PinInit(BOARD_LED_GREEN_GPIO, BOARD_LED_GREEN_GPIO_PIN, &Greenled_config);
	  GPIO_PinInit(BOARD_LED_BLUE_GPIO, BOARD_LED_BLUE_GPIO_PIN, &Blueled_config);
}

 void delay(void) {
	volatile uint32_t i=0;
	for(i=0;i<800000;++i){
		__asm("NOP");
	}
}
int leer_adc(void)
{
	adc16_config_t adc16ConfigStruct;
	    adc16_channel_config_t adc16ChannelConfigStruct;

	    BOARD_InitPins();
	    BOARD_BootClockRUN();
	    BOARD_InitDebugConsole();
	    /*
	     * adc16ConfigStruct.referenceVoltageSource = kADC16_ReferenceVoltageSourceVref;
	     * adc16ConfigStruct.clockSource = kADC16_ClockSourceAsynchronousClock;
	     * adc16ConfigStruct.enableAsynchronousClock = true;
	     * adc16ConfigStruct.clockDivider = kADC16_ClockDivider8;
	     * adc16ConfigStruct.resolution = kADC16_ResolutionSE12Bit;
	     * adc16ConfigStruct.longSampleMode = kADC16_LongSampleDisabled;
	     * adc16ConfigStruct.enableHighSpeed = false;
	     * adc16ConfigStruct.enableLowPower                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       = false;
	     * adc16ConfigStruct.enableContinuousConversion = false;
	     */
	    ADC16_GetDefaultConfig(&adc16ConfigStruct);
	    ADC16_Init(DEMO_ADC16_BASE, &adc16ConfigStruct);
	    ADC16_EnableHardwareTrigger(DEMO_ADC16_BASE, false); /* Make sure the software trigger is used. */
	#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
	    if (kStatus_Success == ADC16_DoAutoCalibration(DEMO_ADC16_BASE))
	    {
	        //PRINTF("ADC16_DoAutoCalibration() Done.\r\n");
	    }
	    else
	    {
	        //PRINTF("ADC16_DoAutoCalibration() Failed.\r\n");
	    }
	#endif /* FSL_FEATURE_ADC16_HAS_CALIBRATION */


	    adc16ChannelConfigStruct.channelNumber = DEMO_ADC16_USER_CHANNEL;
	    adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = false;
	#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
	    adc16ChannelConfigStruct.enableDifferentialConversion = false;
	#endif /* FSL_FEATURE_ADC16_HAS_DIFF_MODE */

	      //GETCHAR();
	        /*
	         When in software trigger mode, each conversion would be launched once calling the "ADC16_ChannelConfigure()"
	         function, which works like writing a conversion command and executing it. For another channel's conversion,
	         just to change the "channelNumber" field in channel's configuration structure, and call the
	         "ADC16_ChannelConfigure() again.
	        */
	        ADC16_SetChannelConfig(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct);
	        while (0U == (kADC16_ChannelConversionDoneFlag &
	                      ADC16_GetChannelStatusFlags(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP)))
	        {
	        }
	        lectura_adc=ADC16_GetChannelConversionValue(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP);
	        delay();


}


void led_rojo(int state)
{

 if(state==1)
 {
	 GPIO_WritePinOutput(BOARD_LED_RED_GPIO, BOARD_LED_RED_GPIO_PIN, 0);
	 PRINTF("ON ROJO \n");
 }
 else
 {
	 GPIO_WritePinOutput(BOARD_LED_RED_GPIO, BOARD_LED_RED_GPIO_PIN, 1U);
	 PRINTF("OFF ROJO \n");
 }
}
void led_verde(int state)
{

 if(state==1)
 {
	 GPIO_WritePinOutput(BOARD_LED_GREEN_GPIO,BOARD_LED_GREEN_GPIO_PIN,0);
	 PRINTF("ON VERDE \n");
 }
 else
 {
	 GPIO_WritePinOutput(BOARD_LED_GREEN_GPIO,BOARD_LED_GREEN_GPIO_PIN,1U);
	 PRINTF("OFF VERDE \n");
 }
}
void led_amarillo(int state)


{

 if(state==1)
 {
	 GPIO_WritePinOutput(BOARD_LED_RED_GPIO, BOARD_LED_RED_GPIO_PIN, 0);
	 GPIO_WritePinOutput(BOARD_LED_GREEN_GPIO,BOARD_LED_GREEN_GPIO_PIN,0);
	 PRINTF("ON AMARILLO \n");
 }
 else
 {
	 GPIO_WritePinOutput(BOARD_LED_RED_GPIO, BOARD_LED_RED_GPIO_PIN, 1U);
	 GPIO_WritePinOutput(BOARD_LED_GREEN_GPIO,BOARD_LED_GREEN_GPIO_PIN,1U);
	 PRINTF("OFF AMARILLO \n");
 }
}

void nivel_distancia(void)
{
		while(lectura_adc<1400)
	    {
		leer_adc();
		PRINTF("ADC Value: %d\r\n",lectura_adc);
	    }
		led_rojo(1);
		while(lectura_adc>=1000 || lectura_adc<900)
		{
		leer_adc();
		PRINTF("ADC Value: %d\r\n",lectura_adc);
		}
		led_rojo(0);
		delay();
		led_amarillo(1);
		while(lectura_adc>=200)
		{
		leer_adc();
		PRINTF("ADC Value: %d\r\n",lectura_adc);
		}
		led_amarillo(0);
		delay();
		led_verde(1);
		PRINTF("LISTO NIVEL DISTANCIA! \n");
		delay();
		led_verde(0);
}


static void i2c_release_bus_delay(void)
{
    uint32_t i = 0;
    for (i = 0; i < I2C_RELEASE_BUS_COUNT; i++)
    {
        __NOP();
    }
}



void BOARD_I2C_ReleaseBus(void)
{
    uint8_t i = 0;
    gpio_pin_config_t pin_config;
    port_pin_config_t i2c_pin_config = {0};

    /* Config pin mux as gpio */
    i2c_pin_config.pullSelect = kPORT_PullUp;
    i2c_pin_config.mux = kPORT_MuxAsGpio;

    pin_config.pinDirection = kGPIO_DigitalOutput;
    pin_config.outputLogic = 1U;
    PORT_SetPinConfig(I2C_RELEASE_SCL_PORT, I2C_RELEASE_SCL_PIN, &i2c_pin_config);
    PORT_SetPinConfig(I2C_RELEASE_SDA_PORT, I2C_RELEASE_SDA_PIN, &i2c_pin_config);

    GPIO_PinInit(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, &pin_config);
    GPIO_PinInit(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, &pin_config);

    /* Drive SDA low first to simulate a start */
    GPIO_WritePinOutput(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 0U);
    i2c_release_bus_delay();

    /* Send 9 pulses on SCL and keep SDA high */
    for (i = 0; i < 9; i++)
    {
        GPIO_WritePinOutput(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 0U);
        i2c_release_bus_delay();

        GPIO_WritePinOutput(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 1U);
        i2c_release_bus_delay();

        GPIO_WritePinOutput(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 1U);
        i2c_release_bus_delay();
        i2c_release_bus_delay();
    }

    /* Send stop */
    GPIO_WritePinOutput(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 0U);
    i2c_release_bus_delay();

    GPIO_WritePinOutput(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 0U);
    i2c_release_bus_delay();

    GPIO_WritePinOutput(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 1U);
    i2c_release_bus_delay();

    GPIO_WritePinOutput(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 1U);
    i2c_release_bus_delay();
}
/* Initialize timer module */
static void Timer_Init(void)
{
    /* convert to match type of data */
    tpm_config_t tpmInfo;
    tpm_chnl_pwm_signal_param_t tpmParam[2];

    /* Configure tpm params with frequency 24kHZ */
    tpmParam[0].chnlNumber = (tpm_chnl_t)BOARD_FIRST_TIMER_CHANNEL;
    tpmParam[0].level = kTPM_LowTrue;
    tpmParam[0].dutyCyclePercent = 0U;

    tpmParam[1].chnlNumber = (tpm_chnl_t)BOARD_SECOND_TIMER_CHANNEL;
    tpmParam[1].level = kTPM_LowTrue;
    tpmParam[1].dutyCyclePercent = 0U;

    /* Initialize TPM module */
    /*
     * tpmInfo.prescale = kTPM_Prescale_Divide_1;
     * tpmInfo.useGlobalTimeBase = false;
     * tpmInfo.enableDoze = false;
     * tpmInfo.enableDebugMode = false;
     * tpmInfo.enableReloadOnTrigger = false;
     * tpmInfo.enableStopOnOverflow = false;
     * tpmInfo.enableStartOnTrigger = false;
     * tpmInfo.enablePauseOnTrigger = false;
     * tpmInfo.triggerSelect = kTPM_Trigger_Select_0;
     * tpmInfo.triggerSource = kTPM_TriggerSource_External;
     */
    TPM_GetDefaultConfig(&tpmInfo);
    TPM_Init(BOARD_TIMER_BASEADDR, &tpmInfo);

    CLOCK_SetTpmClock(TIMER_CLOCK_MODE);

    TPM_SetupPwm(BOARD_TIMER_BASEADDR, tpmParam, 2U, kTPM_EdgeAlignedPwm, 24000U, BOARD_TIMER_SOURCE_CLOCK);
    TPM_StartTimer(BOARD_TIMER_BASEADDR, kTPM_SystemClock);
}

/* Update the duty cycle of an active pwm signal */
static void Board_UpdatePwm(uint16_t x, uint16_t y)
{
    /* Updated duty cycle */
    TPM_UpdatePwmDutycycle(BOARD_TIMER_BASEADDR, (tpm_chnl_t)BOARD_FIRST_TIMER_CHANNEL, kTPM_EdgeAlignedPwm, x);
    TPM_UpdatePwmDutycycle(BOARD_TIMER_BASEADDR, (tpm_chnl_t)BOARD_SECOND_TIMER_CHANNEL, kTPM_EdgeAlignedPwm, y);
}


int leer_acelerometro_y(void)
{
	    mma_handle_t mmaHandle = {0};
	    mma_data_t sensorData = {0};
	    i2c_master_config_t i2cConfig = {0};
	    uint8_t sensorRange = 0;
	    uint8_t dataScale = 0;
	    uint32_t i2cSourceClock = 0;
	    int16_t xData = 0;
	    int16_t yData = 0;
	    int16_t xAngle = 0;
	    int16_t yAngle = 0;
	    uint8_t i = 0;
	    uint8_t regResult = 0;
	    uint8_t array_addr_size = 0;
	    bool foundDevice = false;

	    /* Board pin, clock, debug console init */
	    //BOARD_InitPins();
	    BOARD_BootClockRUN();
	    BOARD_I2C_ReleaseBus();
	    BOARD_I2C_ConfigurePins();
	    BOARD_InitDebugConsole();

	    i2cSourceClock = CLOCK_GetFreq(ACCEL_I2C_CLK_SRC);
	    mmaHandle.base = BOARD_ACCEL_I2C_BASEADDR;
	    mmaHandle.i2cHandle = &g_MasterHandle;

	    /*
	     * i2cConfig.baudRate_Bps = 100000U;
	     * i2cConfig.enableHighDrive = false;
	     * i2cConfig.enableStopHold = false;
	     * i2cConfig.glitchFilterWidth = 0U;
	     * i2cConfig.enableMaster = true;
	     */
	    I2C_MasterGetDefaultConfig(&i2cConfig);
	    I2C_MasterInit(BOARD_ACCEL_I2C_BASEADDR, &i2cConfig, i2cSourceClock);
	    I2C_MasterTransferCreateHandle(BOARD_ACCEL_I2C_BASEADDR, &g_MasterHandle, NULL, NULL);

	    /* Find sensor devices */
	    array_addr_size = sizeof(g_accel_address) / sizeof(g_accel_address[0]);
	    for (i = 0; i < array_addr_size; i++)
	    {
	        mmaHandle.xfer.slaveAddress = g_accel_address[i];
	        if (MMA_ReadReg(&mmaHandle, kMMA8451_WHO_AM_I, &regResult) == kStatus_Success)
	        {
	            foundDevice = true;
	            break;
	        }
	        if ((i == (array_addr_size - 1)) && (!foundDevice))
	        {
	            PRINTF("\r\nDo not found sensor device\r\n");
	            while (1)
	            {
	            };
	        }
	    }

	    /* Init accelerometer sensor */
	    if (MMA_Init(&mmaHandle) != kStatus_Success)
	    {
	        return -1;
	    }

	    /* Get sensor range */
	    if (MMA_ReadReg(&mmaHandle, kMMA8451_XYZ_DATA_CFG, &sensorRange) != kStatus_Success)
	    {
	        return -1;
	    }
	    if (sensorRange == 0x00)
	    {
	        dataScale = 2U;
	    }
	    else if (sensorRange == 0x01)
	    {
	        dataScale = 4U;
	    }
	    else if (sensorRange == 0x10)
	    {
	        dataScale = 8U;
	    }
	    else
	    {
	    }
	    /* Init timer */
	    Timer_Init();

	    /* Print a note to terminal */
	    PRINTF("\r\nWelcome to BUBBLE example\r\n");
	    PRINTF("\r\nYou will see the change of LED brightness when change angles of board\r\n");

	    /* Main loop. Get sensor data and update duty cycle */
	   // while (1)
	    //{
	        /* Get new accelerometer data. */
	        if (MMA_ReadSensorData(&mmaHandle, &sensorData) != kStatus_Success)
	        {
	            return -1;
	        }

	        /* Get the X and Y data from the sensor data structure in 14 bit left format data*/
	        xData = (int16_t)((uint16_t)((uint16_t)sensorData.accelXMSB << 8) | (uint16_t)sensorData.accelXLSB) / 4U;
	        yData = (int16_t)((uint16_t)((uint16_t)sensorData.accelYMSB << 8) | (uint16_t)sensorData.accelYLSB) / 4U;

	        /* Convert raw data to angle (normalize to 0-90 degrees). No negative angles. */
	        xAngle = (int16_t)floor((double)xData * (double)dataScale * 90 / 8192);
	        if (xAngle < 0)
	        {
	            xAngle *= -1;
	        }
	        yAngle = (int16_t)floor((double)yData * (double)dataScale * 90 / 8192);
	        if (yAngle < 0)
	        {
	            yAngle *= -1;
	        }
	        /* Update angles to turn on LEDs when angles ~ 90 */
	        if (xAngle > ANGLE_UPPER_BOUND)
	        {
	            xAngle = 100;
	        }
	        if (yAngle > ANGLE_UPPER_BOUND)
	        {
	            yAngle = 100;
	        }
	        /* Update angles to turn off LEDs when angles ~ 0 */
	        if (xAngle < ANGLE_LOWER_BOUND)
	        {
	            xAngle = 0;
	        }
	        if (yAngle < ANGLE_LOWER_BOUND)
	        {
	            yAngle = 0;
	        }

	        Board_UpdatePwm(xAngle, yAngle);

	        /* Print out the raw accelerometer data. */
	        //PRINTF("x= %6d y = %6d\r\n", xData, yData);
	        //return xData;
	        return yData;
	        //delay();
	    //}
}

int leer_acelerometro_x(void)
{
	    mma_handle_t mmaHandle = {0};
	    mma_data_t sensorData = {0};
	    i2c_master_config_t i2cConfig = {0};
	    uint8_t sensorRange = 0;
	    uint8_t dataScale = 0;
	    uint32_t i2cSourceClock = 0;
	    int16_t xData = 0;
	    int16_t yData = 0;
	    int16_t xAngle = 0;
	    int16_t yAngle = 0;
	    uint8_t i = 0;
	    uint8_t regResult = 0;
	    uint8_t array_addr_size = 0;
	    bool foundDevice = false;

	    /* Board pin, clock, debug console init */
	    //BOARD_InitPins();
	    BOARD_BootClockRUN();
	    BOARD_I2C_ReleaseBus();
	    BOARD_I2C_ConfigurePins();
	    BOARD_InitDebugConsole();

	    i2cSourceClock = CLOCK_GetFreq(ACCEL_I2C_CLK_SRC);
	    mmaHandle.base = BOARD_ACCEL_I2C_BASEADDR;
	    mmaHandle.i2cHandle = &g_MasterHandle;

	    /*
	     * i2cConfig.baudRate_Bps = 100000U;
	     * i2cConfig.enableHighDrive = false;
	     * i2cConfig.enableStopHold = false;
	     * i2cConfig.glitchFilterWidth = 0U;
	     * i2cConfig.enableMaster = true;
	     */
	    I2C_MasterGetDefaultConfig(&i2cConfig);
	    I2C_MasterInit(BOARD_ACCEL_I2C_BASEADDR, &i2cConfig, i2cSourceClock);
	    I2C_MasterTransferCreateHandle(BOARD_ACCEL_I2C_BASEADDR, &g_MasterHandle, NULL, NULL);

	    /* Find sensor devices */
	    array_addr_size = sizeof(g_accel_address) / sizeof(g_accel_address[0]);
	    for (i = 0; i < array_addr_size; i++)
	    {
	        mmaHandle.xfer.slaveAddress = g_accel_address[i];
	        if (MMA_ReadReg(&mmaHandle, kMMA8451_WHO_AM_I, &regResult) == kStatus_Success)
	        {
	            foundDevice = true;
	            break;
	        }
	        if ((i == (array_addr_size - 1)) && (!foundDevice))
	        {
	            PRINTF("\r\nDo not found sensor device\r\n");
	            while (1)
	            {
	            };
	        }
	    }

	    /* Init accelerometer sensor */
	    if (MMA_Init(&mmaHandle) != kStatus_Success)
	    {
	        return -1;
	    }

	    /* Get sensor range */
	    if (MMA_ReadReg(&mmaHandle, kMMA8451_XYZ_DATA_CFG, &sensorRange) != kStatus_Success)
	    {
	        return -1;
	    }
	    if (sensorRange == 0x00)
	    {
	        dataScale = 2U;
	    }
	    else if (sensorRange == 0x01)
	    {
	        dataScale = 4U;
	    }
	    else if (sensorRange == 0x10)
	    {
	        dataScale = 8U;
	    }
	    else
	    {
	    }
	    /* Init timer */
	    Timer_Init();

	    /* Print a note to terminal */
	    PRINTF("\r\nWelcome to BUBBLE example\r\n");
	    PRINTF("\r\nYou will see the change of LED brightness when change angles of board\r\n");

	    /* Main loop. Get sensor data and update duty cycle */
	   // while (1)
	    //{
	        /* Get new accelerometer data. */
	        if (MMA_ReadSensorData(&mmaHandle, &sensorData) != kStatus_Success)
	        {
	            return -1;
	        }

	        /* Get the X and Y data from the sensor data structure in 14 bit left format data*/
	        xData = (int16_t)((uint16_t)((uint16_t)sensorData.accelXMSB << 8) | (uint16_t)sensorData.accelXLSB) / 4U;
	        yData = (int16_t)((uint16_t)((uint16_t)sensorData.accelYMSB << 8) | (uint16_t)sensorData.accelYLSB) / 4U;

	        /* Convert raw data to angle (normalize to 0-90 degrees). No negative angles. */
	        xAngle = (int16_t)floor((double)xData * (double)dataScale * 90 / 8192);
	        if (xAngle < 0)
	        {
	            xAngle *= -1;
	        }
	        yAngle = (int16_t)floor((double)yData * (double)dataScale * 90 / 8192);
	        if (yAngle < 0)
	        {
	            yAngle *= -1;
	        }
	        /* Update angles to turn on LEDs when angles ~ 90 */
	        if (xAngle > ANGLE_UPPER_BOUND)
	        {
	            xAngle = 100;
	        }
	        if (yAngle > ANGLE_UPPER_BOUND)
	        {
	            yAngle = 100;
	        }
	        /* Update angles to turn off LEDs when angles ~ 0 */
	        if (xAngle < ANGLE_LOWER_BOUND)
	        {
	            xAngle = 0;
	        }
	        if (yAngle < ANGLE_LOWER_BOUND)
	        {
	            yAngle = 0;
	        }

	        Board_UpdatePwm(xAngle, yAngle);

	        /* Print out the raw accelerometer data. */
	        //PRINTF("x= %6d y = %6d\r\n", xData, yData);
	        //return xData;
	        return xData;
	        //delay();
	    //}
}

void nivel_aceleracion_1(void)
{
	int contador=0;
	int aceleracion_y=0;

	while (contador<=2)
	{
		while (aceleracion_y>=-1500)
			{
				aceleracion_y=leer_acelerometro_y();
				PRINTF("%d \n",aceleracion_y);

			}
		if(contador==0)
		{
			led_rojo(1);
		} else if (contador==1)
		{
			led_amarillo(1);
		}
		else
		{
			led_verde(1);
		}
			while (aceleracion_y<=-100)
				{
					aceleracion_y=leer_acelerometro_y();
					PRINTF("%d \n",aceleracion_y);
					//delay();
				}
			if(contador==0)
					{
						led_rojo(0);
					} else if (contador==1)
					{
						led_amarillo(0);
					}
					else
					{
						led_verde(0);
					}
			contador=contador+1;
			PRINTF("%d",contador);
	}
}

void nivel_aceleracion_2(void)
{
	int contador=0;
	int aceleracion_x=0;

		while (contador<=2)
		{
			while (aceleracion_x>=-1800)
				{
					aceleracion_x=leer_acelerometro_x();
					PRINTF("%d \n",aceleracion_x);

				}
			if(contador==0)
			{
				led_rojo(1);
			} else if (contador==1)
			{
				led_amarillo(1);
			}
			else
			{
				led_verde(1);
			}
				while (aceleracion_x<=1800)
					{
						aceleracion_x=leer_acelerometro_x();
						PRINTF("%d \n",aceleracion_x);
						//delay();
					}
				if(contador==0)
						{
							led_rojo(0);
						} else if (contador==1)
						{
							led_amarillo(0);
						}
						else
						{
							led_verde(0);
						}
				contador=contador+1;
				PRINTF("%d",contador);
		}

}

void nivel_joystick(void)
{

}

int main(void)
{

	/*bluetooth('\r\n+STWMOD=0\r\n');
	bluetooth('r\n+STPIN=1234\r\n');
	bluetooth('\r\n+INQ=1\r\n');
	bluetooth('\r\n+STAUTO=1\r\n');
	*/
	bluetooth('e');
	inicializar_leds();

	//bluetooth('e');
	nivel_aceleracion_2();

	bluetooth('e');
	nivel_aceleracion_1();
	bluetooth('e');
	nivel_distancia();
	bluetooth('e');

	//while(1)
	//{
	//	bluetooth('e');
	//	delay();
	//}
}
