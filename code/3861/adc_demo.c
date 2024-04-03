/*
 * Copyright (C) 2022 HiHope Open Source Organization .
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http:// www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 *
 * limitations under the License.
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>

#include "ohos_init.h"
#include "cmsis_os2.h"
#include "iot_i2c.h"
#include "iot_gpio_ex.h"
#include "iot_gpio.h"
#include "iot_adc.h"
#include "iot_errno.h"
#include "iot_uart.h"
#include "hi_uart.h"
#include "iot_watchdog.h"
#include "iot_pwm.h"
#include "hi_io.h"


#define NUM 1
#define STACK_SIZE 4096
#define AHT20_BAUDRATE (400 * 1000)
#define AHT20_I2C_IDX 0
#define UART_BUFF_SIZE 100
//#define LED_TEST_GPIO 9
#define RED_LED_PIN_NAME 10
#define RED_LED_PIN_FUNCTION WIFI_IOT_IO_FUNC_GPIO_10_GPIO
#define PWM_FREQ_DIVITION 64000

void Uart2GpioInit(void)
{
    IoTGpioInit(IOT_IO_NAME_GPIO_12);
    // 设置GPIO0的管脚复用关系为UART1_TX Set the pin reuse relationship of GPIO0 to UART1_ TX
    IoSetFunc(IOT_IO_NAME_GPIO_12, IOT_IO_FUNC_GPIO_12_UART2_RXD);
    IoTGpioInit(IOT_IO_NAME_GPIO_11);
    // 设置GPIO1的管脚复用关系为UART1_RX Set the pin reuse relationship of GPIO1 to UART1_ RX
    IoSetFunc(IOT_IO_NAME_GPIO_11, IOT_IO_FUNC_GPIO_11_UART2_TXD);
}

void Uart2Config(void)
{
    uint32_t ret;
    /* 初始化UART配置，波特率 9600，数据bit为8,停止位1，奇偶校验为NONE */
    /* Initialize UART configuration, baud rate is 9600, data bit is 8, stop bit is 1, parity is NONE */
    IotUartAttribute uart_attr = {
        .baudRate = 115200,
        .dataBits = 8,
        .stopBits = 1,
        .parity = 0,
    };
    ret = IoTUartInit(HI_UART_IDX_2, &uart_attr);
    if (ret != IOT_SUCCESS) {
        printf("Init Uart1 Falied Error No : %d\n", ret);
        return;
    }
}

static void ADCLightTask(int *arg)
{
    (void)arg;
    unsigned int ret = 0;
    unsigned int retlight = 0;
    unsigned short data = 0;
    float value = 0.0;
    unsigned int peoplecount = 0;
    unsigned short datalight = 0;
    float valuelight = 0.0;
    float sum = 0.0;
    unsigned int count = 1;
    
  
    const char *uartdata1 = "Some_People\n";
    const char *uartdata2 = "None_People\n";
    uint32_t len = 0;
    unsigned char uartReadBuff[UART_BUFF_SIZE] = {0};
    // 对UART2的一些初始化 Some initialization of UART2
    Uart2GpioInit();
    // 对UART2参数的一些配置 Some configurations of UART2 parameters
    Uart2Config();

    //初始化GPIO9
    //IoTGpioInit(LED_TEST_GPIO);
    //设置为输出
    //IoTGpioSetDir(LED_TEST_GPIO, IOT_GPIO_DIR_OUT);
    
    //炫彩灯板的红灯
    hi_io_set_func(10, 5);
    IoTPwmInit(1);

    OledInit();
    OledFillScreen(0);
    IoTI2cInit(AHT20_I2C_IDX, AHT20_BAUDRATE);
    while (NUM) {
        // 红外检测对应的是ADC channel 3，
        ret = AdcRead(IOT_ADC_CHANNEL_3, &data, IOT_ADC_EQU_MODEL_4, IOT_ADC_CUR_BAIS_DEFAULT, 0xff);
        if (ret != IOT_SUCCESS) {
            printf("ADC3-infrared Read Fail \r\n");
            return;
        } else {
            /* vlt * 1.8 * 4 / 4096.0 为将码字转换为电压 */
            value = (unsigned int)data;
            printf("ADC3-infrared Read value is  %.3f \r\n", value);
            int i;
            if (value > 950)
            {
                peoplecount = 0;
                i=1;
                OledShowString(20, 3, "Some_People", 1); /* 屏幕第20列3行显示1行 */
                // 通过UART1 发送数据 Send data through UART1
                IoTUartWrite(HI_UART_IDX_2, (unsigned char*)uartdata1, strlen(uartdata1));
                // 通过UART1 接收数据 Receive data through UART1
                //len = IoTUartRead(HI_UART_IDX_2, uartReadBuff, UART_BUFF_SIZE);
            }else{
                i=0;
                peoplecount++;
                OledShowString(20, 3, "None_People", 1); /* 屏幕第20列3行显示1行 */
                if (peoplecount%5==0)
                {
                    // 通过UART1 发送数据 Send data through UART1
                    IoTUartWrite(HI_UART_IDX_2, (unsigned char*)uartdata2, strlen(uartdata2));
                    // 通过UART1 接收数据 Receive data through UART1
                    //len = IoTUartRead(HI_UART_IDX_2, uartReadBuff, UART_BUFF_SIZE);
                }
            }
            printf("the i is %d \n",i);
        }
        
        // 光敏检测
        retlight = AdcRead(IOT_ADC_CHANNEL_4, &datalight, IOT_ADC_EQU_MODEL_4, IOT_ADC_CUR_BAIS_DEFAULT, 0xff);
        // set Red LED pin to GPIO function
        count++;
        IoTGpioInit(10); 
        if (retlight != IOT_SUCCESS) {
            printf("ADC4-photosensitive Read Fail \r\n");
            return;
        } else {
            /* vlt * 1.8 * 4 / 4096.0 为将码字转换为电压 */
            valuelight = (float)datalight * (1.8) * 4 /4096;
            printf("ADC-photosensitive Read value is %.3f \r\n", valuelight);
            sum+=valuelight;
            if(count%15 ==0){
                count = 1;
                sum = sum/15;
                if (sum < 0.5)
                {
                IoTPwmStart(1,1,PWM_FREQ_DIVITION);
                }else if(sum > 2.5){
                //输出低电平
                IoTPwmStart(1,99,PWM_FREQ_DIVITION);
                }else{
                IoTPwmStart(1,(sum-0.5)*99/2,PWM_FREQ_DIVITION);
                }
                sum = 0.0;
            }
        }
        //TaskMsleep(20); /* 20:sleep 20ms */
    }
}

static void ADCLightDemo(void)
{
    osThreadAttr_t attr;

    attr.name = "ADCLightTask";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = STACK_SIZE;
    attr.priority = osPriorityNormal;

    if (osThreadNew(ADCLightTask, NULL, &attr) == NULL) {
        printf("[ADCLightDemo] Failed to create ADCLightTask!\n");
    }
}

APP_FEATURE_INIT(ADCLightDemo);
