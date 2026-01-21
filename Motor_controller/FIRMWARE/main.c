#include "stm32f4xx.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* =====================  STATE MACHINE  ===================== */
typedef enum {
    ST_STOP = 0,
    ST_RUN_FORWARD,    // CW
    ST_RUN_REVERSE,    // CCW
		Idle
} motor_state_t;

volatile motor_state_t state = ST_STOP;

/* ===================== CONTROL VARIABLES ===================== */
#define RX_BUFFER_SIZE 32
volatile uint8_t rx_dma_buffer[RX_BUFFER_SIZE];   // DMA buffer
volatile uint8_t rx_msg_buffer[RX_BUFFER_SIZE];   // Processed message
volatile uint8_t rx_ready = 0;

volatile int target_rpm = 0;
volatile int measured_rpm = 0;
volatile int32_t dcnt = 0; 
volatile int current_pwm_duty = 0; 
volatile int need_test = 0;
volatile float value=0;
/* PID gains */
#define KP 2.3f
#define KI 0.5f
#define KD 0.001f

/* Function prototypes */
void GPIO_Config(void);
void TIM3_PWM_Config(void);
void TIM2_Encoder_Config(void);
void USART2_DMA_Config(void);
void SysTick_Handler(void);
void motor_state_machine(void);
void apply_pwm(int duty);
void check_dma_message(void);
void delay_ms(uint32_t ms);

/* =========================================================== */
int main(void)
{
    // Initialize peripherals
    GPIO_Config();
    TIM3_PWM_Config();
    TIM2_Encoder_Config();
    USART2_DMA_Config();

    SysTick_Config(SystemCoreClock / 1000); // 1 ms tick

    TIM3->CCR4 = 0;
    current_pwm_duty = 0;

    while(1)
    {
        check_dma_message(); // Poll DMA buffer
		
        if(rx_ready)
        {
            rx_ready = 0;
            value = atoi((char*)rx_msg_buffer);
            memset((void*)rx_msg_buffer, 0, RX_BUFFER_SIZE);

            // Limit value to -3000…3000 RPM
            if(value > 3000) value = 3000;
            if(value < -3000) value = -3000;

            // Set target RPM
            target_rpm = value;

            if(value > 0){
							state = Idle;
							state = ST_RUN_FORWARD;}
            else if(value < 0) {
							state = Idle;
							state =ST_RUN_REVERSE;
						}
            else state = ST_STOP;
        }

        motor_state_machine();

    }
}

/* ============================================================ */
void apply_pwm(int duty)
{
    if(duty < 0) duty = 0;
    if(duty > 4199) duty = 4199;

    TIM3->CCR4 = (uint32_t)duty;
    current_pwm_duty = duty;
}

/* ============================================================ */
void motor_state_machine(void)
{
    switch(state)
    {
        case ST_STOP:
            apply_pwm(1500);
						delay_ms(250);
						apply_pwm(0);
						GPIO_SetBits(GPIOD, GPIO_Pin_0 | GPIO_Pin_1);
            break;

        case ST_RUN_FORWARD:
            GPIO_SetBits(GPIOD, GPIO_Pin_1);
            GPIO_ResetBits(GPIOD, GPIO_Pin_0);
            break;

        case ST_RUN_REVERSE:
            GPIO_SetBits(GPIOD, GPIO_Pin_0);
            GPIO_ResetBits(GPIOD, GPIO_Pin_1);
            break;
				case Idle:
					apply_pwm(1700);
					delay_ms(150);
					apply_pwm(0);
					delay_ms(150);
				break;
    }
}

/* ============================================================ */
void USART2_DMA_Config(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_DMA1, ENABLE);

    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

    USART_InitTypeDef uart;
    uart.USART_BaudRate = 115200;
    uart.USART_WordLength = USART_WordLength_8b;
    uart.USART_StopBits = USART_StopBits_1;
    uart.USART_Parity = USART_Parity_No;
    uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    uart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART2, &uart);

    USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
    USART_Cmd(USART2, ENABLE);

    DMA_InitTypeDef dma;
    DMA_DeInit(DMA1_Stream5);
    dma.DMA_Channel = DMA_Channel_4;
    dma.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;
    dma.DMA_Memory0BaseAddr = (uint32_t)rx_dma_buffer;
    dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
    dma.DMA_BufferSize = RX_BUFFER_SIZE;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode = DMA_Mode_Circular;
    dma.DMA_Priority = DMA_Priority_High;
    dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
    dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream5, &dma);

    DMA_Cmd(DMA1_Stream5, ENABLE);
}

/* ============================================================ */
void check_dma_message(void)
{
    static uint16_t last_pos = 0;
    uint16_t pos = RX_BUFFER_SIZE - DMA_GetCurrDataCounter(DMA1_Stream5);

    while(last_pos != pos)
    {
        uint8_t b = rx_dma_buffer[last_pos];
        last_pos = (last_pos + 1) % RX_BUFFER_SIZE;

        if(b == '\r' || b == '\n')
        {
            if(rx_msg_buffer[0] != 0)
            {
                rx_ready = 1;
                rx_msg_buffer[RX_BUFFER_SIZE-1] = 0;
            }
        }
        else
        {
            size_t len = strlen((char*)rx_msg_buffer);
            if(len < RX_BUFFER_SIZE-1)
                rx_msg_buffer[len] = b;
            rx_msg_buffer[len+1] = 0;
        }
    }
}

/* ============================================================ */
void SysTick_Handler(void)
{
    static uint32_t last_cnt = 0;
    static uint32_t uart_send_counter = 0;
    static float pid_integral = 0;
    static float last_error = 0;

    /* ==== Read encoder ==== */
    uint32_t now = TIM2->CNT;
    int32_t diff = (int32_t)(now - last_cnt);

    if(diff > 32767) diff -= 65536;
    else if(diff < -32768) diff += 65536;

    last_cnt = now;
    dcnt = diff;

    /* pulses/ms ? RPM  (correct formula) */
    measured_rpm = (float)(diff * 18600.0f / 234.0f);
    /* 3000 = 60s / 0.02s (50Hz encoder sampling) adjust as needed */

    if(state != ST_STOP)
    {
        /* ==== PID ==== */
        int error = (float)target_rpm*7 - measured_rpm;
				need_test = abs((float)target_rpm - measured_rpm);
        /* Integral with anti-windup */
        pid_integral += error * 0.001f;
        if(pid_integral > 2000) pid_integral = 2000;
        if(pid_integral < -2000) pid_integral = -2000;

        /* Proper derivative: DO NOT use abs() */
        float derivative = (error - last_error) * 1000.0f;   // dt = 1ms

        float output = KP * error + KI * pid_integral + KD * derivative;

        /* Direction based on sign of target RPM */
        int pwm = (int)output;

        if(pwm < 0) pwm = -pwm;
        if(pwm > 4199) pwm = 4199;

        apply_pwm(pwm);

        last_error = error;
    }
    else
    {
        /* STOP state */
        pid_integral = 0;
        last_error = 0;
        apply_pwm(0);
    }

    /* ==== Send UART feedback every 100ms ==== */
    uart_send_counter++;
    if(uart_send_counter >= 100)//10
    {
        uart_send_counter = 0;
        char txbuf[64];
        sprintf(txbuf, "TRPM:%d --- MRPM:%d --- PWM:%d --- ERR:%d\n",
                target_rpm, measured_rpm, current_pwm_duty, need_test);

        for(int i = 0; i < strlen(txbuf); i++)
        {
            while(!(USART2->SR & USART_SR_TXE));
            USART2->DR = txbuf[i];
        }
    }
}

/* ============================================================ */
void TIM3_PWM_Config(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    // GPIO clock for PB1
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    // PB1 ? TIM3_CH4
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = GPIO_Pin_1;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);

    TIM_TimeBaseInitTypeDef tb;
    tb.TIM_Prescaler = 1;
    tb.TIM_Period = 4200 - 1;
    tb.TIM_CounterMode = TIM_CounterMode_Up;
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM3, &tb);

    TIM_OCInitTypeDef oc;
    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    oc.TIM_Pulse = 0;
    TIM_OC4Init(TIM3, &oc);
    TIM_OC4PreloadConfig(TIM3, ENABLE);

    TIM_Cmd(TIM3, ENABLE);
}

/* ============================================================ */
void TIM2_Encoder_Config(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);

    TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    TIM_SetAutoreload(TIM2, 0xFFFF);
    TIM_Cmd(TIM2, ENABLE);
}

/* ============================================================ */
void GPIO_Config(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    GPIO_InitTypeDef g;
    g.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    g.GPIO_Mode = GPIO_Mode_OUT;
    g.GPIO_OType = GPIO_OType_PP;
    g.GPIO_PuPd = GPIO_PuPd_NOPULL;
    g.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &g);

    GPIO_ResetBits(GPIOD, GPIO_Pin_0 | GPIO_Pin_1);
}

void delay_ms(uint32_t ms)
{
    for(uint32_t i = 0; i < (ms * 168000); i++);
}
