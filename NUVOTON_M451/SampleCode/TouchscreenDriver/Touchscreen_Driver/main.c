/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 2025/10/29 $
 * @brief    Implement touchscreen driver TSC40/IC based.
 * @notes
 * [X] Timer - CPS control
 * [X] EADC - Analog read for resistive touchscreen (pressure not implemented)
 * [X] FMC - Save user data to Flash memory (like EEPROM)
 * [X] WDT - Watchdog
 * [X] UART - Serial comm
 * [ ] USB - Not implemented
 * @copyright RTEK1000
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 *
 * Note: MCU target M452LD3AE (Tested using only Keil; other IDEs may require selecting the MCU)
 ******************************************************************************/
#include <stdio.h>
#include "M451Series.h"

#define DEBUG_ENABLE 0 // 1: enabled; 0: disabled

#define PLL_CLOCK           72000000

#define ABS(a) (((a) < 0) ? -(a) : (a))

#define ABS_MAX 5

#define PANEL_YD PD0
#define PANEL_YU PD1
#define PANEL_XR PD2
#define PANEL_XL PD3
#define PANEL_THOa PC0

#define AD_YD PB5 // CH13
#define AD_YU PB7 // CH15
#define AD_XL PB3 // CH3
#define AD_XR PB4 // CH4

// The base address of Data Flash is determined by DFBA
#define DATA_FLASH_TEST_BASE1       0x11000 // DFBA 
#define DATA_FLASH_TEST_BASE2       0x11800 // DFBA 

#define CMD_NULL 0x00
#define CMD_COOR_OUTPUT_RATE 0x05
#define CMD_GOTO_COOR_DAT_SEND_START1 0x01
#define CMD_GOTO_COOR_DAT_SEND_START2 0x21
#define CMD_GOTO_COOR_DAT_SEND_START3 0x31
#define CMD_COOR_DATA_END 0x02
#define CMD_GOTO_CAL_DAT_SEND_START1 0x0A
#define CMD_GOTO_CAL_DAT_SEND_START2 0x2A
#define CMD_GOTO_CAL_DAT_SEND_START3 0x3A
#define CMD_CAL_DATA_SEND_END 0x0B
#define CMD_GOTO_CAL_DATA_SETUP1 0x0D
#define CMD_GOTO_CAL_DATA_SETUP2 0x0E
#define CMD_GOTO_CAL_DATA_READ 0x1D
#define CMD_GOTO_STOP 0x0F
#define CMD_WRITE_EEPROM 0x19
#define CMD_RESET 0x55
#define CMD_PANEL_ID 0x15

#define RESPONSE_ACK 0x06
#define RESPONSE_NACK 0x15
#define RESPONSE_PEN_UP 0x10

enum MODES
{
    MODE_POWER_SAVE1 = 0,
    MODE_COOR_DATA1,
    MODE_INIT1,
    MODE_CAL_DATA_SEND1,
    MODE_CAL_DATA_SEND2,
    MODE_CAL_DATA_SEND3,
    MODE_IDLE1,
    MODE_CAL_DATA_SETUP1,
    MODE_STOP1,
    MODE_CAL_DATA_READ1,
};

enum OUTPUT_RATES
{
    OUTPUT_RATE_30CPS = 0x40,
    OUTPUT_RATE_50CPS,
    OUTPUT_RATE_80CPS,
    OUTPUT_RATE_100CPS,
    OUTPUT_RATE_130CPS,
    OUTPUT_RATE_150CPS,
    OUTPUT_RATE_ONCE = 0x50
};

enum EEPROM_ADDR
{
    ADDR_P00_X_RAW_H = 0x00, // 0
    ADDR_P00_X_RAW_L,        // 1
    ADDR_P00_L_RAW_H,        // 2
    ADDR_P00_L_RAW_L,        // 3

    ADDR_P00_X_CAL_H, // 4
    ADDR_P00_X_CAL_L, // 5
    ADDR_P00_L_CAL_H, // 6
    ADDR_P00_L_CAL_L, // 7

    ADDR_P01_X_RAW_H, // 8
    ADDR_P01_X_RAW_L, // 9
    ADDR_P01_L_RAW_H, // 10
    ADDR_P01_L_RAW_L, // 11

    ADDR_P01_X_CAL_H, // 12
    ADDR_P01_X_CAL_L, // 13
    ADDR_P01_L_CAL_H, // 14
    ADDR_P01_L_CAL_L, // 15

    ADDR_P10_X_RAW_H, // 16
    ADDR_P10_X_RAW_L, // 17
    ADDR_P10_L_RAW_H, // 18
    ADDR_P10_L_RAW_L, // 19

    ADDR_P10_X_CAL_H, // 20
    ADDR_P10_X_CAL_L, // 21
    ADDR_P10_L_CAL_H, // 22
    ADDR_P10_L_CAL_L, // 23

    ADDR_P11_X_RAW_H, // 24
    ADDR_P11_X_RAW_L, // 25
    ADDR_P11_L_RAW_H, // 26
    ADDR_P11_L_RAW_L, // 27

    ADDR_P11_X_CAL_H, // 28
    ADDR_P11_X_CAL_L, // 29
    ADDR_P11_L_CAL_H, // 30
    ADDR_P11_L_CAL_L, // 31
};

#define CAL_POINTS_MAX 4 // TSC50: 4, 5 or 9 (Not implemented: 5 and 9)

#if (CAL_POINTS_MAX != 4)
#warning The code needs to be completed if CAL_POINTS_MAX is different from 4
#endif

struct TSPoint_struct {
    int16_t x;
    int16_t y;
};

typedef struct TSPoint_struct TSPoint;

// Note: The data values were obtained through practical tests.
#if (CAL_POINTS_MAX == 4)
TSPoint point_raw[CAL_POINTS_MAX] = {
    {0x0028, 0x0032},
    {0x03D4, 0x0028},
    {0x001E, 0x03D4},
    {0x03CA, 0x03CA}
};
#else
TSPoint point_raw[CAL_POINTS_MAX] = {0};
#endif

#if (CAL_POINTS_MAX == 4)
TSPoint point_cal[CAL_POINTS_MAX] = {
    {0x0000, 0x0000},
    {0x0320, 0x0000},
    {0x0000, 0x0258},
    {0x0320, 0x0258}
};
#else
TSPoint point_cal[CAL_POINTS_MAX] = {0};
#endif

#if (CAL_POINTS_MAX == 4)
TSPoint point_raw_recall[CAL_POINTS_MAX] = {
    {0x0028, 0x0032},
    {0x03D4, 0x0028},
    {0x001E, 0x03D4},
    {0x03CA, 0x03CA}
};
#else
TSPoint point_raw_recall[CAL_POINTS_MAX] = {0};
#endif

#if (CAL_POINTS_MAX == 4)
TSPoint point_cal_recall[CAL_POINTS_MAX] = {
    {0x0000, 0x0000},
    {0x0320, 0x0000},
    {0x0000, 0x0258},
    {0x0320, 0x0258}
};
#else
TSPoint point_cal_recall[CAL_POINTS_MAX] = {0};
#endif

TSPoint point_max_val = {0x03FF, 0x03FF};

uint8_t cal_points = CAL_POINTS_MAX;

/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint8_t timer_flag = 0;

volatile uint8_t cycle_cnt1 = 0;

volatile uint8_t cycle_cnt2 = 0;
uint8_t cycle_cnt2_old = 0;

volatile uint8_t cycle_cnt100 = 0;

volatile uint8_t cycle_cnt64 = 0;

volatile uint8_t is_touch_flag = 0;

const uint8_t pen_up_cnt_recall = 50;
volatile uint8_t pen_up_cnt = 0;
uint8_t pen_down = 0;
uint8_t pen_down_old = 0;

#if (DEBUG == 1)
volatile uint8_t operation_mode = MODE_CAL_DATA_SEND3; // MODE_INIT1; // MODE_CAL_DATA_SEND3
#else
volatile uint8_t operation_mode = MODE_INIT1; // MODE_INIT1; // MODE_CAL_DATA_SEND3
#endif

volatile uint8_t RX_data_new = 0;

volatile uint8_t buffer_index = 0;
// uint8_t RX_buffer[BUFFER_SIZE];

volatile uint8_t set_coord_output_rate = 0;
volatile uint8_t coord_output_rate_ms = 33; // 33ms for 30 cps
volatile uint8_t send_ack = 0;
volatile uint8_t send_nack = 0;

volatile uint32_t g_u32AdcIntFlag, g_u32COVNUMFlag = 0;

#define BUFFER_SIZE 38
uint8_t TX_buffer[10] = {0};
volatile uint8_t RX_buffer[BUFFER_SIZE] = {0};

uint16_t u16ConversionDataX1[3][128] = {{0}};
uint16_t u16ConversionDataY1[3][128] = {{0}};

uint16_t u16ConversionDataX2[3] = {0};
uint16_t u16ConversionDataY2[3] = {0};
uint32_t u32TimeOutCnt;

void touch_read(void);
static int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);
TSPoint calibratePoint(TSPoint pRaw, TSPoint *pRawRef, TSPoint *pCalRef, TSPoint pMaxRef);
void WDT_rst(void);
void UART1_IRQHandler(void);
static void insert_sort(uint16_t array[], uint8_t size);
static int32_t adc_read(uint8_t ch);
int8_t save_data(void);
int8_t load_data(void);

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to HIRC */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Set PLL to Power-down mode and PLLSTB bit in CLK_STATUS register will be cleared by hardware.*/
    CLK_DisablePLL();

    /* Enable HXT clock (external XTAL 16MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL and SysTick source to HCLK/2*/
    CLK_SetCoreClock(PLL_CLOCK);
    CLK_SetSysTickClockSrc(CLK_CLKSEL0_STCLKSEL_HCLK_DIV2);

    /* Enable peripheral clock */
    CLK_EnableModuleClock(UART1_MODULE);
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(WDT_MODULE);

    /* Enable EADC module clock */
    CLK_EnableModuleClock(EADC_MODULE);

    /* EADC clock source is 72MHz, set divider to 4, ADC clock is 72/8 MHz */
    CLK_SetModuleClock(EADC_MODULE, 0, CLK_CLKDIV0_EADC(4));

    /* Peripheral clock source */
    /* Select UART module clock source as PLL and UART module clock divider as 4 */
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UARTSEL_PLL, CLK_CLKDIV0_UART(4));

    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HXT, 0);

    CLK_SetModuleClock(WDT_MODULE, CLK_CLKSEL1_WDTSEL_LIRC, 0);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PA multi-function pins for UART1 RXD(PA.1) and TXD(PA.0) */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA1MFP_Msk | SYS_GPA_MFPL_PA0MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA1MFP_UART1_RXD | SYS_GPA_MFPL_PA0MFP_UART1_TXD);

    /* Configure the GPB3, GPB4, GPB5 and GPB6 ADC analog input pins.  */
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB3MFP_Msk | SYS_GPB_MFPL_PB4MFP_Msk |
                       SYS_GPB_MFPL_PB5MFP_Msk | SYS_GPB_MFPL_PB7MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB3MFP_EADC_CH3 | SYS_GPB_MFPL_PB4MFP_EADC_CH4 |
                      SYS_GPB_MFPL_PB5MFP_EADC_CH13 | SYS_GPB_MFPL_PB7MFP_EADC_CH15);

    /* Disable the GPB3, GPB4, GPB5 and GPB6 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, 0xB8);

//    /* Set PF multi-function pin for EINT5(PF.0) */
//    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF0MFP_Msk)) | SYS_GPF_MFPL_PF0MFP_INT5;
}

/**
 * @brief       Timer0 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The Timer0 default IRQ, declared in startup_M451Series.s.
 */
void TMR0_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER0) == 1)
    {
        /* Clear Timer0 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER0);

        timer_flag = 1;

        if (pen_up_cnt > 0) {
            pen_up_cnt--;
        }
    }
}

/**
 * @brief       IRQ Handler for WDT Interrupt
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The WDT_IRQHandler is default IRQ of WDT, declared in startup_M451Series.s.
 */
void WDT_IRQHandler(void)
{
    // We will not reset WDT counter value and system will be reset immediately by WDT time-out reset signal
}

/*---------------------------------------------------------------------------------------------------------*/
/* EADC interrupt handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void ADC00_IRQHandler(void)
{
    g_u32AdcIntFlag = 1;
    EADC_CLR_INT_FLAG(EADC, 0x1);      /* Clear the A/D ADINT0 interrupt flag */
}

///**
// * @brief       GPIO PD IRQ
// *
// * @param       None
// *
// * @return      None
// *
// * @details     The PD default IRQ, declared in startup_M451Series.s.
// */
//void GPD_IRQHandler(void)
//{
//    /* To check if PD.7 interrupt occurred */
//    if(GPIO_GET_INT_FLAG(PD, BIT7))
//    {
//        GPIO_CLR_INT_FLAG(PD, BIT7);

//#if (DEBUG_ENABLE == 1)
//        printf("PD.7 INT occurred, RESET CHIP NOW\n");
//#endif

//        // uint32_t delay_cnt1 = 0xFFF;

//        // while(delay_cnt1--);

//        SYS_ResetChip();
//    }
//    else
//    {
//        /* Un-expected interrupt. Just clear all PD interrupts */
//        PD->INTSRC = PD->INTSRC;

//#if (DEBUG_ENABLE == 1)
//        printf("Un-expected interrupts.\n");
//#endif
//    }
//}

/**
 * @brief       External INT5 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The External INT5 default IRQ, declared in startup_M451Series.s.
 */
void EINT5_IRQHandler(void)
{
    /* To check if PF.0 external interrupt occurred */
    if(GPIO_GET_INT_FLAG(PF, BIT0))
    {
        GPIO_CLR_INT_FLAG(PF, BIT0);
#if (DEBUG_ENABLE == 1)
        printf("PF.0 EINT0 occurred.\n");
#endif
				SYS_ResetChip(); 
    }

}

void UART1_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART1_RST);

    /* Configure UART1 and set UART1 baud rate */
    UART_Open(UART1, 9600);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    volatile uint32_t u32InitCount;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART1_Init();

    /* Configure PC.0, PD.0, PD.1, PD.2, PD.3 as Output mode */
    GPIO_SetMode(PC, BIT0, GPIO_MODE_OUTPUT); // PANEL_THOa
    GPIO_SetMode(PD, BIT0, GPIO_MODE_OUTPUT); // PANEL_YD
    GPIO_SetMode(PD, BIT1, GPIO_MODE_OUTPUT); // PANEL_YU
    GPIO_SetMode(PD, BIT2, GPIO_MODE_OUTPUT); // PANEL_XR
    GPIO_SetMode(PD, BIT3, GPIO_MODE_OUTPUT); // PANEL_XL

    /* Set output pin value is high */
    PANEL_YD = 1;
    PANEL_XR = 1;
    PANEL_XL = 1;

    PANEL_THOa = 1;

    /* Set output pin value is low */
    PANEL_YU = 0;

    /* Configure PB.3, PB.4, PB.5 and PB.7 as Input mode */
    GPIO_SetMode(PB, BIT3, GPIO_MODE_INPUT); // AD_XL
    GPIO_SetMode(PB, BIT4, GPIO_MODE_INPUT); // AD_XR
    GPIO_SetMode(PB, BIT5, GPIO_MODE_INPUT); // AD_YD
    GPIO_SetMode(PB, BIT7, GPIO_MODE_INPUT); // AD_YU

    /* Open Timer0 in periodic mode, enable interrupt and 783 interrupt tick per second (Hz)*/
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 783);
    TIMER_EnableInt(TIMER0);

    /* Enable Timer0 ~ Timer3 NVIC */
    NVIC_EnableIRQ(TMR0_IRQn);

    /* Start Timer0 counting */
    TIMER_Start(TIMER0);

    /* Enable UART RDA/Time-out interrupt */
    UART_EnableInt(UART1, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk));
    NVIC_EnableIRQ(UART1_IRQn);

#if (DEBUG_ENABLE == 1)
    printf("Power On\n");
#endif

    /* Because of all bits can be written in WDT Control Register are write-protected;
       To program it needs to disable register protection first. */
    SYS_UnlockReg();

    uint32_t delay_cnt1 = 0x1FFF7;

    while(delay_cnt1--) {
        WDT_RESET_COUNTER();
    }

//		delay_cnt1 = 30;

//		while(delay_cnt1 > 0) {
//				if(timer_flag == 1) {
//						timer_flag = 0;
//
//						delay_cnt1--;
//				}
//		}

    /* Lock protected registers */
    SYS_LockReg();

#if (DEBUG_ENABLE == 1)
    printf("Start\n");
#endif

#if (DEBUG_ENABLE == 0)
    TX_buffer[0] = 0x12;
    UART_Write(UART1, TX_buffer, 1);
#endif

    /* Enable WDT NVIC */
    NVIC_EnableIRQ(WDT_IRQn);

    int8_t res = 0;

    res = load_data();

    if (res == 0) {
#if (DEBUG_ENABLE == 1)
        printf("[OK] load_data()\n");
#endif
    }
    else
    {
#if (DEBUG_ENABLE == 1)
        printf("[Error] load_data(): %d\n", res);
#endif

        res = save_data();

        if (res == 0) {
#if (DEBUG_ENABLE == 1)
            printf("[OK] save_data()\n");
#endif
        }
        else
        {
#if (DEBUG_ENABLE == 1)
            printf("[Error] save_data(): %d\n", res);
#endif
        }

        res = load_data();

        if (res == 0) {
#if (DEBUG_ENABLE == 1)
            printf("[OK] load_data()\n");
#endif
        }
        else
        {
#if (DEBUG_ENABLE == 1)
            printf("[Error] load_data(): %d (Flash error?)\n", res);
            printf("Load default data\n");
#endif
            for (uint8_t i = 0; i < CAL_POINTS_MAX; i++) {
                point_raw[i] = point_raw_recall[i];
                point_cal[i] = point_cal_recall[i];
            }
        }
    }

    /* Because of all bits can be written in WDT Control Register are write-protected;
       To program it needs to disable register protection first. */
    SYS_UnlockReg();

    /* Configure WDT settings and start WDT counting */
    WDT_Open(WDT_TIMEOUT_2POW14, WDT_RESET_DELAY_18CLK, TRUE, TRUE);

    WDT_RESET_COUNTER();

    /* Enable WDT interrupt function */
    WDT_EnableInt();

    /* Lock protected registers */
    SYS_LockReg();

//		/* Configure PF.1 as Input mode */
//    GPIO_SetMode(PF, BIT1, GPIO_MODE_INPUT); // (Alternative RESET)

//    /* Configure PF.0 as QUASI mode and enable interrupt by falling edge trigger */
//    GPIO_SetMode(PF, BIT0, GPIO_MODE_INPUT); // (Alternative RESET)

//    //PF0 = 1; // Enable pull-up

//    /* Disable interrupt de-bounce function */
//    //GPIO_DISABLE_DEBOUNCE(PF, BIT0);

//		GPIO_EnableInt(PF, 0, GPIO_INT_FALLING);
//    NVIC_EnableIRQ(EINT5_IRQn);
		
    /* Enable interrupt de-bounce function and select de-bounce sampling cycle time is 1024 clocks of LIRC clock */
    //GPIO_SET_DEBOUNCE_TIME(GPIO_DBCTL_DBCLKSRC_HCLK, GPIO_DBCTL_DBCLKSEL_1);
    //GPIO_ENABLE_DEBOUNCE(PF, BIT0);
		
    while(1) {
        if (RX_data_new == 1) {
            point_raw[0].x = (RX_buffer[2] << 8) | RX_buffer[3];
            point_raw[0].y = (RX_buffer[4] << 8) | RX_buffer[5];
            point_cal[0].x = (RX_buffer[6] << 8) | RX_buffer[7];
            point_cal[0].y = (RX_buffer[8] << 8) | RX_buffer[9];

            point_raw[1].x = (RX_buffer[10] << 8) | RX_buffer[11];
            point_raw[1].y = (RX_buffer[12] << 8) | RX_buffer[13];
            point_cal[1].x = (RX_buffer[14] << 8) | RX_buffer[15];
            point_cal[1].y = (RX_buffer[16] << 8) | RX_buffer[17];

            point_raw[2].x = (RX_buffer[18] << 8) | RX_buffer[19];
            point_raw[2].y = (RX_buffer[20] << 8) | RX_buffer[21];
            point_cal[2].x = (RX_buffer[22] << 8) | RX_buffer[23];
            point_cal[2].y = (RX_buffer[24] << 8) | RX_buffer[25];

            point_raw[3].x = (RX_buffer[26] << 8) | RX_buffer[27];
            point_raw[3].y = (RX_buffer[28] << 8) | RX_buffer[29];
            point_cal[3].x = (RX_buffer[30] << 8) | RX_buffer[31];
            point_cal[3].y = (RX_buffer[32] << 8) | RX_buffer[33];

            save_data();
						
						RX_data_new = 0;
        }
				else if (operation_mode != MODE_IDLE1)
				{
						touch_read();
				}

        if (send_nack == 1)
        {
#if (DEBUG_ENABLE == 0)
            TX_buffer[0] = RESPONSE_NACK;
            UART_Write(UART1, TX_buffer, 1);
#endif
            send_nack = 0;
        }

        if (send_ack == 1)
        {
#if (DEBUG_ENABLE == 0)
            TX_buffer[0] = RESPONSE_ACK;
            UART_Write(UART1, TX_buffer, 1);
#endif
            send_ack = 0;
        }

        WDT_rst();
    }
}

void touch_read(void) {
    TSPoint p_send;

    if(timer_flag == 1) {
        timer_flag = 0;

        if (cycle_cnt1 >= 25) {
            PANEL_YU = 0;
            PANEL_YD = 1;
            PANEL_XL = 1;
            PANEL_XR = 1;

            GPIO_SetMode(PC, BIT0, GPIO_MODE_OUTPUT);

            if (is_touch_flag == 1) {
                for(uint8_t i = 0; i < 3; i++) {
                    PANEL_THOa = 0;
                }
            }

            PANEL_THOa = 1;

            /* Enable the GPB5 digital input path. */
            GPIO_ENABLE_DIGITAL_PATH(PB, 0x20);

            if (AD_YD == 0) {
                is_touch_flag = 1;

                cycle_cnt1 = 1;

                GPIO_SetMode(PC, BIT0, GPIO_MODE_INPUT);
                PANEL_THOa = 0;

                /* Disable the GPB3, GPB4, GPB5 and GPB6 digital input path to avoid the leakage current. */
                GPIO_DISABLE_DIGITAL_PATH(PB, 0xB8);
            } else {
                is_touch_flag = 0;

                if (pen_down == 1) {
                    if (pen_up_cnt == 0) {
                        pen_down = 0;

#if (DEBUG_ENABLE == 0)
                        TX_buffer[0] = RESPONSE_PEN_UP;
                        UART_Write(UART1, TX_buffer, 1);
#endif
                    }
                }
            }
        } else if (cycle_cnt1 < 25) {
            cycle_cnt1++;
        }

        if (is_touch_flag == 1) {
            switch(cycle_cnt1) {
            case 7:
                PANEL_YD = 1;

                GPIO_SetMode(PC, BIT0, GPIO_MODE_OUTPUT);
                PANEL_THOa = 1;

                PANEL_YU = 0;
                break;
            default:
                break;
            }
        }

        cycle_cnt2 = cycle_cnt1;
    }

    if (cycle_cnt2_old != cycle_cnt2) {
        cycle_cnt2_old = cycle_cnt2;

        if ((cycle_cnt2 >= 1) && (cycle_cnt2 <= 6)) {
            GPIO_SetMode(PC, BIT0, GPIO_MODE_INPUT);
            PANEL_THOa = 0;

            /* Disable the GPB3, GPB4, GPB5 and GPB6 digital input path to avoid the leakage current. */
            GPIO_DISABLE_DIGITAL_PATH(PB, 0xB8);

            uint8_t index1 = 0;

            switch (cycle_cnt2) {
            case 1:
            case 2:
                index1 = 0;
                break;
            case 3:
            case 4:
                index1 = 1;
                break;
            case 5:
            case 6:
                index1 = 2;
                break;
            default:
                break;
            }

            for (uint8_t counter1 = 0; counter1 < 128; counter1++) {
                if ((cycle_cnt2 == 1) || (cycle_cnt2 == 3) || (cycle_cnt2 == 5)) {
                    /* Power off Y axis */
                    for(uint8_t i = 0; i < 1; i++) {
                        PANEL_YU = 0;
                    }

                    for(uint8_t i = 0; i < 1; i++) {
                        PANEL_YD = 1;
                    }

                    /* Power on X axis */
                    for(uint8_t i = 0; i < 1; i++) {
                        PANEL_XL = 1;
                    }

                    for(uint8_t i = 0; i < 1; i++) {
                        PANEL_XR = 0;
                    }

                    u16ConversionDataX1[index1][counter1] = adc_read(15) >> 2;
                } else { // if ((cycle_cnt1 == 2) || (cycle_cnt1 == 4) || (cycle_cnt1 == 6)) {
                    /* Power on Y axis */
                    for(uint8_t i = 0; i < 1; i++) {
                        PANEL_YU = 1;
                    }

                    for(uint8_t i = 0; i < 1; i++) {
                        PANEL_YD = 0;
                    }

                    /* Power off X axis */
                    for(uint8_t i = 0; i < 1; i++) {
                        PANEL_XL = 0;
                    }

                    for(uint8_t i = 0; i < 1; i++) {
                        PANEL_XR = 1;
                    }

                    u16ConversionDataY1[index1][counter1] = adc_read(4) >> 2;
                }
            }
        } else {
            if (cycle_cnt2 == 7) {
                for (uint8_t i = 0; i < 3; i++) {
                    uint32_t tempX = 0;
                    uint32_t tempY = 0;

                    for (uint8_t j = 16; j < 112; j++) {
                        tempX += u16ConversionDataX1[i][j];
                        tempY += u16ConversionDataY1[i][j];
                    }

                    tempX /= 96;
                    tempY /= 96;

                    u16ConversionDataX2[i] = tempX;
                    u16ConversionDataY2[i] = tempY;
                }

                uint8_t valid = 0;

                if (((ABS(u16ConversionDataX2[0] - u16ConversionDataX2[1]) <= ABS_MAX) &&
                        (ABS(u16ConversionDataX2[0] - u16ConversionDataX2[2]) <= ABS_MAX)) &&
                        ((ABS(u16ConversionDataY2[0] - u16ConversionDataY2[1]) <= ABS_MAX) &&
                         (ABS(u16ConversionDataY2[0] - u16ConversionDataY2[2]) <= ABS_MAX))) {

                    u16ConversionDataX2[0] += u16ConversionDataX2[1];
                    u16ConversionDataX2[0] += u16ConversionDataX2[2];
                    u16ConversionDataX2[0] /= 3;

                    u16ConversionDataY2[0] += u16ConversionDataY2[1];
                    u16ConversionDataY2[0] += u16ConversionDataY2[2];
                    u16ConversionDataY2[0] /= 3;

                    valid = 1;

                    pen_down = 1;

                    pen_up_cnt = pen_up_cnt_recall;

                    p_send.x = u16ConversionDataX2[0];
                    p_send.y = u16ConversionDataY2[0];

                    if (operation_mode == MODE_CAL_DATA_SEND3)
                    {
                        p_send = calibratePoint(p_send, point_raw, point_cal, point_max_val);
                    }

                    uint8_t pen_ID = 0x11;

                    uint8_t pen_XH = (p_send.x >> 8) & 0xFF;
                    uint8_t pen_XL = p_send.x & 0xFF;
                    uint8_t pen_YH = (p_send.y >> 8) & 0xFF;
                    uint8_t pen_YL = p_send.y & 0xFF;

                    //pen_ID |= (p.y >> 6) & 0x01;
#if (DEBUG_ENABLE == 0)
                    TX_buffer[0] = pen_ID;
                    TX_buffer[1] = pen_XH;
                    TX_buffer[2] = pen_XL;
                    TX_buffer[3] = pen_YH;
                    TX_buffer[4] = pen_YL;

                    UART_Write(UART1, TX_buffer, 5);
#endif
                }

#if (DEBUG_ENABLE == 1)
                printf("X: %d, Y: %d, V:%d\n", u16ConversionDataX2[0],	u16ConversionDataY2[0], valid);
#endif
            }
        }
    }
}

static int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

TSPoint calibratePoint(TSPoint pRaw, TSPoint *pRawRef, TSPoint *pCalRef, TSPoint pMaxRef)
{
    int16_t x_raw_pos = pRaw.x;
    int16_t x_raw_min = (pRawRef[0].x + pRawRef[2].x) / 2;
    int16_t x_raw_max = (pRawRef[1].x + pRawRef[3].x) / 2;

    int16_t x_cal_min = (pCalRef[0].x + pCalRef[2].x) / 2;
    int16_t x_cal_max = (pCalRef[1].x + pCalRef[3].x) / 2;

    int16_t x_cal_pos = map(x_raw_pos, x_raw_min, x_raw_max, x_cal_min, x_cal_max);

    if (x_cal_pos > pMaxRef.x)
    {
        x_cal_pos = pMaxRef.x;
    }
    else if (x_cal_pos < 0)
    {
        x_cal_pos = 0;
    }

    int16_t y_raw_pos = pRaw.y;
    int16_t y_raw_min = (pRawRef[0].y + pRawRef[1].y) / 2;
    int16_t y_raw_max = (pRawRef[2].y + pRawRef[3].y) / 2;

    int16_t y_cal_min = (pCalRef[0].y + pCalRef[1].y) / 2;
    int16_t y_cal_max = (pCalRef[2].y + pCalRef[3].y) / 2;

    int16_t y_cal_pos = map(y_raw_pos, y_raw_min, y_raw_max, y_cal_min, y_cal_max);

    if (y_cal_pos > pMaxRef.y)
    {
        y_cal_pos = pMaxRef.y;
    }
    else if (y_cal_pos < 0)
    {
        y_cal_pos = 0;
    }

    TSPoint p_res;

    p_res.x = x_cal_pos;
    p_res.y = y_cal_pos;

    return p_res;
}

void WDT_rst(void) {
    /* Because of all bits can be written in WDT Control Register are write-protected;
       To program it needs to disable register protection first. */
    SYS_UnlockReg();

    WDT_RESET_COUNTER();

    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 0 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_IRQHandler(void)
{
    uint8_t _data = 0xFF;
    uint32_t u32IntSts = UART1->INTSTS;
    uint8_t u8OutChar[2] = {0};

    /* Receive Data Available Interrupt Handle */
    if(u32IntSts & UART_INTSTS_RDAINT_Msk)
    {
        /* Get all the input characters */
        while(UART_IS_RX_READY(UART1))
        {
            /* Get the character from UART Buffer */
            _data = UART_READ(UART1);

#if (DEBUG_ENABLE == 1)
            printf("%02X ", _data);
#endif
            if (operation_mode == MODE_INIT1)
            {
                if (set_coord_output_rate == 0)
                {
                    if (_data == CMD_COOR_OUTPUT_RATE)
                    {
                        set_coord_output_rate = 1;
                    }
                }
                else
                {
                    switch (_data)
                    {
                    case OUTPUT_RATE_30CPS:
                        coord_output_rate_ms = 33; // 30 cps
                        send_ack = 1;
                        break;
                    case OUTPUT_RATE_50CPS:
                        coord_output_rate_ms = 20; // 50 cps
                        send_ack = 1;
                        break;
                    case OUTPUT_RATE_80CPS:
                        coord_output_rate_ms = 12; // 80 cps
                        send_ack = 1;
                        break;
                    case OUTPUT_RATE_100CPS:
                        coord_output_rate_ms = 10; // 100 cps
                        send_ack = 1;
                        break;
                    case OUTPUT_RATE_130CPS:
                        coord_output_rate_ms = 8; // 130 cps
                        send_ack = 1;
                        break;
                    case OUTPUT_RATE_150CPS:
                        coord_output_rate_ms = 7; // 150 cps
                        send_ack = 1;
                        break;
                    case OUTPUT_RATE_ONCE:
                        coord_output_rate_ms = 0; // Once when touched
                        send_ack = 1;
                        break;
                    default:
                        send_nack = 1;
                        break;
                    }

                    if (send_ack == 1)
                    {
                        operation_mode = MODE_IDLE1;
                    }

                    set_coord_output_rate = 0;
                }
            }
            else if (operation_mode == MODE_COOR_DATA1)
            {
                if (_data == CMD_COOR_DATA_END)
                {
                    operation_mode = MODE_IDLE1;
                    send_ack = 1;
                }
            }
            else if ((operation_mode == MODE_CAL_DATA_SEND1) ||
                     (operation_mode == MODE_CAL_DATA_SEND2) ||
                     (operation_mode == MODE_CAL_DATA_SEND3))
            {
                if (_data == CMD_CAL_DATA_SEND_END)
                {
                    operation_mode = MODE_IDLE1;
                    send_ack = 1;
                }
            }
            else if (operation_mode == MODE_IDLE1)
            {
                switch (_data)
                {
                case CMD_GOTO_CAL_DAT_SEND_START3:
                    operation_mode = MODE_CAL_DATA_SEND3;
                    break;
                case CMD_GOTO_COOR_DAT_SEND_START3:
                    operation_mode = MODE_COOR_DATA1;
                    send_ack = 1;
                    break;
                case CMD_GOTO_CAL_DATA_SETUP1:
                    operation_mode = MODE_CAL_DATA_SETUP1;
                    buffer_index = 0;
                    break;
                case CMD_NULL:
                    break;
                default:
                    send_nack = 1;
                    break;
                }
            }
            else if (operation_mode == MODE_CAL_DATA_SETUP1)
            {
                if (buffer_index < BUFFER_SIZE)
                {
                    RX_buffer[buffer_index++] = _data;

                    if (buffer_index == BUFFER_SIZE)
                    {
                        operation_mode = MODE_IDLE1;

                        if ((RX_buffer[0] == 0x02) && (RX_buffer[1] == 0x02) &&
                                (RX_buffer[34] == 0x03) && (RX_buffer[35] == 0xFF) &&
                                (RX_buffer[36] == 0x03) && (RX_buffer[37] == 0xFF))
                        {
                            RX_data_new = 1;

//                            point_raw[0].x = (RX_buffer[2] << 8) | RX_buffer[3];
//                            point_raw[0].y = (RX_buffer[4] << 8) | RX_buffer[5];
//                            point_cal[0].x = (RX_buffer[6] << 8) | RX_buffer[7];
//                            point_cal[0].y = (RX_buffer[8] << 8) | RX_buffer[9];

//                            point_raw[1].x = (RX_buffer[10] << 8) | RX_buffer[11];
//                            point_raw[1].y = (RX_buffer[12] << 8) | RX_buffer[13];
//                            point_cal[1].x = (RX_buffer[14] << 8) | RX_buffer[15];
//                            point_cal[1].y = (RX_buffer[16] << 8) | RX_buffer[17];

//                            point_raw[2].x = (RX_buffer[18] << 8) | RX_buffer[19];
//                            point_raw[2].y = (RX_buffer[20] << 8) | RX_buffer[21];
//                            point_cal[2].x = (RX_buffer[22] << 8) | RX_buffer[23];
//                            point_cal[2].y = (RX_buffer[24] << 8) | RX_buffer[25];

//                            point_raw[3].x = (RX_buffer[26] << 8) | RX_buffer[27];
//                            point_raw[3].y = (RX_buffer[28] << 8) | RX_buffer[29];
//                            point_cal[3].x = (RX_buffer[30] << 8) | RX_buffer[31];
//                            point_cal[3].y = (RX_buffer[32] << 8) | RX_buffer[33];

//                            save_data();

                            send_ack = 1;
                        }
                        else
                        {
                            send_nack = 1;
                        }
                    }
                }
            }
        }
    }
}

static void insert_sort(uint16_t array[], uint8_t size)
{
    uint8_t j;
    uint16_t save;

    for (uint16_t i = 1; i < size; i++)
    {
        save = array[i];
        for (j = i; j >= 1 && save < array[j - 1]; j--)
            array[j] = array[j - 1];
        array[j] = save;
    }
}

static int32_t adc_read(uint8_t ch)
{
    /* Set the ADC internal sampling time, input mode as single-end and enable the A/D converter */
    EADC_Open(EADC, EADC_CTL_DIFFEN_SINGLE_END);
    EADC_SetInternalSampleTime(EADC, 6);

    /* Configure the sample module 0 for analog input channel ch and software trigger source.*/
    EADC_ConfigSampleModule(EADC, 0, EADC_SOFTWARE_TRIGGER, ch);

    /* Clear the A/D ADINT0 interrupt flag for safe */
    EADC_CLR_INT_FLAG(EADC, 0x1);

    /* Enable the sample module 0 interrupt.  */
    EADC_ENABLE_INT(EADC, 0x1);//Enable sample module A/D ADINT0 interrupt.
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 0, 0x1);//Enable sample module 0 interrupt.
    NVIC_EnableIRQ(ADC00_IRQn);

    /* Reset the ADC interrupt indicator and trigger sample module 0 to start A/D conversion */
    g_u32AdcIntFlag = 0;
    EADC_START_CONV(EADC, 0x1);

    /* Wait ADC interrupt (g_u32AdcIntFlag will be set at IRQ_Handler function) */
    u32TimeOutCnt = EADC_TIMEOUT;
    while(g_u32AdcIntFlag == 0)
    {
        if(--u32TimeOutCnt == 0)
        {
#if (DEBUG_ENABLE == 1)
            printf("Wait for EADC interrupt time-out!\n");
#endif
            return -1;
        }
    }

    /* Disable the ADINT0 interrupt */
    EADC_DISABLE_INT(EADC, 0x1);

    /* Get the conversion result of the sample module 0 */
    return EADC_GET_CONV_DATA(EADC, 0);
}

static int  SetDataFlashBase(uint32_t u32DFBA)
{
    uint32_t au32Config[2];

    /* Read current User Configuration */
    FMC_ReadConfig(au32Config, 1);

    /* Just return when Data Flash has been enabled */
    if(!(au32Config[0] & 0x1))
        return 0;

    /* Enable User Configuration Update */
    FMC_EnableConfigUpdate();

    /* Erase User Configuration */
    FMC_Erase(FMC_CONFIG_BASE);

    /* Write User Configuration to Enable Data Flash */
    au32Config[0] &= ~0x1;
    au32Config[1] = u32DFBA;

    if(FMC_WriteConfig(au32Config, 2))
        return -1;

#if (DEBUG_ENABLE == 1)
    printf("\nSet Data Flash base as 0x%x.\n", FMC_ReadDataFlashBaseAddr());
#endif

    /* Perform chip reset to make new User Config take effect */
    SYS->IPRST0 |= SYS_IPRST0_CHIPRST_Msk;

    return 0;
}

int32_t  VerifyData(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t u32Pattern)
{
    uint32_t    u32Addr;
    uint32_t    u32Data;

    for(u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += 4)
    {
        u32Data = FMC_Read(u32Addr);
        if(u32Data != u32Pattern)
        {
#if (DEBUG_ENABLE == 1)
            printf("\nFMC_Read data verify failed at address 0x%x, read=0x%x, expect=0x%x\n", u32Addr, u32Data, u32Pattern);
#endif
            return -1;
        }
    }
    return 0;
}

int8_t save_data(void)
{
#if (DEBUG_ENABLE == 1)
    printf("save_data()\n");
#endif
    uint8_t u8Index = 0;
    int8_t error = 0;
    uint32_t u32Addr;
    uint32_t u32Data[(CAL_POINTS_MAX * 2) + 1];

    // For CAL_POINTS_MAX = 4: (4x (2x u32) + 1x u32 checksum) to u8: 36
    uint8_t data_size = ((CAL_POINTS_MAX * 2) + 1) * 4;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable FMC ISP function */
    FMC_Open();

    if(SetDataFlashBase(DATA_FLASH_TEST_BASE1) < 0)
    {
#if (DEBUG_ENABLE == 1)
        printf("Failed to set Data Flash base address!\n");
#endif
        error = -1;
    }

    /* Read BS */
    // printf("  Boot Mode ............................. ");
    if(FMC_GetBootSource() == 0)
    {
        // printf("[APROM]\n");
    }
    else
    {
#if (DEBUG_ENABLE == 1)
        printf("[LDROM]\n");
        printf("  WARNING: The driver sample code must execute in AP mode!\n");
#endif
        error = -2;
    }

    uint8_t i = 0;

    for (i = 0; i < (cal_points * 2); i += 2)
    {
        u32Data[i] = point_raw[i].x << 16;
        u32Data[i] += point_raw[i].y;
        u32Data[i + 1] = point_cal[i].x << 16;
        u32Data[i + 1] += point_cal[i].y;
    }

    uint32_t u32Sum = 0;

    for (uint8_t j = 0; j < (cal_points * 2); j++)
    {
        u32Sum += u32Data[j];
    }

    u32Sum = 0xFFFFFFFF - u32Sum;

    u32Data[CAL_POINTS_MAX * 2] = u32Sum;

    u8Index = 0;
    uint8_t update_flag = 0;

    // 9x u32 to u8: 36
    for(u32Addr = DATA_FLASH_TEST_BASE1; u32Addr < (DATA_FLASH_TEST_BASE1 + data_size); u32Addr += 4)
    {
        uint32_t flash_data = FMC_Read(u32Addr);
#if (DEBUG_ENABLE == 1)
        printf("Data In: 0x%08X, Flash Data: 0x%08X\n", u32Data[u8Index], flash_data);
#endif

        if (u32Data[u8Index] != flash_data) {

            update_flag = 1;
        }

        u8Index++;
    }

    if (update_flag == 0) {
#if (DEBUG_ENABLE == 1)
        printf("Skip Flash Write. Data OK.\n");
#endif
        return 0;
    }

    // Erase page
    FMC_Erase(DATA_FLASH_TEST_BASE1);

    // Verify if page contents are all 0xFFFFFFFF
    if(VerifyData(DATA_FLASH_TEST_BASE1, DATA_FLASH_TEST_BASE1 + FMC_FLASH_PAGE_SIZE, 0xFFFFFFFF) < 0)
    {
#if (DEBUG_ENABLE == 1)
        printf("\nPage 0x%x erase verify failed!\n", DATA_FLASH_TEST_BASE1);
#endif
        error = -3;
    }

    u8Index = 0;

    for(u32Addr = DATA_FLASH_TEST_BASE1; u32Addr < (DATA_FLASH_TEST_BASE1 + data_size); u32Addr += 4)
    {
        FMC_Write(u32Addr, u32Data[u8Index]);

        u8Index++;
    }

    u8Index = 0;

    // 9x u32 to u8: 36
    for(u32Addr = DATA_FLASH_TEST_BASE1; u32Addr < (DATA_FLASH_TEST_BASE1 + data_size); u32Addr += 4)
    {
        if (u32Data[u8Index] != FMC_Read(u32Addr)) {
#if (DEBUG_ENABLE == 1)
            printf("\nPage 0x%x write verify failed!\n", DATA_FLASH_TEST_BASE1);
#endif

            error = -4;
        }

        u8Index++;
    }

error_exit:

    /* Disable FMC ISP function */
    FMC_Close();

    /* Lock protected registers */
    SYS_LockReg();

    return error;
}

int8_t load_data(void)
{
#if (DEBUG_ENABLE == 1)
    printf("load_data()\n");
#endif
    int8_t error = 0;

    // For CAL_POINTS_MAX = 4: (4x (2x u32) + 1x u32 checksum) to u8: 36
    uint8_t data_size = ((CAL_POINTS_MAX * 2) + 1) * 4;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable FMC ISP function */
    FMC_Open();

    if(SetDataFlashBase(DATA_FLASH_TEST_BASE1) < 0)
    {
#if (DEBUG_ENABLE == 1)
        printf("Failed to set Data Flash base address!\n");
#endif
        error = -1;
        goto error_exit;
    }

    uint32_t u32Addr;
    uint32_t u32Data[(CAL_POINTS_MAX * 2) + 1];
    uint8_t u8Index = 0;

    for(u32Addr = DATA_FLASH_TEST_BASE1; u32Addr < (DATA_FLASH_TEST_BASE1 + data_size); u32Addr += 4)
    {
        u32Data[u8Index] = FMC_Read(u32Addr);

#if (DEBUG_ENABLE == 1)
        printf("u32Data = 0x%08X\n", u32Data[u8Index]);
#endif

        u8Index++;
    }

    uint8_t i = 0;

    for (i = 0; i < (cal_points * 2); i += 2)
    {
        point_raw[i].x = u32Data[i] >> 16;
        point_raw[i].y = u32Data[i] & 0xFFFF;
        point_cal[i].x = u32Data[i + 1] >> 16;
        point_cal[i].y = u32Data[i + 1] & 0xFFFF;
    }

    uint32_t u32Sum = 0;

    for (uint8_t j = 0; j < (cal_points * 2); j++)
    {
        u32Sum += u32Data[j];
    }

    u32Sum = 0xFFFFFFFF - u32Sum;

    if (u32Data[CAL_POINTS_MAX * 2] != u32Sum) {

        for (uint8_t j = 0; j < (cal_points * 2); j++)
        {
            u32Sum += u32Data[j];
        }

        error = -2;
        goto error_exit;
    }

error_exit:

    /* Disable FMC ISP function */
    FMC_Close();

    /* Lock protected registers */
    SYS_LockReg();

    return error;
}

/*** (C) COPYRIGHT 2013~2015 Nuvoton Technology Corp. ***/
