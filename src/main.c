/*
**
**                           Main.c
**
**
**********************************************************************/
/*
   Last committed:     $Revision: 00 $
   Last changed by:    $Author: $
   Last changed date:  $Date:  $
   ID:                 $Id:  $

**********************************************************************/
#include "stm32f10x_conf.h"
#include "driver_uart.h"
#include "delay.h"
#include <stdbool.h>
#include <string.h>

#define BUTTON_PAIR               GPIO_Pin_0
#define SENSOR_PIN                GPIO_Pin_1
#define RF_CS_PIN                 GPIO_Pin_7
#define RF_SET_PIN                GPIO_Pin_8

#define PIN_IN1_L298              GPIO_Pin_12
#define PIN_IN2_L298              GPIO_Pin_13
#define PIN_CONTROL_L298          GPIO_Pin_14
#define PIN_CONTROL_LEFT          GPIO_Pin_8
#define PIN_CONTROL_RIGHT         GPIO_Pin_9
#define PIN_CONTROL_MOTOR         GPIO_Pin_2

#define LED_ON()                  GPIO_SetBits(GPIOA, GPIO_Pin_6)
#define LED_OFF()                 GPIO_ResetBits(GPIOA, GPIO_Pin_6)

#define MOTOR_DISABLE()           GPIO_ResetBits(GPIOB, PIN_IN1_L298), GPIO_ResetBits(GPIOB, PIN_IN2_L298)
#define OPEN_DOOR()               GPIO_SetBits(GPIOB, PIN_IN1_L298), GPIO_ResetBits(GPIOB, PIN_IN2_L298)
#define CLOSE_DOOR()              GPIO_SetBits(GPIOB, PIN_IN2_L298), GPIO_ResetBits(GPIOB, PIN_IN1_L298)

#define STM32_UUID                ((uint32_t *)0x1FFFF7E8)
#define FLASH_UID_ADDR            0x0801FC00

bool status_pair = false;
volatile uint8_t slot_rx_time = 0;
volatile uint8_t slot_ping_time = 0;
volatile uint8_t sync_send_time = 0;
volatile bool sync_status = true;
volatile bool tx1_status = true;
volatile bool rx_status = true;
volatile bool ping_status = true;
volatile bool status_motor = true;
volatile uint8_t time_motor = true;

char status_pair_arr[64];
char type[2];
char ID_master[5];
char id_node[1];
char mess[1];

extern char Rx_bufArr[13];
extern char uart1_rx[13];

void Timer_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    TIM_TimeBaseInitTypeDef   Timer_InitStructure;
    Timer_InitStructure.TIM_Prescaler   = 7200 - 1;
    Timer_InitStructure.TIM_Period      = 10000 - 1;
    Timer_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &Timer_InitStructure);
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM2, ENABLE);

    NVIC_InitTypeDef        NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel    = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd  = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 10;
    NVIC_Init(&NVIC_InitStructure);
}

void TIM2_IRQHandler(void)
{
    if(TIM_GetFlagStatus(TIM2, TIM_IT_Update) != RESET)
    {
        slot_rx_time++;
        status_motor = false;
        if(slot_rx_time == 1) tx1_status = false;
        if(slot_rx_time == 2) rx_status = false;
        if(slot_rx_time == 3)
        {
            slot_ping_time++;
            if(slot_ping_time == 2) ping_status = false;
            if(slot_ping_time > 2) slot_ping_time = 0;
        }
        if(slot_rx_time > 3)
        {
            slot_rx_time = 0;
        }

        TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    }
}
/**
  * @brief  Gpio_Init config gpio
  * @param
  * @retval
  */
void Gpio_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitTypeDef   GPIO_Init_Structure;

    GPIO_Init_Structure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init_Structure.GPIO_Pin   = PIN_IN1_L298|PIN_IN2_L298|PIN_CONTROL_L298;
    GPIO_Init_Structure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOB, &GPIO_Init_Structure);

    GPIO_Init_Structure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init_Structure.GPIO_Pin   = PIN_CONTROL_LEFT|PIN_CONTROL_RIGHT;
    GPIO_Init_Structure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOB, &GPIO_Init_Structure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_Init_Structure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init_Structure.GPIO_Pin   = BUTTON_PAIR|SENSOR_PIN|PIN_CONTROL_MOTOR;
    GPIO_Init_Structure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOA, &GPIO_Init_Structure);

     GPIO_Init_Structure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init_Structure.GPIO_Pin    = RF_CS_PIN|RF_SET_PIN|GPIO_Pin_6;
    GPIO_Init_Structure.GPIO_Speed  = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_Init_Structure);
}
/**
  * @brief  Reads the specified GPIO output data port.
  * @param  GPIOx: where x can be (A..G) to select the GPIO peripheral.
  * @retval GPIO output data port value.
  */
void Button_Detect_Event(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, bool *status)
{
    bool status_button;
    if(GPIO_ReadInputDataBit(GPIOx, GPIO_Pin) == 1)
    {
        status_button = true;
    }

    if(GPIO_ReadInputDataBit(GPIOx, GPIO_Pin) == 0 && status_button == true)
    {
        status_button = false;
        *status = true;
    }
}

/**
  * @brief
  * @param
  * @retval
  */
void Sensor_Detect_Event(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, bool *status)
{
    bool status_sensor;
    if(GPIO_ReadInputDataBit(GPIOx, GPIO_Pin) == 1)
    {
        status_sensor = true;
         *status = true;
    }

    if(GPIO_ReadInputDataBit(GPIOx, GPIO_Pin) == 0 && status_sensor == true)
    {
        status_sensor = false;
        *status = false;
    }
}
/**
  * @brief
  * @param
  * @retval
  */
void Process_Message(char *str, char *type, char *ID_get_way, char *ID_node, char *mess)
{
    char *p_rx;
    char *p_type;
    char *p_id_getway;
    char *p_id_node;
    char *p_mess;
    p_rx = str;
    p_type = p_rx;
    p_rx = strchr(p_type, ',');

    if(p_rx != NULL)
    {
        *p_rx = 0;
        strcpy(type,p_type);
    }

    p_id_getway = p_rx+1;
    p_rx = strchr(p_id_getway, ',');
    if(p_rx != NULL)
    {
        *p_rx = 0;
        strcpy(ID_get_way,p_id_getway);
    }

    p_id_node = p_rx+1;
    p_rx = strchr(p_id_node, ',');
    if(p_rx != NULL)
    {
        *p_rx = 0;
        strcpy(ID_node,p_id_node);
    }

    p_mess = p_rx+1;
    p_rx = strchr(p_mess, ',');
    if(p_rx != NULL)
    {
        *p_rx = 0;
        strcpy(mess, p_mess);
    }

}
/**
  * @brief
  * @param
  * @retval
  */
void Task_Pair_RF_Connect(char *str)
{
    Button_Detect_Event(GPIOA, BUTTON_PAIR, &status_pair);
    char status_pair_arr[30];
    while(status_pair)
    {
        LED_ON();
        if(strstr(str, "pa") != NULL)
        {
            Flash_ProgramPage(str, FLASH_UID_ADDR);
            memset(str, 0, strlen(str));
        }
        Flash_ReadChar(status_pair_arr, FLASH_UID_ADDR, sizeof(status_pair_arr));

        if(strstr(status_pair_arr,"1") != NULL)
        {
            LED_OFF();
            status_pair = false;
        }
    }
}

/**
  * @brief
  * @param
  * @retval
  */

void Task_Ping_Status(char id_master[])
{
    printf(",pi,%s,1,1,\r\n", id_master);
}

/**
  * @brief
  * @param
  * @retval
  */

void Task_Send_Sensor_Status(char id_master[])
{
    bool status;
    MOTOR_DISABLE();

    Sensor_Detect_Event(GPIOA, SENSOR_PIN, &status);

    if(status)
    {
         printf(",lk,%s,1,1,\r\n", id_master);
    }

    else{
        printf(",lk,%s,1,0,\r\n", id_master);
    }
}

/**
  * @brief
  * @param
  * @retval
  */

void Task_Button_Control_L298(void)
{
    bool status_door;
    bool status_motor_t;
    bool status_sensor_left;
    bool status_sensor_right;
    Sensor_Detect_Event(GPIOA, PIN_CONTROL_MOTOR, &status_motor_t);
    Sensor_Detect_Event(GPIOB, PIN_CONTROL_LEFT, &status_sensor_left);
    Sensor_Detect_Event(GPIOB, PIN_CONTROL_RIGHT, &status_sensor_right);

    //open door
    if(status_motor_t == false && status_sensor_left == false && GPIO_ReadInputDataBit(GPIOA, SENSOR_PIN) == false)
    {

        time_motor = 0;
        /** mo cua*/
        LED_ON();
        OPEN_DOOR();
        Sensor_Detect_Event(GPIOB, PIN_CONTROL_RIGHT, &status_sensor_right);
        while(status_sensor_right != false)
        {
            Sensor_Detect_Event(GPIOB, PIN_CONTROL_RIGHT, &status_sensor_right);

        }
        MOTOR_DISABLE();
        while(time_motor < 5)
        {

            if(status_motor == false)
            {
                time_motor++;
                status_motor = true;
            }
        }

        Sensor_Detect_Event(GPIOA, SENSOR_PIN, &status_door);

        if(GPIO_ReadInputDataBit(GPIOA, SENSOR_PIN) == false)// cua van dong
        {
            /** Dong cua */
           CLOSE_DOOR();
           Sensor_Detect_Event(GPIOB, PIN_CONTROL_LEFT, &status_sensor_left);
            while(status_sensor_left != false)
            {
                Sensor_Detect_Event(GPIOB, PIN_CONTROL_LEFT, &status_sensor_left);
            }
            MOTOR_DISABLE();
            LED_OFF();
        }
    }

    if(GPIO_ReadInputDataBit(GPIOA, SENSOR_PIN) == false && status_sensor_right == false)// cua van dong
    {
        /** Dong cua */
       CLOSE_DOOR();
       Sensor_Detect_Event(GPIOB, PIN_CONTROL_LEFT, &status_sensor_left);
        while(status_sensor_left != false)
        {
            Sensor_Detect_Event(GPIOB, PIN_CONTROL_LEFT, &status_sensor_left);
        }
        MOTOR_DISABLE();
        LED_OFF();
    }
}

/**
  * @brief
  * @param
  * @retval
  */

void Task_Uart_Control_L298(char *str, char *ID_master)
{
    char type_rx[5];
    char id_master_rx[15];
    char id_node_rx[5];
    char mess_rx[5];
    bool status_door;
    bool status_motor_t;
    bool status_sensor_left;
    bool status_sensor_right;

    if(strlen(str) > 0)
    {
        Process_Message(str, type_rx, id_master_rx, id_node_rx, mess_rx);
        if((strstr(type_rx, "ct") != NULL) && (strstr(ID_master, id_master_rx) != NULL) && (strstr(id_node_rx,"1") != NULL) && (strstr(mess_rx,"1") != NULL))
        {
            time_motor = 0;
            /** mo cua*/
            OPEN_DOOR();
            LED_ON();
            Sensor_Detect_Event(GPIOB, PIN_CONTROL_RIGHT, &status_sensor_right);
            while(status_sensor_right != false)
            {
                Sensor_Detect_Event(GPIOB, PIN_CONTROL_RIGHT, &status_sensor_right);

            }
            MOTOR_DISABLE();
            while(time_motor < 5)
            {

                if(status_motor == false)
                {
                    time_motor++;
                    status_motor = true;
                }
            }

            Sensor_Detect_Event(GPIOA, SENSOR_PIN, &status_door);

            if(GPIO_ReadInputDataBit(GPIOA, SENSOR_PIN) == false && status_sensor_right == false)// cua van dong
            {
                /** Dong cua */
                CLOSE_DOOR();
                Sensor_Detect_Event(GPIOB, PIN_CONTROL_LEFT, &status_sensor_left);
                while(status_sensor_left != false)
                {
                    Sensor_Detect_Event(GPIOB, PIN_CONTROL_LEFT, &status_sensor_left);
                }
                MOTOR_DISABLE();
                LED_OFF();
            }
        }
    }
}


int main(void)
{
    memset(status_pair_arr, 0, sizeof(status_pair_arr));
    memset(type, 0, sizeof(type));
    memset(ID_master, 0, sizeof(ID_master));
    memset(id_node, 0, sizeof(id_node));
    memset(mess, 0, sizeof(mess));
    SysTick_Init();

    Gpio_Init();

    UART1_Init_A9A10(1200);
    UART3_Config(115200);
    GPIO_SetBits(GPIOA, RF_SET_PIN);
    GPIO_ResetBits(GPIOA, RF_CS_PIN);
    Timer_Init();
    LED_OFF();
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
    while(1)
    {
        Flash_ReadChar(status_pair_arr, FLASH_UID_ADDR, sizeof(status_pair_arr));
        Process_Message(status_pair_arr, type, ID_master, id_node, mess);
        Task_Pair_RF_Connect(uart1_rx);
        Task_Button_Control_L298();
        if((slot_rx_time == 1) &&  (tx1_status == false))
        {
            tx1_status = true;
            Task_Send_Sensor_Status(ID_master);
        }

        if((slot_rx_time == 2) &&  (rx_status == false))
        {
            rx_status = true;
            Task_Uart_Control_L298(uart1_rx, ID_master);

            memset(uart1_rx, 0, sizeof(uart1_rx));
        }

        if((slot_ping_time == 2) &&  (ping_status == false))
        {
            ping_status = true;
            Task_Ping_Status(ID_master);
        }

        else{
            __WFI();
            __SEV();
            __WFE();
        }
    }
}
