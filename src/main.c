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

#define PIN_IN1_L298              GPIO_Pin_12
#define PIN_IN2_L298              GPIO_Pin_13
#define PIN_CONTROL_L298          GPIO_Pin_14
#define PIN_CONTROL_LEFT          GPIO_Pin_8
#define PIN_CONTROL_RIGHT         GPIO_Pin_9
#define PIN_CONTROL_MOTOR         GPIO_Pin_2

#define LED_ON()                  GPIO_SetBits(GPIOA, GPIO_Pin_5)
#define LED_OFF()                 GPIO_ResetBits(GPIOA, GPIO_Pin_5)

#define MOTOR_DISABLE()           GPIO_ResetBits(GPIOB, PIN_IN1_L298), GPIO_ResetBits(GPIOB, PIN_IN2_L298)
#define OPEN_DOOR()               GPIO_SetBits(GPIOB, PIN_IN1_L298), GPIO_ResetBits(GPIOB, PIN_IN2_L298)
#define CLOSE_DOOR()              GPIO_SetBits(GPIOB, PIN_IN2_L298), GPIO_ResetBits(GPIOB, PIN_IN1_L298)

#define STM32_UUID                ((uint32_t *)0x1FFFF7E8)
#define FLASH_UID_ADDR            0x08007C00

bool status_pair = false;
volatile uint8_t slot_rx_time = 0;
volatile uint8_t sync_send_time = 0;
volatile bool sync_status = true;
volatile bool tx1_status = true;
volatile bool rx_status = true;
volatile bool ping_status = true;
volatile bool status_motor = true;
volatile uint8_t time_motor = true;

extern char Rx_bufArr[128];
extern char uart1_rx[128];

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
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStructure);
}

void TIM2_IRQHandler(void)
{
    if(TIM_GetFlagStatus(TIM2, TIM_IT_Update) != RESET)
    {
        slot_rx_time++;
        status_motor = false;
        if(slot_rx_time == 1) tx1_status = false;
        if(slot_rx_time == 3) rx_status = false;
        if(slot_rx_time == 5) ping_status = false;
        if(slot_rx_time > 10)
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
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitTypeDef   GPIO_Init_Structure;
    GPIO_Init_Structure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init_Structure.GPIO_Pin   = GPIO_Pin_13;
    GPIO_Init_Structure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOC, &GPIO_Init_Structure);

    GPIO_Init_Structure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init_Structure.GPIO_Pin   = PIN_IN1_L298|PIN_IN2_L298|PIN_CONTROL_L298;
    GPIO_Init_Structure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOB, &GPIO_Init_Structure);

    GPIO_Init_Structure.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_Init_Structure.GPIO_Pin   = PIN_CONTROL_LEFT|PIN_CONTROL_RIGHT;
    GPIO_Init_Structure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOB, &GPIO_Init_Structure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_Init_Structure.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_Init_Structure.GPIO_Pin   = BUTTON_PAIR|SENSOR_PIN|PIN_CONTROL_MOTOR|GPIO_Pin_5;
    GPIO_Init_Structure.GPIO_Speed = GPIO_Speed_10MHz;
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

    else if(GPIO_ReadInputDataBit(GPIOx, GPIO_Pin) == 0 && status_button == true)
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

    else if(GPIO_ReadInputDataBit(GPIOx, GPIO_Pin) == 0 && status_sensor == true)
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
bool Process_Message(char *str, char *type, char *ID_get_way, char *ID_node, char *mess)
{
    char *p_rx;
    char *p_type;
    char *p_id_getway;
    char *p_id_node;
    char *p_mess;
    if((str != NULL) && (type != NULL) && (ID_get_way != NULL) && (ID_node != NULL) && (mess != NULL))
    {
        p_rx = str;
        p_type = p_rx;
        p_rx = strchr(p_type,',');
        if(p_rx != NULL)
        {
            *p_rx = 0;
            strcpy(type,p_type);
        }
        p_id_getway = p_rx+1;
        p_rx = strchr(p_id_getway,',');
        if(p_rx != NULL)
        {
            *p_rx = 0;
            strcpy(ID_get_way,p_id_getway);
        }
        p_id_node = p_rx+1;
        p_rx = strchr(p_id_node,',');
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
        return true;
    }

    else{
        return false;
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
    char status_pair_arr[22];
    while(status_pair)
    {
        LED_ON();
        if(strstr(str, "pair") != NULL)
        {
            UART_PutStr(USART3, "READ_RX_MASTER: ");// debug
            UART_PutStr(USART3, str);// debug
            __disable_irq();
            Flash_ProgramPage(str, FLASH_UID_ADDR);
            __enable_irq();
            memset(str, 0, strlen(str));
        }
        Flash_ReadChar(status_pair_arr, FLASH_UID_ADDR, sizeof(status_pair_arr));

        if(strstr(status_pair_arr,"pair") != NULL)
        {
            UART_PutStr(USART3, "READ_FLASH: ");// debug
            UART_PutStr(USART3, status_pair_arr);// debug
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

void Task_Ping_Status(void)
{
    char status_pair_arr[22];
    char type[5];
    char id_master[15];
    char id_node[5];
    char mess[5];
    Flash_ReadChar(status_pair_arr, FLASH_UID_ADDR, sizeof(status_pair_arr));
    Process_Message(status_pair_arr, type, id_master, id_node, mess);
    printf("ping,%s,%s,true,\n", id_master, id_node);
}

/**
  * @brief
  * @param
  * @retval
  */

void Task_Send_Sensor_Status(void)
{
    bool status;
    char status_pair_arr[22];
    char type[5];
    char id_master[15];
    char id_node[5];
    char mess[5];
    MOTOR_DISABLE();
    Flash_ReadChar(status_pair_arr, FLASH_UID_ADDR, sizeof(status_pair_arr));
    Process_Message(status_pair_arr, type, id_master, id_node, mess);
    Sensor_Detect_Event(GPIOA, SENSOR_PIN, &status);

    if(status)
    {
         printf("lock,%s,%s,lock,\n", id_master, id_node);
    }

    else{
        printf("lock,%s,%s,open,\n", id_master, id_node);
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

    //open door
    if(status_motor_t == false && status_sensor_left == false)
    {
        time_motor = 0;
        /** mo cua*/
        LED_ON();
        GPIO_SetBits(GPIOB, PIN_IN1_L298);
        GPIO_ResetBits(GPIOB, PIN_IN2_L298);
        Sensor_Detect_Event(GPIOB, PIN_CONTROL_RIGHT, &status_sensor_right);
//
        while(status_sensor_right != false)
        {
            Sensor_Detect_Event(GPIOB, PIN_CONTROL_RIGHT, &status_sensor_right);

        }
        MOTOR_DISABLE();
        while(time_motor < 10)
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
            GPIO_SetBits(GPIOB, PIN_IN2_L298);
            GPIO_ResetBits(GPIOB, PIN_IN1_L298);
            Sensor_Detect_Event(GPIOA, PIN_CONTROL_LEFT, &status_sensor_left);

            if(status_sensor_left == false)
            {
                MOTOR_DISABLE();
            }
            LED_OFF();
        }
    }
}

/**
  * @brief
  * @param
  * @retval
  */

void Task_Uart_Control_L298(char *str)
{
    char status_pair_arr[22];
    char type[5], type_rx[5];
    char id_master[15],id_master_rx[15];
    char id_node[5],id_node_rx[5];
    char mess[5],mess_rx[5];
    bool status_door;
    bool status_motor_t;
    bool status_sensor_left;
    bool status_sensor_right;

    if(strlen(str) > 0)
    {
        Flash_ReadChar(status_pair_arr, FLASH_UID_ADDR, sizeof(status_pair_arr));
        Process_Message(str, type_rx, id_master_rx, id_node_rx, mess_rx);
        Process_Message(status_pair_arr, type, id_master, id_node, mess);
        if((strstr(type_rx, "ctrl") != NULL) && (strstr(id_master, id_master_rx) != NULL) && (strstr(id_node_rx,id_node) != NULL) && (strstr(mess_rx,"open") != NULL))
        {
            time_motor = 0;
            /** mo cua*/
             OPEN_DOOR();
            Sensor_Detect_Event(GPIOB, PIN_CONTROL_RIGHT, &status_sensor_right);
            while(status_sensor_right != false)
            {
                Sensor_Detect_Event(GPIOB, PIN_CONTROL_RIGHT, &status_sensor_right);

            }
            MOTOR_DISABLE();
            while(time_motor < 10)
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
                Sensor_Detect_Event(GPIOA, PIN_CONTROL_LEFT, &status_sensor_left);

                if(status_sensor_left == false)
                {
                    MOTOR_DISABLE();
                }
            }
        }
    }
}

int main(void)
{
    SysTick_Init();
    Timer_Init();
    Gpio_Init();

    UART1_Init_A9A10(19200);
    UART3_Config(19200);

    LED_OFF();
   // while(strstr(uart1_rx, "sync") == NULL);
    while(1)
    {
        Task_Pair_RF_Connect(uart1_rx);
        Task_Button_Control_L298();
        if(strstr(uart1_rx, "sync") != NULL)
        {
            slot_rx_time = 0;
            memset(uart1_rx, 0 , sizeof(uart1_rx));
        }

        if((slot_rx_time == 1) &&  (tx1_status == false))
        {
            tx1_status = true;
            Task_Send_Sensor_Status();
        }

        if((slot_rx_time == 3) &&  (rx_status == false))
        {
            rx_status = true;
            UART_PutStr(USART3, uart1_rx);
            Task_Uart_Control_L298(uart1_rx);
            memset(uart1_rx, 0, sizeof(uart1_rx));
        }

        if((slot_rx_time == 5) &&  (ping_status == false))
        {
            ping_status = true;
            Task_Ping_Status();
            //UART_PutStr(USART3, uart1_rx);
            //memset(uart1_rx, 0, sizeof(uart1_rx));
        }
    }
}
