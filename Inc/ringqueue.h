/**
  ******************************************************************************
  * @file           : ringqueue.h
  * @author         : liuchengfei
  * @version        : v1.0
  * @date           : 2017.09
  * @brief          : 
  *
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RINGQUEUE_H
#define __RINGQUEUE_H	 
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h" 
#include <stdbool.h>
/* Private types -------------------------------------------------------------*/
/**
  * @brief  ���д�С
  */
#define  MAXSIZE   200

/**
  * @brief  
  */
typedef uint8_t QElemType;

/**
  * @brief  ѭ������˳��洢�ṹ 
  */
typedef struct
{
	QElemType data[MAXSIZE];
	uint8_t front;   /* ͷָ�� */
	uint8_t rear;    /* βָ�� */
}SqQueue;

/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/ 
void QueueInit(SqQueue *Q);
uint8_t QueueGetLen(SqQueue *Q);
bool EnQueue(SqQueue *Q,QElemType e);
bool DeQueue(SqQueue *Q,QElemType *e);
bool Is_Queue_Empty(SqQueue *Q); 
bool Is_Queue_Full(SqQueue *Q);


#endif /* __RINGQUEUE_H */
/************************ (C) COPYRIGHT ZiFiSense *****END OF FILE****/

