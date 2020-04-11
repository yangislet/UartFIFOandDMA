/**
  ******************************************************************************
  * @file           : ringqueue.c
  * @author         : liuchengfei
  * @version        : v1.0
  * @date           : 2017.09
  * @brief          : 
  *
  ******************************************************************************
  * @attention  顺序存储循环队列
  *
  *
  ******************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/
#include"ringqueue.h" 
#include <stdbool.h>
#include <stdint.h>

/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private function ----------------------------------------------------------*/

/* Exported function ----------------------------------------------------------*/

/**
  * @brief  循环队列初始化
  * @param  None
  * @retval None
  */
 void QueueInit(SqQueue* Q)
 {
	Q->front = 0;
   	Q->rear = 0;
 }
/**
  * @brief  获取循环队列数据长度
  * @param  None
  * @retval None
  */
 uint8_t QueueGetLen(SqQueue *Q)
 {
	 return (Q->rear  + MAXSIZE - Q->front)%MAXSIZE;
	 //return (Q->rear - Q->front + MAXSIZE)%MAXSIZE;  //不知道是否溢出,不会
 }
 /**
  * @brief  从尾部入队列
  * @param  None
  * @retval None
  */
 bool EnQueue(SqQueue*Q,QElemType e)
 {
	 if((Q->rear + 1)%MAXSIZE == Q->front)   /* 满 */
		 return false;
	 Q->data[Q->rear] = e;
	 Q->rear =  (Q->rear +1 )%MAXSIZE;
     return true;
 }
  /**
 * @brief  从头部出队列
  * @param  None
  * @retval None
  */
bool DeQueue(SqQueue*Q,QElemType* e)
{
  if(Q->front == Q->rear)   /* 空 */
		return false;
	*e = Q->data[Q->front];
	Q->front = (Q->front + 1)%MAXSIZE;
    return true;
 }
 
   /**
  * @brief  判断队列是否为空
  * @param  None
  * @retval None
  */	

bool Is_Queue_Empty(SqQueue*Q)
{
    return Q->front == Q->rear;   /* 空 */ 
}

/**
* @brief  判断队列是否为满
* @param  None
* @retval None
*/	

bool Is_Queue_Full(SqQueue*Q)
{
    return (Q->rear + 1)%MAXSIZE == Q->front;   /* 满 */ 
}







/************************ (C) COPYRIGHT ZiFiSense *****END OF FILE****/

