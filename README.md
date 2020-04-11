# UartFIFOandDMA
the Demo how to use stm32F103 MiroChip's Uart to  recieve and transmit data with fifo and dma

简介：
1.KEIL5开发环境，该工程演示了使用Stm32F103C8T6串口和PC进行通讯
2.串口2负责在中断中接受PC发送的串口数据，将数据写入FIFO中
3.在Main中将FIFO数据读取，并使用串口3的DMA的模式将PC的接受的数据再次发送给PC
4.使用5MS定时发送一段数据，自测试50万个字数数据，没有丢包。
