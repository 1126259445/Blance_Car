/**
*******************************************************************************
* @file      serial.h
* @version   V1.0
* @date      2016.11.23
* @brief     ????
*******************************************************************************
* @copy
*
* @author	F-Series Group
*******************************************************************************
*/

/*---------------------------- Include ---------------------------------------*/
#ifndef _serial_h
#define _serial_h

#include "stm32f10x.h"
#include "stm32f10x_dma.h"

#define SERIAL_DEFAULT_RX_BUFSIZE	128

typedef void serialTxDMACallback_t(void *s);

typedef struct
{
    unsigned int baudRate;
    uint16_t flowControl;
    uint16_t parity;
    uint16_t stopBits;

    unsigned int txBufSize;
    volatile unsigned char *txBuf;
    volatile unsigned int txHead, txTail;

    unsigned int rxBufSize;
    volatile unsigned char *rxBuf;
    volatile unsigned int rxHead, rxTail;
    volatile unsigned int rxPos;

    USART_TypeDef *USARTx;
    DMA_Channel_TypeDef *rxDMAChannel;
    DMA_Channel_TypeDef *txDMAChannel;
    uint32_t rxDmaFlags, txDmaFlags;
    volatile unsigned char txDmaRunning;
    serialTxDMACallback_t *txDMACallback;
    void *txDMACallbackParam;

    uint8_t waitFlag;
} serialPort_t;

extern serialPort_t *serialOpen(USART_TypeDef *USARTx, unsigned int baud, uint16_t flowControl, unsigned int rxBufSize, unsigned int txBufSize);
extern void serialChangeBaud(serialPort_t *s, unsigned int baud);
extern void serialChangeParity(serialPort_t *s, uint16_t parity);
extern void serialChangeStopBits(serialPort_t *s, uint16_t stopBits);
extern void serialSetSTDIO(serialPort_t *s);
extern void serialWrite(serialPort_t *s, unsigned char ch);
extern void serialWriteBlock(serialPort_t *s, unsigned char *buf, unsigned char len);
extern void serialWatch(void);
extern unsigned char serialAvailable(serialPort_t *s);
extern uint16_t serialAvaiableCount(serialPort_t *s);
extern int serialRead(serialPort_t *s);
extern int serialReadBlock(serialPort_t *s, uint8_t *buf, uint16_t len);
extern void serialPrint(serialPort_t *s, const char *str);
extern int _serialStartTxDMA(serialPort_t *s, void *buf, int size, serialTxDMACallback_t *txDMACallback, void *txDMACallbackParam);
extern int __putchar(int ch);
extern int __getchar(void);

extern serialPort_t *serialSTDIO;

#endif /* __serial_h */

/* end of file */
