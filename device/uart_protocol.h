
#ifndef _UART_PROTOCOL_H_
#define _UART_PROTOCOL_H_

#include "nrf.h"
#include "nordic_common.h"

#define UART0_BAUDRATE  	115200
#define UART0_TX_BUF_SIZE	2*21
#define UART0_RX_BUF_SIZE	10
#define UART0_RX_BUF_COUNT  10
#define btBEH_UART_RX_BUFFER_COUNT 20

#define btBEH_PROTOCOL_Rx_BUFFER_SIZE           (byte)(21)//max byte
#define btBEH_PROTOCOL_Tx_BUFFER_SIZE           (byte)(21)//max Byte
#define BEH_PROTOCOL_MAXIMUM_FRAME_SIZE 36

#define  CPU_CRITICAL_ENTER()  do { __disable_irq(); } while (0)          /* Disable   interrupts.                      */
#define  CPU_CRITICAL_EXIT()   do { __enable_irq();  } while (0)          /* Re-enable interrupts.                      */

#define byte uint8_t

typedef struct
{
    uint8_t         *pUartTxBuf;
    uint8_t         *pUartRxPackageReadIndex;
    uint8_t         (*pUartRxBuf)[UART0_RX_BUF_SIZE];

    //T_eUartStatusTypeDef        btUartStatus;
    
    uint8_t         usUartTxBufSize;
    uint8_t         usUartRxBufSize;
    
    uint8_t         usUartTxWriteIndex;
    uint8_t         usUartTxReadIndex;
    uint8_t         usUartTxCount;
    
    uint8_t         usUartRxHeaderIndex;
    uint8_t         usUartRxPackageIndex;
    uint8_t         usUartRxBaseIndex;
    uint8_t         usUartRxReadFrameSize;
    
    uint8_t         usUartRxPackageValid;
    
    //void (*uartSendBeforeCallback)(void);
    //void (*uartSendAfterCallback)(void);
    //void (*uartReceiveNewCallback)(void);
    
}UartFifo_TypeDef;

typedef enum
{
		BEH_PROTOCOL_BT_HEADER_INDEX=0x00,
		BEH_PROTOCOL_BT_SIZE_INDEX,
		BEH_PROTOCOL_BT_COMMAND_ID_INDEX,
		BEH_PROTOCOL_BT_PARAMETER_INDEX	
}T_eTxBufferIndex;

typedef enum
{
    UART_STATUS_READY = 0x00,
    UART_STATUS_SENDING,
    UART_STATUS_WAITTING,
    UART_STATUS_READING,
    UART_STATUS_INVALID
}T_eUartStatusTypeDef;

typedef enum
{
    UART_SEND_DFU_START                     = 0x00,
	UART_SEND_CONTROLLER_DATA,
	UART_SEND_RFM_SW_VERSION,
    UART_SEND_RFS_SW_VERSION,
    UART_SEND_DFU_ACK,
    UART_SEND_DFU_DONE,
    UART_SEND_RAW_DATA
}T_eUartSendCommand;

extern UartFifo_TypeDef g_tUart0;
extern byte g_UartTxBuf0[UART0_TX_BUF_SIZE];		                    /* uart0 tx buffer */
extern byte g_UartRxBuf0[UART0_RX_BUF_COUNT][UART0_RX_BUF_SIZE];		/* uart0 rx buffer */
extern byte g_UartRxPackageBuf0[UART0_RX_BUF_COUNT];

extern byte abtDecodedPackageTxBuffer[btBEH_PROTOCOL_Tx_BUFFER_SIZE];
extern byte abtDecodedPackageRxBuffer[btBEH_PROTOCOL_Rx_BUFFER_SIZE];

void vBEH_Uart_NewFrame_(T_eUartSendCommand  eeUartSendCmd);
#endif // End of _uart_protocol_ define
