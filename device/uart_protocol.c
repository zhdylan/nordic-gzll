
#include "nrf_log.h"
#include "uart_protocol.h"

UartFifo_TypeDef g_tUart0;
byte g_UartTxBuf0[UART0_TX_BUF_SIZE] = {0};		                        /* uart0 tx buffer */
byte g_UartRxBuf0[UART0_RX_BUF_COUNT][UART0_RX_BUF_SIZE] = {0x00};		/* uart0 rx buffer */
byte g_UartRxPackageBuf0[UART0_RX_BUF_COUNT] = {0x00};

byte abtDecodedPackageTxBuffer[btBEH_PROTOCOL_Tx_BUFFER_SIZE];
byte abtDecodedPackageRxBuffer[btBEH_PROTOCOL_Rx_BUFFER_SIZE];
byte btDecodedRxFrameSize;
byte btDecodedTxFrameSize;

static void NRF_UART0_TX_(void)
{
    if(g_tUart0.usUartTxCount == 0x00)
    {
        NRF_UART0->TASKS_STARTRX    = 1;
    }
    else
    {
        NRF_UART0->TXD = g_tUart0.pUartTxBuf[g_tUart0.usUartTxReadIndex]; 
        if(++g_tUart0.usUartTxReadIndex >= g_tUart0.usUartTxBufSize)
        {
            g_tUart0.usUartTxReadIndex = 0x00;
        }
        g_tUart0.usUartTxCount--; 
     }
}
uint8_t uart_test_msg_buf[10] = {0xA2,0x00,};
static uint8_t btBEH_Protocol_BuildNewFrame( T_eUartSendCommand  eUartSendCmd)
{
    static uint8_t send_cnt = 0x00;
    
    uart_test_msg_buf[1] = send_cnt++;
    memcpy(abtDecodedPackageTxBuffer, uart_test_msg_buf, 10);
    btDecodedTxFrameSize = 10;    
}

void vBEH_Uart_NewFrame_(T_eUartSendCommand  eeUartSendCmd)
{
    uint8_t loop = 0;
    uint16_t usRead = 0;
    
    btBEH_Protocol_BuildNewFrame(eeUartSendCmd);
    
    /* this variable should be protected
                    cause it maybe overwrited in the uart intterrupt routines */
    CPU_CRITICAL_ENTER();
    usRead = g_tUart0.usUartTxCount;
    CPU_CRITICAL_EXIT();
    
    if(usRead > (g_tUart0.usUartTxBufSize - BEH_PROTOCOL_MAXIMUM_FRAME_SIZE))
    {
        return;
    }
    
    for(loop = BEH_PROTOCOL_BT_HEADER_INDEX; loop < btDecodedTxFrameSize; loop++)
    {
        g_tUart0.pUartTxBuf[g_tUart0.usUartTxWriteIndex] = abtDecodedPackageTxBuffer[loop];
        
        CPU_CRITICAL_ENTER();
        if(++g_tUart0.usUartTxWriteIndex >= g_tUart0.usUartTxBufSize)
        {
            g_tUart0.usUartTxWriteIndex = 0x00;
        }
        g_tUart0.usUartTxCount++;
        CPU_CRITICAL_EXIT();
    }

    //NRF_UART0->TASKS_STOPRX     = 1;
    NRF_UART0->TASKS_STARTTX    = 1;
    NRF_UART0_TX_();
}

void UART0_IRQHandler(void)
{
    uint8_t dummy;

    if (NRF_UART0->EVENTS_RXDRDY != 0)
    {
        NRF_UART0->EVENTS_RXDRDY = 0;

        dummy = (uint8_t)NRF_UART0->RXD;
        abtDecodedPackageRxBuffer[g_tUart0.usUartRxPackageIndex] = dummy;

        if(g_tUart0.usUartRxPackageIndex == 0x00)
        {
            if(0xAA == abtDecodedPackageRxBuffer[g_tUart0.usUartRxPackageIndex])
            {
                g_tUart0.usUartRxPackageIndex++;
            }
            else if(0xCC == abtDecodedPackageRxBuffer[g_tUart0.usUartRxPackageIndex])
            {
                g_tUart0.usUartRxPackageIndex++;
            }
            else if(0xEE == abtDecodedPackageRxBuffer[g_tUart0.usUartRxPackageIndex])
            {
                g_tUart0.usUartRxPackageIndex++;
            }
        }
        else
        {
            if(0xAA == abtDecodedPackageRxBuffer[0])
            {
                if(++g_tUart0.usUartRxPackageIndex == 0x02)
                {
                    btDecodedRxFrameSize = abtDecodedPackageRxBuffer[1];
                }
                
                if(g_tUart0.usUartRxPackageIndex >= btDecodedRxFrameSize + 1)
                {
                   
                    g_tUart0.usUartRxPackageIndex = 0x00;
                }
            }
            else if(0xCC == abtDecodedPackageRxBuffer[0])
            {
                if(++g_tUart0.usUartRxPackageIndex >= 11)
                {
                    //memcpy(uart_msg1[uart_msg_index++],abtDecodedPackageRxBuffer,11);
                    if(abtDecodedPackageRxBuffer[1] == 0xdd && abtDecodedPackageRxBuffer[2] == 0xee)
                    {
                       
                    }
                    g_tUart0.usUartRxPackageIndex = 0x00;
                }
            }
        }
    }

    if (NRF_UART0->EVENTS_TXDRDY != 0)
    {
        NRF_UART0->EVENTS_TXDRDY = 0;
        NRF_UART0_TX_();
    }

    
    if (NRF_UART0->EVENTS_CTS != 0 )
    {
        NRF_UART0->EVENTS_CTS = 0;
        NRF_LOG_INFO("CTS Event");
    }
    
    if (NRF_UART0->EVENTS_NCTS != 0 )
    {
        NRF_UART0->EVENTS_NCTS = 0;
        NRF_LOG_INFO("NCTS Event");
    }
    
    if (NRF_UART0->EVENTS_ERROR != 0)
    {
        uint32_t       error_source;
        NRF_UART0->EVENTS_ERROR = 0;
        error_source        = NRF_UART0->ERRORSRC;
        NRF_UART0->ERRORSRC = error_source;
        NRF_LOG_INFO("err: %d", error_source);
    }
}
