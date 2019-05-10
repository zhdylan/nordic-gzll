
#ifndef _BSP_UART_H_
#define _BSP_UART_H_

//#define UART0_HWFLOW_ENABLE
#define UART0_TX_PIN_NUMBER     6
#define UART0_RX_PIN_NUMBER     8
#define UART0_CTS_PIN_NUMBER    7
#define UART0_RTS_PIN_NUMBER    40

typedef enum
{
    UART0_FLOW_CONTROLE_DISABLE = 0,
    UART0_FLOW_CONTROLE_ENABLE = !UART0_FLOW_CONTROLE_DISABLE
}eUartFlowControlStatus_TypeDef;

void bsp_uart0_init(void);
#endif // End of _bsp_uart_ define