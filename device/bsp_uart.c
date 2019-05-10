
#include "nrf.h"
#include "nrf_gpio.h"
#include "nordic_common.h"
#include "bsp_uart.h"
#include "uart_protocol.h"

static void bsp_uart0_sw_init(void)
{
	g_tUart0.pUartTxBuf                 = g_UartTxBuf0;		    /* pointer of tx buffer */
	g_tUart0.pUartRxBuf                 = g_UartRxBuf0;			/* pointer of rx buffer */
    g_tUart0.pUartRxPackageReadIndex    = g_UartRxPackageBuf0;  /* pointer of package buffer */
	g_tUart0.usUartTxBufSize            = UART0_TX_BUF_SIZE;	/* size of tx buffer */
	g_tUart0.usUartRxBufSize            = UART0_RX_BUF_SIZE;	/* size of rx buffer */
	g_tUart0.usUartTxWriteIndex         = 0;					/* write index of tx fifo */
	g_tUart0.usUartTxReadIndex          = 0;					/* read index of tx fifo*/
	g_tUart0.usUartRxHeaderIndex        = 0;				    /* write index of rx fifo*/
	g_tUart0.usUartRxPackageIndex       = 0;					/* read index of rx fifo */
	g_tUart0.usUartTxCount              = 0;					/* count of tx data */
	g_tUart0.usUartRxBaseIndex          = 0;					/* count of rx data */
    g_tUart0.usUartRxPackageValid       = 0;                    /* flag of package status */
    g_tUart0.usUartRxReadFrameSize      = 0;                    /* count of received frame size */
}

static void bsp_uart0_hw_deinit(void)
{
    NVIC_DisableIRQ(UART0_IRQn);
    
    NRF_UART0->ENABLE           = (UART_ENABLE_ENABLE_Disabled << UART_ENABLE_ENABLE_Pos);
    
    NRF_GPIO->PIN_CNF[UART0_TX_PIN_NUMBER] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
                               | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
                               | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
                               | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
                               | (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);
    
    NRF_GPIO->PIN_CNF[UART0_RX_PIN_NUMBER] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
                               | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
                               | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
                               | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
                               | (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);
}

static void bsp_uart0_hw_init(eUartFlowControlStatus_TypeDef ctlFlowStatus)
{
    bsp_uart0_hw_deinit();
    nrf_gpio_cfg_output(UART0_TX_PIN_NUMBER);
    nrf_gpio_cfg_input(UART0_RX_PIN_NUMBER, NRF_GPIO_PIN_NOPULL);  

    NRF_UART0->PSELTXD = UART0_TX_PIN_NUMBER;
    NRF_UART0->PSELRXD = UART0_RX_PIN_NUMBER;

    if (ctlFlowStatus)
    {
#ifdef UART0_HWFLOW_ENABLE
        nrf_gpio_cfg_output(UART0_RTS_PIN_NUMBER);
        nrf_gpio_cfg_input(UART0_CTS_PIN_NUMBER, NRF_GPIO_PIN_NOPULL);
        NRF_UART0->PSELCTS = UART0_CTS_PIN_NUMBER;
        NRF_UART0->PSELRTS = UART0_RTS_PIN_NUMBER;
        NRF_UART0->CONFIG  = (UART_CONFIG_HWFC_Enabled << UART_CONFIG_HWFC_Pos);
#endif
    }

    NRF_UART0->BAUDRATE         = (UART_BAUDRATE_BAUDRATE_Baud921600 << UART_BAUDRATE_BAUDRATE_Pos);
    NRF_UART0->ENABLE           = (UART_ENABLE_ENABLE_Enabled << UART_ENABLE_ENABLE_Pos);
    //NRF_UART0->TASKS_STARTTX    = 1;
    NRF_UART0->TASKS_STOPTX     = 1;
    NRF_UART0->TASKS_STARTRX    = 1;
    NRF_UART0->EVENTS_RXDRDY    = 0;
}


static void bsp_uart0_int_init(void)
{
    NRF_UART0->INTENCLR = 0xffffffffUL;
    NRF_UART0->INTENSET = (UART_INTENSET_RXDRDY_Set << UART_INTENSET_RXDRDY_Pos) |
                          (UART_INTENSET_TXDRDY_Set << UART_INTENSET_TXDRDY_Pos) |
#ifdef UART0_HWFLOW_ENABLE
                          (UART_INTENSET_NCTS_Set << UART_INTENSET_NCTS_Pos)     |
                          (UART_INTENSET_CTS_Set << UART_INTENSET_CTS_Pos)       |
#endif
                          (UART_INTENSET_ERROR_Set << UART_INTENSET_ERROR_Pos);

    NVIC_ClearPendingIRQ(UART0_IRQn);
    NVIC_SetPriority(UART0_IRQn, 3);
    NVIC_EnableIRQ(UART0_IRQn);
}

void bsp_uart0_init(void)
{
    bsp_uart0_sw_init();
    bsp_uart0_hw_init(UART0_FLOW_CONTROLE_ENABLE);
    bsp_uart0_int_init();
}
