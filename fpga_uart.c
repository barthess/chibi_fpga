#include "main.h"
#include "pads.h"

#include "fpga_uart.h"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
#define FCR_FIFO_EN       (1U << 0)
#define FCR_FIFO_RX_RST   (1U << 1) // self clearing
#define FCR_FIFO_TX_RST   (1U << 2) // self clearing
#define FCR_FIFO_64BIT    (1U << 5) // sets bigger than 16 bytes FIFO size

#define LCR_DLAB          (1U << 7)

#define MCR_RTS           (1U << 1)
#define MCR_AFE           (1U << 5)

/* baked registers numbers in order of appearance in TL16C750 datasheet */
#define CTL_THR   0
#define CTL_RBR   0
#define CTL_IER   1
#define CTL_IIR   2
#define CTL_FCR   2
#define CTL_LCR   3
#define CTL_MCR   4
#define CTL_LSR   5
#define CTL_MSR   6
#define CTL_SCR   7
#define CTL_DLL   0
#define CTL_DLM   1

#define RX_SPLIT_TRIG   ((FPGA_UART_FIFO_SIZE * 3) / 4)

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */
FPGAUARTBridge_t FPGAUARTBridge;

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

/*
 ******************************************************************************
 * PROTOTYPES
 ******************************************************************************
 */

/*
 *******************************************************************************
 *******************************************************************************
 * LOCAL FUNCTIONS
 *******************************************************************************
 *******************************************************************************
 */

/**
 *
 */
void assert_reset(FPGAUARTDriver *uartp) {

  FPGAUARTBridge.bridge->RCR |= 1U << uartp->idx;
}

/**
 *
 */
void release_reset(FPGAUARTDriver *uartp) {

  FPGAUARTBridge.bridge->RCR &= ~(1U << uartp->idx);
}

/*
 *******************************************************************************
 * EXPORTED FUNCTIONS
 *******************************************************************************
 */
/**
 *
 */
void fpgaUartObjectInit(FPGAUARTBridge_t *bridgep, const FPGADriver *fpgap) {

  bridgep->bridge = (FPGAUARTBRIDGE_TypeDef *)fpgaGetSlicePtr(fpgap, FPGA_WB_SLICE_UART);

  for (size_t i=0; i<FPGA_UART_NUMBER; i++) {
    FPGAUARTDriver *u = &bridgep->FPGAUARTD[i];
    u->idx      = i;
    u->state    = FPGAUART_STOP;
    u->txstate  = FPGAUART_TX_IDLE;
    u->rxstate  = FPGAUART_RX_IDLE;
    u->config   = NULL;
    u->uart     = &bridgep->bridge->UART[i];
  }

  bridgep->state = FPGAUART_BRIDGE_STOP;
}

/**
 *
 */
void fpgaUartBridgeStart(FPGAUARTBridge_t *bridgep) {

  osalDbgCheck(NULL != bridgep);
  osalDbgAssert((bridgep->state == FPGAUART_BRIDGE_STOP) || (bridgep->state == FPGAUART_BRIDGE_READY),
                "invalid state");

  bridgep->bridge->RCR = 0xFF & (~0);

  bridgep->state = FPGAUART_BRIDGE_READY;
}

/**
 *
 */
void fpgaUartBridgeStop(FPGAUARTBridge_t *bridgep) {

  osalDbgCheck(NULL != bridgep);
  osalDbgAssert((bridgep->state == FPGAUART_BRIDGE_STOP) || (bridgep->state == FPGAUART_BRIDGE_READY),
                "invalid state");

  for (size_t i=0; i<FPGA_UART_NUMBER; i++) {
    FPGAUARTDriver *u = &bridgep->FPGAUARTD[i];
    osalDbgAssert(FPGAUART_STOP == u->state, "all UARTs must be stopped before");
  }

  bridgep->state = FPGAUART_BRIDGE_STOP;
}

/**
 *
 */
void fpgaUartStart(FPGAUARTDriver *uartp, const FPGAUARTConfig *config) {

  osalDbgCheck((uartp != NULL) && (config != NULL));

  osalSysLock();
  osalDbgAssert((uartp->state == FPGAUART_STOP) || (uartp->state == FPGAUART_READY),
                "invalid state");

  uartp->config = config;

  FPGAUART_TypeDef *u = uartp->uart;

  /* release UART reset */
  release_reset(uartp);

  /* switch hardware flow control */
  if (FPGAUART_HW_FLOW_BOTH == config->flow_control) {
    u->CTL[CTL_MCR] = MCR_RTS | MCR_AFE;
  }

  /* switch banks for tuning */
  u->CTL[CTL_LCR] |= LCR_DLAB;

  /* set baudrate */
  uint32_t div = FPGA_UART_CLK / (config->baudrate * 16);
  u->CTL[CTL_DLL] = div & 0xFF;
  u->CTL[CTL_DLM] = (div >> 8) & 0xFF;

  /* reset and tune FIFO */
  u->CTL[CTL_FCR] = FCR_FIFO_EN | FCR_FIFO_RX_RST | FCR_FIFO_TX_RST | FCR_FIFO_64BIT;
  u->tx_trig = 0;
  u->rx_trig = 0;

  /* set frame length and clear DLAB bit */
  u->CTL[CTL_LCR] = 3;

  uartp->state = FPGAUART_READY;
  osalSysUnlock();
}

/**
 *
 */
void fpgaUartStop(FPGAUARTDriver *uartp) {
  osalDbgCheck(uartp != NULL);

  osalSysLock();
  osalDbgAssert((uartp->state == FPGAUART_STOP) || (uartp->state == FPGAUART_READY),
                "invalid state");

  assert_reset(uartp);

  uartp->state   = FPGAUART_STOP;
  uartp->txstate = FPGAUART_TX_IDLE;
  uartp->rxstate = FPGAUART_RX_IDLE;
  osalSysUnlock();
}

/**
 *
 */
void push_fifo(FPGAUARTDriver *uartp, size_t n, const uint8_t *txbuf) {
  for (size_t i=0; i<n; i++) {
    uartp->uart->CTL[CTL_THR] = txbuf[i];
  }
}

/**
 *
 */
void read_fifo(FPGAUARTDriver *uartp, size_t n, uint8_t *rxbuf) {

  for (size_t i=0; i<n; i++) {
    rxbuf[i] = uartp->uart->CTL[CTL_RBR] & 0xFF;
  }
}

/**
 *
 */
size_t fpgaUartGetTxFee(FPGAUARTDriver *uartp) {

  osalDbgAssert((uartp->state == FPGAUART_STOP) || (uartp->state == FPGAUART_READY),
                "invalid state");

  return FPGA_UART_FIFO_SIZE - uartp->uart->tx_fill;
}

/**
 *
 */
size_t fpgaUartGetRxAvail(FPGAUARTDriver *uartp) {

  osalDbgAssert((uartp->state == FPGAUART_STOP) || (uartp->state == FPGAUART_READY),
                "invalid state");

  return uartp->uart->rx_avail;
}

/**
 * @brief   Push data to TX fifo
 * @retval  Number of bytes actually placed to fifo
 */
size_t fpgaUartWrite(FPGAUARTDriver *uartp, size_t n, const uint8_t *txbuf) {

  osalDbgAssert((uartp->state == FPGAUART_STOP) || (uartp->state == FPGAUART_READY),
                "invalid state");

  size_t empty = FPGA_UART_FIFO_SIZE - uartp->uart->tx_fill;

  if (empty <= n) {
    push_fifo(uartp, n, txbuf);
    return n;
  }
  else {
    push_fifo(uartp, empty, txbuf);
    return empty;
  }
}

/**
 * @brief   Receive data from RX fifo.
 */
size_t fpgaUartRead(FPGAUARTDriver *uartp, size_t n, uint8_t *rxbuf) {

  osalDbgAssert((uartp->state == FPGAUART_STOP) || (uartp->state == FPGAUART_READY),
                "invalid state");

  size_t avail = uartp->uart->rx_avail;

  if (avail >= n) {
    read_fifo(uartp, n, rxbuf);
    return n;
  }
  else {
    read_fifo(uartp, avail, rxbuf);
    return avail;
  }
}


