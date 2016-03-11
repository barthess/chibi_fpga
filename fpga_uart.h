#ifndef FPGA_UART_H_
#define FPGA_UART_H_

#include "fpga.h"

#define FPGA_UART_NUMBER      5
#define FPGA_UART_FIFO_SIZE   64
#define FPGA_UART_CLK         (100 * 1000 * 1000) // Hz

/**
 * @brief   Bridge state machine possible states.
 */
typedef enum {
  FPGAUART_BRIDGE_UNINIT  = 0,        /**< Not initialized.                   */
  FPGAUART_BRIDGE_STOP    = 1,        /**< Stopped.                           */
  FPGAUART_BRIDGE_READY   = 2,        /**< Ready.                             */
} fpgauartbridgestate_t;

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  FPGAUART_UNINIT  = 0,             /**< Not initialized.                   */
  FPGAUART_STOP    = 1,             /**< Stopped.                           */
  FPGAUART_READY   = 2,             /**< Ready.                             */
} fpgauartstate_t;

/**
 * @brief   Transmitter state machine states.
 */
typedef enum {
  FPGAUART_TX_IDLE = 0,                 /**< Not transmitting.                  */
  FPGAUART_TX_ACTIVE = 1,               /**< Transmitting.                      */
  FPGAUART_TX_COMPLETE = 2              /**< Buffer complete.                   */
} fpgauarttxstate_t;

/**
 * @brief   Receiver state machine states.
 */
typedef enum {
  FPGAUART_RX_IDLE = 0,                 /**< Not receiving.                     */
  FPGAUART_RX_ACTIVE = 1,               /**< Receiving.                         */
  FPGAUART_RX_COMPLETE = 2              /**< Buffer complete.                   */
} fpgauartrxstate_t;

/**
 * @brief   Hardware flow control mode
 */
typedef enum {
  FPGAUART_HW_FLOW_NONE = 0,
  FPGAUART_HW_FLOW_CTS = 1,
  FPGAUART_HW_FLOW_RTS = 2,
  FPGAUART_HW_FLOW_BOTH = 3
} fpgauarthwflow_t;

/**
 * @brief   Forward declaration.
 */
typedef struct FPGAUARTDriver     FPGAUARTDriver;
typedef struct FPGAUARTBridge_t   FPGAUARTBridge_t;

/**
 *
 */
typedef struct {
  volatile fpgaword_t CTL[8];
  volatile fpgaword_t tx_fill;    // number of bytes in TX fifo
  volatile fpgaword_t rx_avail;   // number of bytes in RX fifo
  volatile fpgaword_t tx_trig;    // TX fifo trigger level
  volatile fpgaword_t rx_trig;    // RX fifo trigger level
  volatile fpgaword_t reserved[4];
} FPGAUART_TypeDef;

/**
 *
 */
typedef struct {
  FPGAUART_TypeDef UART[FPGA_UART_NUMBER];
  volatile fpgaword_t ISR; // interrupt status register
  volatile fpgaword_t RCR; // reset control register
} FPGAUARTBRIDGE_TypeDef;

/**
 *
 */
typedef struct {
  uint32_t              baudrate;
  fpgauarthwflow_t      flow_control;
} FPGAUARTConfig;

/**
 *
 */
struct  FPGAUARTDriver {
  size_t                idx; // number of this UART in memory mapper array
  fpgauartstate_t       state;
  fpgauarttxstate_t     txstate;
  fpgauartrxstate_t     rxstate;
  const FPGAUARTConfig  *config;
  FPGAUART_TypeDef      *uart;

  /* helper fields for transaction split */
  uint8_t               *rxbuf;
  size_t                rxbytes;
};

/**
 *
 */
struct  FPGAUARTBridge_t {
  FPGAUARTDriver          FPGAUARTD[FPGA_UART_NUMBER];
  FPGAUARTBRIDGE_TypeDef  *bridge;
  fpgauartbridgestate_t   state;
};

/**
 *
 */
extern FPGAUARTBridge_t FPGAUARTBridge;

/**
 *
 */
#ifdef __cplusplus
extern "C" {
#endif
  void fpgaUartObjectInit(FPGAUARTBridge_t *bridgep, const FPGADriver *fpgap);
  void fpgaUartBridgeStart(FPGAUARTBridge_t *bridgep);
  void fpgaUartBridgeStop(FPGAUARTBridge_t *bridgep);

  void fpgaUartStart(FPGAUARTDriver *uartp, const FPGAUARTConfig *cfgp);
  void fpgaUartStop(FPGAUARTDriver *uartp);
  size_t fpgaUartStartWrite(FPGAUARTDriver *uartp, size_t n, const uint8_t *txbuf);
  size_t fpgaUartStartRead(FPGAUARTDriver *uartp, size_t n, uint8_t *rxbuf);
#ifdef __cplusplus
}
#endif

#endif /* FPGA_PWM_H_ */
