#ifndef HOST_HAL_H
#define HOST_HAL_H
#include <stdint.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { uint32_t TXBRP, TXBTO, TXBCF, TXEFS; } FDCAN_GlobalTypeDef;
typedef struct { FDCAN_GlobalTypeDef *Instance; } FDCAN_HandleTypeDef;
typedef struct { struct { uint32_t Mode; } Init; } DMA_HandleTypeDef;
typedef struct {
    void *Instance;
    DMA_HandleTypeDef *hdmarx;
    uint32_t ErrorCode;
} UART_HandleTypeDef;
typedef struct { int unused; } SPI_HandleTypeDef;
typedef struct { uint16_t ODR; } GPIO_TypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET } GPIO_PinState;
typedef enum {
    HAL_UART_RXEVENT_TC = 0, HAL_UART_RXEVENT_HT, HAL_UART_RXEVENT_IDLE
} HAL_UART_RxEventTypeTypeDef;
extern GPIO_TypeDef host_gpio_d, host_gpio_e, host_gpio_f;
#define GPIOD (&host_gpio_d)
#define GPIOE (&host_gpio_e)
#define GPIOF (&host_gpio_f)
#define GPIO_PIN_0 ((uint16_t)0x0001)
#define GPIO_PIN_5 ((uint16_t)0x0020)
#define GPIO_PIN_6 ((uint16_t)0x0040)
#define GPIO_PIN_15 ((uint16_t)0x8000)
#define USART2 ((void *)2)
#define USART3 ((void *)3)
#define DMA_CIRCULAR 1U
#define HAL_UART_ERROR_NONE 0U
#define HAL_UART_ERROR_ORE 8U
#define __DMB() ((void)0)
void HAL_GPIO_WritePin(GPIO_TypeDef *, uint16_t, GPIO_PinState);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *, uint16_t);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_UART_RxEventTypeTypeDef HAL_UARTEx_GetRxEventType(const UART_HandleTypeDef *);
uint32_t HAL_UART_GetError(const UART_HandleTypeDef *);
typedef struct {
    uint32_t Identifier, IdType, TxFrameType, DataLength, ErrorStateIndicator;
    uint32_t BitRateSwitch, FDFormat, TxEventFifoControl, MessageMarker;
} FDCAN_TxHeaderTypeDef;
typedef struct { uint32_t Identifier, IdType, RxFrameType, DataLength; } FDCAN_RxHeaderTypeDef;
typedef struct { uint32_t Identifier, DataLength, MessageMarker, TxTimestamp; } FDCAN_TxEventFifoTypeDef;
typedef struct { uint32_t BusOff, LastErrorCode, Activity; } FDCAN_ProtocolStatusTypeDef;
typedef struct { uint32_t TxErrorCnt, RxErrorCnt; } FDCAN_ErrorCountersTypeDef;
#define FDCAN_STANDARD_ID 0U
#define FDCAN_DATA_FRAME 0U
#define FDCAN_ESI_ACTIVE 0U
#define FDCAN_BRS_OFF 0U
#define FDCAN_CLASSIC_CAN 0U
#define FDCAN_NO_TX_EVENTS 0U
#define FDCAN_STORE_TX_EVENTS 1U
/* Match this repository's bundled STM32H7 HAL: DataLength is a raw nibble,
 * already shifted down from the Message RAM header by HAL_GetRxMessage. */
#define FDCAN_DLC_BYTES_0 0U
#define FDCAN_DLC_BYTES_1 1U
#define FDCAN_DLC_BYTES_2 2U
#define FDCAN_DLC_BYTES_3 3U
#define FDCAN_DLC_BYTES_4 4U
#define FDCAN_DLC_BYTES_5 5U
#define FDCAN_DLC_BYTES_6 6U
#define FDCAN_DLC_BYTES_7 7U
#define FDCAN_DLC_BYTES_8 8U
#define FDCAN_TXEFS_EFFL 0x3FU
#define FDCAN_RX_FIFO0 0U
#define __disable_irq() ((void)0)
#define __enable_irq() ((void)0)
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *, uint32_t, FDCAN_RxHeaderTypeDef *, uint8_t *);
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(const FDCAN_HandleTypeDef *, FDCAN_ProtocolStatusTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_GetErrorCounters(FDCAN_HandleTypeDef *, FDCAN_ErrorCountersTypeDef *);
uint32_t HAL_FDCAN_GetError(FDCAN_HandleTypeDef *);
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(const FDCAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxBuffer(FDCAN_HandleTypeDef *, const FDCAN_TxHeaderTypeDef *, const uint8_t *, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_EnableTxBufferRequest(FDCAN_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_AbortTxRequest(FDCAN_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_GetTxEvent(FDCAN_HandleTypeDef *, FDCAN_TxEventFifoTypeDef *);
#endif
