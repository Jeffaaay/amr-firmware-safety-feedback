/* Production parser -> stored request -> real fault predicates -> GPIO pins.
 * Only hardware and unrelated peripheral diagnostic commands are stubbed. */
#include "fake_bus.h"
#include "battery.h"
#include "can_trace.h"
#include "canopen.h"
#include "drive.h"
#include "imu.h"
#include "jetson_protocol.h"
#include "jetson_tx.h"
#include "lift.h"
#include "safety_io.h"
#include "status_light.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

GPIO_TypeDef host_gpio_e, host_gpio_f;
DMA_HandleTypeDef hdma_usart3_rx = {{DMA_CIRCULAR}};
UART_HandleTypeDef huart3 = {USART3, &hdma_usart3_rx, 0};
static unsigned gpio_writes;
static unsigned gpio_target;
static bool gpio_saw_enable;
static unsigned long last_ack_sequence;

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    unsigned channel;
    if (port == LIGHT_RED_GPIO_Port && pin == LIGHT_RED_Pin) channel = 4U;
    else if (port == LIGHT_GREEN_GPIO_Port && pin == LIGHT_GREEN_Pin) channel = 2U;
    else {
        assert(port == LIGHT_BLUE_GPIO_Port && pin == LIGHT_BLUE_Pin);
        channel = 1U;
    }
    gpio_writes++;
    if (state == GPIO_PIN_SET) {
        assert(!gpio_saw_enable); /* all unwanted channels off first */
        assert((gpio_target & channel) == 0U); /* no periodic off pulses */
        port->ODR |= pin;
    } else {
        assert(state == GPIO_PIN_RESET && (gpio_target & channel) != 0U);
        gpio_saw_enable = true;
        port->ODR &= (uint16_t)~pin;
    }
}

HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *uart)
{ assert(uart == &huart3); return HAL_OK; }
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *uart,
                                             uint8_t *buffer, uint16_t length)
{
    assert(uart == &huart3 && buffer != NULL && length == 4096U);
    return HAL_OK;
}
HAL_UART_RxEventTypeTypeDef HAL_UARTEx_GetRxEventType(const UART_HandleTypeDef *uart)
{ (void)uart; return HAL_UART_RXEVENT_IDLE; }
uint32_t HAL_UART_GetError(const UART_HandleTypeDef *uart) { return uart->ErrorCode; }
void JetsonTx_OnTransmitComplete(UART_HandleTypeDef *uart) { (void)uart; }
void JetsonTx_OnUartErrorFromISR(UART_HandleTypeDef *uart, uint32_t error)
{ (void)uart; (void)error; }
/* These commands are outside this test; an accidental dispatch must fail. */
void IMU_PrintDebug(void) { assert(false); }
void Battery_PrintDebug(uint32_t nonce) { (void)nonce; assert(false); }
void JetsonTx_PrintDebug(uint32_t nonce) { (void)nonce; assert(false); }
bool CanTrace_RequestStatus(uint32_t nonce) { (void)nonce; assert(false); return false; }
bool CanTrace_RequestDump(uint32_t nonce) { (void)nonce; assert(false); return false; }

static void ExpectPins(unsigned rgb)
{
    assert(((LIGHT_RED_GPIO_Port->ODR & LIGHT_RED_Pin) == 0U) == ((rgb & 4U) != 0U));
    assert(((LIGHT_GREEN_GPIO_Port->ODR & LIGHT_GREEN_Pin) == 0U) == ((rgb & 2U) != 0U));
    assert(((LIGHT_BLUE_GPIO_Port->ODR & LIGHT_BLUE_Pin) == 0U) == ((rgb & 1U) != 0U));
}

static void PrepareGpio(unsigned rgb)
{ gpio_target = rgb; gpio_saw_enable = false; gpio_writes = 0U; }

static void Update(unsigned rgb)
{
    PrepareGpio(rgb);
    StatusLight_Update();
    assert(gpio_writes == 3U);
    ExpectPins(rgb);
}

static void Reset(void)
{
    Fake_Reset();
    Drive_TestResetState();
    Lift_TestResetState();
    tick_step = 0U;
    JetsonProtocol_TestResetRxState();
    PrepareGpio(2U);
    StatusLight_Init();
    ExpectPins(2U);
    assert(!Drive_IsFaulted() && !Lift_IsFaulted());
}

static void ExpectAck(const char *command, bool accepted, const char *reason,
                      unsigned long nonce)
{
    char actual_command[32], actual_reason[32];
    unsigned actual_accepted;
    unsigned long sequence, actual_nonce;
    int consumed = 0;
    const char *ack = strstr(Fake_Log(), "CMD_ACK ");
    if (ack == NULL) {
        fprintf(stderr, "Expected %s ACK, received: [%s]\n", command, Fake_Log());
    }
    assert(ack != NULL);
    if (strcmp(command, "LIGHT") == 0) assert(ack == Fake_Log());
    assert(sscanf(ack,
                  "CMD_ACK %31s seq=%lu accepted=%u reason=%31s req=%8lX\r\n%n",
                  actual_command, &sequence, &actual_accepted, actual_reason,
                  &actual_nonce, &consumed) == 5);
    assert(ack[consumed] == '\0');
    assert(strcmp(actual_command, command) == 0);
    assert(actual_accepted == (accepted ? 1U : 0U));
    assert(strcmp(actual_reason, reason) == 0 && actual_nonce == nonce);
    assert(sequence == last_ack_sequence + 1UL);
    last_ack_sequence = sequence;
}

static void Command(const char *line, bool accepted, const char *reason,
                    unsigned long nonce)
{
    unsigned writes_before = gpio_writes;
    uint32_t sdo_before = sdo_ops;
    Fake_ClearLog();
    JetsonProtocol_Execute(line);
    ExpectAck("LIGHT", accepted, reason, nonce);
    assert(gpio_writes == writes_before); /* parser only stores state */
    assert(sdo_ops == sdo_before); /* lighting never sends motor/lift SDOs */
}

static void Request(unsigned rgb)
{
    char line[32];
    snprintf(line, sizeof(line), "LIGHT RGB %u %u %u",
             (rgb >> 2) & 1U, (rgb >> 1) & 1U, rgb & 1U);
    Command(line, true, "OK", 0UL);
}

static void SetFaults(unsigned faults)
{
    /* Drive test hook selects the real state predicate. Lift receives a real
     * nonzero node-3 EMCY through production CANopen + Lift_Update. */
    Drive_TestSetRuntimeReady((faults & 1U) ? DRIVE_STATE_FAULT : DRIVE_STATE_DISABLED,
                              0, 0, tick);
    Lift_TestResetState();
    if ((faults & 2U) != 0U) {
        const uint8_t emcy[8] = {0x10, 0x23, 0, 0, 0, 0, 0, 0};
        Fake_Rx(0x083U, emcy, sizeof(emcy));
        Canopen_ProcessRx(tick, 128U);
        Lift_Update();
    }
    assert(Drive_IsFaulted() == ((faults & 1U) != 0U));
    assert(Lift_IsFaulted() == ((faults & 2U) != 0U));
}

static void TestColorsAndFaults(void)
{
    Reset();
    Update(2U); /* no command -> green */
    SetFaults(1U); Update(4U);
    SetFaults(0U); Update(2U); /* no request to restore */
    for (unsigned rgb = 0U; rgb < 8U; rgb++) {
        Reset(); Request(rgb);
        for (unsigned i = 0; i < 20U; i++) Update(rgb);
        for (unsigned faults = 1U; faults < 4U; faults++) {
            SetFaults(faults);
            for (unsigned i = 0; i < 20U; i++) Update(4U);
            SetFaults(0U); Update(rgb);
        }
    }
    Reset();
    SetFaults(3U); Update(4U);
    Request(1U); Update(4U); /* first-ever request during fault */
    Request(7U); Update(4U); /* latest request wins after recovery */
    SetFaults(2U); Update(4U); /* clearing one fault is insufficient */
    SetFaults(0U); Update(7U);
    PrepareGpio(2U); StatusLight_Init(); Update(2U); /* boot resets request */
    puts("PASS: boot GREEN; all 8 colors; Drive/Lift/both force solid RED; restore latest request including OFF; active-low pins and off-before-on ordering");
}

static void TestInvalidCommands(void)
{
    static const char *invalid[] = {
        "LIGHT", "LIGHT RGB", "LIGHT RGB 1", "LIGHT RGB 1 0",
        "LIGHT RGB 2 0 0", "LIGHT RGB -1 0 1", "LIGHT RGB abc 0 1",
        "LIGHT RGB 0 2 0", "LIGHT RGB 0 0 2", "LIGHT RGB 1 0 -1",
        "LIGHT RGB 1 0 2147483648", "LIGHT RGB 1 0 1 0",
        "LIGHT RGB 1 0 1x", "LIGHT RGB 1.0 0 1", "LIGHT RGB +1 0 1",
        "LIGHT RGB 01 0 1", "LIGHT RGB -0 0 1", "LIGHT RGB 1 0 1 ",
        " LIGHT RGB 1 0 1", "LIGHT RGB 1\t0 1", "LIGHT RGBX 1 0 1",
        "LIGHT rgb 1 0 1", "LIGHT RGB 1 0 1 garbage",
        "LIGHT RGB 1 0 1 req=12345678 extra", "LIGHT RGB 1 0 1 req=",
        "LIGHT RGB 1 0 1 req=1", "LIGHT RGB 1 0 1 req=00000000",
        "LIGHT RGB 1 0 1 req=123456789", "LIGHT RGB 1 0 1 req=ZZZZZZZZ",
        "LIGHT RGB 1 0 1 req=12345678 req=87654321"
    };
    Reset();
    Command("LIGHT RGB 2 0 0", false, "BAD_COMMAND", 0UL);
    Update(2U);
    for (unsigned rgb = 0U; rgb < 8U; rgb++) {
        Request(rgb);
        for (unsigned fault = 0U; fault < 2U; fault++) {
            SetFaults(fault); Update(fault ? 4U : rgb);
            for (unsigned i = 0U; i < sizeof(invalid) / sizeof(invalid[0]); i++) {
                Command(invalid[i], false,
                        strstr(invalid[i], " req=") ? "BAD_REQ" : "BAD_COMMAND", 0UL);
                Update(fault ? 4U : rgb);
            }
        }
        SetFaults(0U); Update(rgb);
    }
    Command("LIGHT RGB 2 0 0 req=12345678", false, "BAD_COMMAND", 0x12345678UL);
    Update(7U);
    Command("LIGHT  RGB  0 1  1   req=aBcD1234", true, "OK", 0xABCD1234UL);
    Update(3U);
    puts("PASS: malformed channels, missing/extra tokens, nonce errors leave every saved color intact; normal CMD_ACK sequence/reasons/nonce; valid command after errors");
}

static void Wire(const char *text)
{
    while (*text) JetsonProtocol_TestInjectRxByte((uint8_t)*text++);
    JetsonProtocol_PollLine();
}

static void TestUartAndExistingCommands(void)
{
    Reset();
    Fake_ClearLog();
    Wire("LIGHT RGB 0 ");
    assert(Fake_Log()[0] == '\0'); Update(2U);
    Wire("0 1\r\n");
    ExpectAck("LIGHT", true, "OK", 0UL); Update(1U);
    Fake_ClearLog();
    Wire("LIGHT RGB abc 0 1\r\n");
    ExpectAck("LIGHT", false, "BAD_COMMAND", 0UL); Update(1U);
    Fake_ClearLog();
    Wire("LIGHT RGB 0 0 0 req=12345678\n");
    ExpectAck("LIGHT", true, "OK", 0x12345678UL); Update(0U);
    Fake_ClearLog(); Wire("FW_VERSION\r\n");
    assert(strstr(Fake_Log(), "FW variant=") != NULL);
    Fake_ClearLog(); Wire("ENABLE\r\n");
    ExpectAck("ENABLE", false, "BAD_REQ", 0UL);
    Fake_ClearLog(); Wire("CMD_VEL abc 0\r\n");
    ExpectAck("CMD_VEL", false, "BAD_COMMAND", 0UL);
    Fake_ClearLog(); Wire("STOP\r\n");
    ExpectAck("STOP", true, "OK", 0UL);
    Update(0U);

    /* Exercise the existing circular DMA drain -> line parser as well. */
    JetsonProtocol_TestResetRxState();
    JetsonProtocol_StartUartRxInterrupts();
    const char line[] = "LIGHT RGB 1 0 1\r\n";
    JetsonProtocol_TestWriteRxDma(0U, (const uint8_t *)line, sizeof(line) - 1U);
    JetsonProtocol_TestInjectRxDmaEvent(sizeof(line) - 1U, HAL_UART_RXEVENT_IDLE,
                                      JetsonProtocol_TestGetRxDmaEpoch());
    Fake_ClearLog();
    JetsonProtocol_ServiceRx(); JetsonProtocol_PollLine();
    ExpectAck("LIGHT", true, "OK", 0UL); Update(5U);
    puts("PASS: fragmented CRLF/LF UART framing, invalid-then-valid recovery, DMA drain, FW_VERSION/ENABLE/CMD_VEL/STOP dispatch");
}

int main(void)
{
    setbuf(stdout, NULL);
    TestColorsAndFaults();
    TestInvalidCommands();
    TestUartAndExistingCommands();
    puts("PASS: production-C status light tests (hardware NOT RUN)");
    return 0;
}
