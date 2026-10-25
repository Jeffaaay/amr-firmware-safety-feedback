#include "safety_io.h"

#include "main.h"

void SafetyIo_Init(void)
{
    SafetyIo_SetRgb(false, true, false);
}

void SafetyIo_SetRgb(bool red, bool green, bool blue)
{
    /* Turn unwanted channels OFF before enabling requested channels. Do not
     * pulse an active channel off on every main-loop update. */
    if (!red) {
        HAL_GPIO_WritePin(LIGHT_RED_GPIO_Port, LIGHT_RED_Pin, GPIO_PIN_SET);
    }
    if (!green) {
        HAL_GPIO_WritePin(LIGHT_GREEN_GPIO_Port, LIGHT_GREEN_Pin, GPIO_PIN_SET);
    }
    if (!blue) {
        HAL_GPIO_WritePin(LIGHT_BLUE_GPIO_Port, LIGHT_BLUE_Pin, GPIO_PIN_SET);
    }
    if (red) {
        HAL_GPIO_WritePin(LIGHT_RED_GPIO_Port, LIGHT_RED_Pin, GPIO_PIN_RESET);
    }
    if (green) {
        HAL_GPIO_WritePin(LIGHT_GREEN_GPIO_Port, LIGHT_GREEN_Pin, GPIO_PIN_RESET);
    }
    if (blue) {
        HAL_GPIO_WritePin(LIGHT_BLUE_GPIO_Port, LIGHT_BLUE_Pin, GPIO_PIN_RESET);
    }
}
