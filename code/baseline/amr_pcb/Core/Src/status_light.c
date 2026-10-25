#include "status_light.h"

#include "drive.h"
#include "lift.h"
#include "safety_io.h"

static bool s_request_valid;
static bool s_requested_red;
static bool s_requested_green = true;
static bool s_requested_blue;

void StatusLight_Init(void)
{
    s_request_valid = false;
    s_requested_red = false;
    s_requested_green = true;
    s_requested_blue = false;
    SafetyIo_Init();
}

void StatusLight_RequestRgb(bool red, bool green, bool blue)
{
    s_requested_red = red;
    s_requested_green = green;
    s_requested_blue = blue;
    s_request_valid = true;
}

void StatusLight_Update(void)
{
    if (Drive_IsFaulted() || Lift_IsFaulted()) {
        SafetyIo_SetRgb(true, false, false);
    } else if (s_request_valid) {
        SafetyIo_SetRgb(s_requested_red, s_requested_green, s_requested_blue);
    } else {
        SafetyIo_SetRgb(false, true, false);
    }
}
