#include "app.h"
#include "clock.h"

error_status AppInit(void)
{
    error_status init_result = ERROR;
    init_result |= ClockInit();
    return init_result;
}
