#include "app.h"
#include "test_periph.h"

int main(void)
{
    error_status init_result;
    init_result = AppInit();
    if(init_result == ERROR) {
        while(1);
    }
    while(1) {
        PeriphTest();
    }
}
