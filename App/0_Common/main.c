#include "at32f403a_407.h"
#include "app.h"

// typedef struct GPIO_init_type {
//     uint16_t pins;
//     uint16_t mode;
//     uint16_t out_type;
//     uint16_t pull;
//     uint16_t drive_strength;
// } GPIO_init_type;

gpio_init_type GPIOD_CfgForLed2_str;

void gpio_set_cfg_data(gpio_init_type *gpio_init_struct, uint16_t pins, uint16_t mode, uint16_t out_type, uint16_t pull, uint16_t drive_strength)
{
    gpio_init_struct->gpio_pins  = pins;
    gpio_init_struct->gpio_mode = mode;
    gpio_init_struct->gpio_out_type = out_type;
    gpio_init_struct->gpio_pull = pull;
    gpio_init_struct->gpio_drive_strength = drive_strength;
}

int main(void)
{
    crm_periph_clock_enable(CRM_GPIOD_PERIPH_CLOCK, TRUE);
    gpio_set_cfg_data(&GPIOD_CfgForLed2_str, GPIO_PINS_13, GPIO_MODE_OUTPUT, GPIO_OUTPUT_PUSH_PULL, GPIO_PULL_UP, GPIO_DRIVE_STRENGTH_STRONGER);
    gpio_init(GPIOD, &GPIOD_CfgForLed2_str);

    while(1) {

        gpio_bits_set(GPIOD, GPIO_PINS_13);
        for(int i = 0; i < 600000; i++);
        gpio_bits_reset(GPIOD, GPIO_PINS_13);
        for(int i = 0; i < 600000; i++);
    }
}
