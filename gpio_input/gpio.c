/**
 * GPIO input test using push button on Gapuino.
 * Before using check your board configuration.
 */

/* PMSIS includes */
#include "pmsis.h"

/* Variables used. */
struct pi_device gpio;


void test_gpio(void)
{
    int32_t errors = 0;
    uint32_t value = 0;
    struct pi_gpio_conf gpio_conf = {0};
    pi_gpio_conf_init(&gpio_conf);
    pi_open_from_conf(&gpio, &gpio_conf);
    errors = pi_gpio_open(&gpio);
    if (errors)
    {
        printf("Error opening GPIO %d\n", errors);
        pmsis_exit(errors);
    }

    pi_gpio_e gpio_out = PI_GPIO_A3_PAD_15_B1;
    pi_gpio_flags_e cfg_flags = PI_GPIO_OUTPUT | PI_GPIO_PULL_DISABLE | PI_GPIO_DRIVE_STRENGTH_HIGH;
    pi_gpio_pin_configure(&gpio, gpio_out, cfg_flags);

    pi_gpio_e gpio_in = PI_GPIO_A0_PAD_12_A3;
    pi_gpio_notif_e irq_type = PI_GPIO_NOTIF_FALL;
    cfg_flags = PI_GPIO_INPUT|PI_GPIO_PULL_DISABLE|PI_GPIO_DRIVE_STRENGTH_LOW;

    /* Configure gpio input. */
    pi_gpio_pin_configure(&gpio, gpio_in, cfg_flags);
    pi_gpio_pin_read(&gpio, gpio_in, &value);
    printf("GPIO opened, in val: %d\n", value);
    pi_gpio_pin_notif_configure(&gpio, gpio_in, irq_type);

    uint16_t done = 0;
    while (1)
    {
      if (pi_gpio_pin_notif_get(&gpio, gpio_in))
      {
        pi_time_wait_us(20000);
        pi_gpio_pin_read(&gpio, gpio_in, &value);
        if (value == 0)
        {
          done++;
          pi_gpio_pin_read(&gpio, gpio_in, &value);
          printf("GPIO %d in: %d -> %d...\n", gpio_in & 0xFF, value, done);
          pi_gpio_pin_write(&gpio, gpio_out, (done & 0x1));
        }
        pi_gpio_pin_notif_clear(&gpio, gpio_in);
      }
    }

    pmsis_exit(errors);
}

/* Program Entry. */
int main(void)
{
    printf("\n\n\t *** PMSIS GPIO Input ***\n\n");
    return pmsis_kickoff((void *) test_gpio);
}

