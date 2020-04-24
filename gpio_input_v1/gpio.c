/**
 * GPIO input test using push button on Gapuino.
 * Before using check your board configuration.
 */

/* PMSIS includes */
#include "pmsis.h"

/* Variables used. */
struct pi_device gpio;

volatile uint8_t done = 0;

volatile uint8_t key_pre = 0, key_det = 0;

void __pi_cb_gpio(void *arg)
{
  uint32_t gpio_pin = (uint32_t)arg;

  key_pre = 1;
  pi_gpio_pin_notif_clear(&gpio, gpio_pin);
}

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

    pi_gpio_e gpio_in = PI_GPIO_A0_PAD_12_A3;
    pi_gpio_notif_e irq_type = PI_GPIO_NOTIF_FALL;
    pi_gpio_flags_e cfg_flags = PI_GPIO_INPUT|PI_GPIO_PULL_DISABLE|PI_GPIO_DRIVE_STRENGTH_LOW;
    pi_gpio_pin_configure(&gpio, gpio_in, cfg_flags);

    pi_task_t cb_gpio = {0};
    pi_task_callback(&cb_gpio, __pi_cb_gpio, (void *)gpio_in);

    rt_gpio_set_event(0, gpio_in, &cb_gpio);
    pi_gpio_pin_notif_configure(&gpio, gpio_in, irq_type);
    pi_gpio_pin_notif_clear(&gpio, gpio_in);

    pi_gpio_pin_read(&gpio, gpio_in, &value);
    printf("GPIO %d in: %d\n", gpio_in & 0xFF, value);

    while (1)
    {
      pi_yield();

      if (key_pre && !key_det)
      {
        pi_time_wait_us(1000);

        pi_gpio_pin_read(&gpio, gpio_in, &value);
        if (value == 0)
        {
          key_det = 1;
        }
      }

      if (key_pre && key_det)
      {
        done++;
        printf("GPIO callback %d\n", done);

        key_pre = 0;
        key_det = 0;
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

