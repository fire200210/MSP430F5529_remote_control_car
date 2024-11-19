#include "gpio.h"
#include <msp430.h>
#include <stdint.h>

void gpio_init(void)
{
    /* P1.0 GPIO and output low */
    P3SEL &= ~BIT5;
    P3DIR |= BIT5;
    P3OUT &= ~BIT5;

    /* P1.1 GPIO and output low */
    P3SEL &= ~BIT6;
    P3DIR |= BIT6;
    P3OUT &= ~BIT6;

    /* P1.2 GPIO and output low */
    P3SEL &= ~BIT7;
    P3DIR |= BIT7;
    P3OUT &= ~BIT7;

    /* P1.2 GPIO and output low */
    P4SEL &= ~BIT0;
    P4DIR |= BIT0;
    P4OUT &= ~BIT0;
}

void gpio_set_output(gpio_t gpio, bool enable)
{
    uint16_t bit = 0;
    switch (gpio)
    {
    case GPIO_XSHUT_FIRST:
        //bit = BIT0;
        if (enable) {
            P4OUT |= BIT0;
        } else {
            P4OUT &= ~BIT0;
        }
        break;
    case GPIO_XSHUT_SECOND:
        bit = BIT5;
        if (enable) {
            P3OUT |= bit;
        } else {
            P3OUT &= ~bit;
        }
        break;
    case GPIO_XSHUT_THIRD:
        bit = BIT6;
        if (enable) {
            P3OUT |= bit;
        } else {
            P3OUT &= ~bit;
        }
        break;
    case GPIO_XSHUT_FOURTH:
        bit = BIT7;
        if (enable) {
            P3OUT |= bit;
        } else {
            P3OUT &= ~bit;
        }
        break;
    }

}
