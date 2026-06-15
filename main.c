#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <stdbool.h>
#include <stdio.h>
#include <util/delay.h>

#define ARRAY_SIZE(_arr) (sizeof(_arr) / sizeof((_arr)[0]))

#define BAUD_RATE 115200UL
#define USART_BAUD_RATE(_br) ((F_CPU * 4 + (_br) / 2) / (_br))

static void spi_init(void)
{
    /* Set MOSI and CLK pins direction to output */
    PORTA.DIR |= PIN4_bm | PIN6_bm;

    /* LSB first, CLK_PER / 16 */
    SPI0.CTRLA = SPI_DORD_bm | SPI_ENABLE_bm | SPI_MASTER_bm |
                 SPI_PRESC_DIV16_gc;

    SPI0.CTRLB = SPI_MODE_3_gc | SPI_SSD_bm;
}

static void spi_wait_bsy(void)
{
    for (;;) {
        if (SPI0.INTFLAGS & SPI_IF_bm) {
            break;
        }
    }
}

static void spi_send_sync(uint8_t v)
{
    SPI0.DATA = v;
    spi_wait_bsy();
}

static int uart_putchar(int c)
{
    while (!(USART2.STATUS & USART_DREIF_bm)) {
        ;
    }

    USART2.TXDATAL = c;

    return c;
}

static int uart_putchar_stream(char c, FILE *stream)
{
    if (c == '\n') {
        uart_putchar('\r');
    }

    return uart_putchar(c);
}

static FILE uart_stream =
    FDEV_SETUP_STREAM(uart_putchar_stream, NULL, _FDEV_SETUP_WRITE);

static void uart_init(void)
{
    USART2.BAUD = USART_BAUD_RATE(BAUD_RATE);
    USART2.CTRLB = USART_TXEN_bm;

    /* PIN F1 TX*/
    PORTF.DIR |= PIN0_bm;

    stdout = &uart_stream;
}

static void pwm_init(void)
{
    /* LED on PC0: use PWM */
    PORTMUX.TCAROUTEA &= ~PORTMUX_TCA0_gm;
    PORTMUX.TCAROUTEA |= PORTMUX_TCA0_PORTC_gc;

    /* Period */
    TCA0.SINGLE.PER = 0xFF;

    /* Duty cycle */
    TCA0.SINGLE.CMP0 = 0x10;

    /* Set divider */
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV2_gc | TCA_SINGLE_ENABLE_bm;

    TCA0.SINGLE.CTRLB = TCA_SINGLE_CMP0EN_bm | TCA_SINGLE_WGMODE_SINGLESLOPE_gc;

    PORTC.DIR |= PIN0_bm;
}

static void init(void)
{
    cli();

    /* Watchdog setup */
    /* wdt_enable(WDTO_2S); */

    /* Per datasheet:
   *
   * The 40-pin version of the ATmega4809 is using the die of the 48-pin
   * ATmega4809 but offers fewer connected pads. For this reason, the pins
   * PB[5:0] and PC[7:6] must be disabled (INPUT_DISABLE) or enable pull-ups
   * (PULLUPEN).
   */
    PORTB.PIN0CTRL &= ~PORT_ISC_gm;
    PORTB.PIN0CTRL |= PORT_ISC_INPUT_DISABLE_gc;
    PORTB.PIN1CTRL &= ~PORT_ISC_gm;
    PORTB.PIN1CTRL |= PORT_ISC_INPUT_DISABLE_gc;
    PORTB.PIN2CTRL &= ~PORT_ISC_gm;
    PORTB.PIN2CTRL |= PORT_ISC_INPUT_DISABLE_gc;
    PORTB.PIN3CTRL &= ~PORT_ISC_gm;
    PORTB.PIN3CTRL |= PORT_ISC_INPUT_DISABLE_gc;
    PORTB.PIN4CTRL &= ~PORT_ISC_gm;
    PORTB.PIN4CTRL |= PORT_ISC_INPUT_DISABLE_gc;
    PORTB.PIN5CTRL &= ~PORT_ISC_gm;
    PORTB.PIN5CTRL |= PORT_ISC_INPUT_DISABLE_gc;
    PORTC.PIN6CTRL &= ~PORT_ISC_gm;
    PORTC.PIN6CTRL |= PORT_ISC_INPUT_DISABLE_gc;
    PORTC.PIN7CTRL &= ~PORT_ISC_gm;
    PORTC.PIN7CTRL |= PORT_ISC_INPUT_DISABLE_gc;

    /* Enable prescaler with PDIV = 0 (DIV 2). With fuse2/osccfg set to 0x01
     * that will give us a 8MHz CPU clock.
     *
     * According to the datasheet, with our 3V5 power supply the chip cannot
     * safely run at 16MHz but we could run at 10MHz if we wanted and stay
     * within the safe range.
     */
    _PROTECTED_WRITE(CLKCTRL_MCLKCTRLB, CLKCTRL_PEN_bm | CLKCTRL_PDIV_4X_gc);

    uart_init();
    puts("Starting up...");

    pwm_init();

    spi_init();
}

int main(void)
{
    unsigned i;
    init();

    for (i = 0;; i++) {
        TCA0.SINGLE.CMP0BUF = 0x80;
        printf("loop %u\n", i);

        spi_send_sync(0x85);

        TCA0.SINGLE.CMP0BUF = 0x10;
        _delay_ms(500);
        wdt_reset();
    }

    return 0;
}
