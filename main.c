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

    /* Pull Up on MISO */
    PORTA.PIN5CTRL = PORT_PULLUPEN_bm;

    SPI0.CTRLB = SPI_MODE_3_gc | SPI_SSD_bm;

    /* LSB first, CLK_PER / 16 */
    SPI0.CTRLA = SPI_DORD_bm | SPI_ENABLE_bm | SPI_MASTER_bm |
                 SPI_PRESC_DIV16_gc;
}

static uint8_t spi_wait_bsy(void)
{
    for (;;) {
        uint8_t flg = SPI0.INTFLAGS;

        if (flg & SPI_WRCOL_bm) {
            printf("SPI collision!");
            return 0xff;
        }

        if (flg & SPI_IF_bm) {
            return SPI0.DATA;
        }

        _delay_ms(500);
    }
}

static uint8_t spi_exchange(uint8_t v)
{
    SPI0.DATA = v;
    return spi_wait_bsy();
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

static void rtc_waitbsy(void)
{
    while (RTC.STATUS & (RTC_CTRLABUSY_bm | RTC_CNTBUSY_bm | RTC_PERBUSY_bm |
                         RTC_CMPBUSY_bm)) {
        ;
    }
}

static volatile uint8_t nticks = 0;

ISR(RTC_CNT_vect)
{
    uint8_t pwm_cmp;

    /* ACK */
    RTC.INTFLAGS = RTC_OVF_bm;

    nticks += 1;

    pwm_cmp = TCA0.SINGLE.CMP0BUFL;

    if (pwm_cmp > 10) {
        pwm_cmp -= 3;

        TCA0.SINGLE.CMP0BUFL = pwm_cmp;
        TCA0.SINGLE.CMP0BUFH = 0;
    }
}

static void rtc_init(void)
{
    const unsigned inclk = 32768;

    rtc_waitbsy();
    /* We want a tic every 10ms */
    RTC.PER = (inclk + (inclk >> 1)) / 100;

    rtc_waitbsy();
    RTC.CLKSEL = RTC_CLKSEL_INT32K_gc;

    /* Enable overflow IRQ */
    rtc_waitbsy();
    RTC.INTCTRL = RTC_OVF_bm;

    /* Enable RTC with no prescaling */
    rtc_waitbsy();
    RTC.CTRLA = RTC_RTCEN_bm | RTC_PRESCALER_DIV1_gc;

    rtc_waitbsy();
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

    rtc_init();

    sei();
}

int main(void)
{
    unsigned i;
    init();

    for (i = 0;; i++) {
        uint8_t v;

        cli();
        TCA0.SINGLE.CMP0BUF = 0x80;
        sei();

        printf("loop %u %u\n", i, nticks);

        v = spi_exchange(0x85);

        printf("send done, got %x\n", v);

        _delay_ms(1000);
        wdt_reset();
    }

    return 0;
}
