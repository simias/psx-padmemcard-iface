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

enum padmem_slot {
    SLOT_1 = 1,
    SLOT_2 = 2,
};

static void spi_init(void)
{
    /* Set MOSI and CLK pins direction to output */
    PORTA.DIR |= PIN4_bm | PIN6_bm;

    /* Pull Up on MISO (warning: real hardware uses 1kohm, in my tests this
     * internal PU is not sufficient)*/
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

enum uart_rx_state {
    WAIT_FOR_0D,
    WAIT_FOR_A5,
    WAIT_FOR_CMD,
};

static volatile enum uart_rx_state uart_st = WAIT_FOR_0D;

ISR_N(USART2_RXC_vect_num)
static void uart_rx_irq(void)
{
    uint8_t b = USART2.RXDATAL;
    enum uart_rx_state nstate = WAIT_FOR_0D;

    switch (uart_st) {
    case WAIT_FOR_0D:
        if (b == 0x0d) {
            nstate = WAIT_FOR_A5;
        }
        break;
    case WAIT_FOR_A5:
        if (b == 0xA5) {
            nstate = WAIT_FOR_CMD;
        } else if (b == 0x0d) {
            nstate = WAIT_FOR_A5;
        } else {
            nstate = WAIT_FOR_0D;
        }
        break;
    case WAIT_FOR_CMD:
        nstate = WAIT_FOR_0D;
    }

    if (nstate != WAIT_FOR_0D) {
        TCA0.SINGLE.CMP0BUF = 0x30;
    }

    uart_st = nstate;
}

static void uart_init(void)
{
    /* Add pull-up on RX pin for when nothing is connected */
    PORTF.PIN1CTRL = PORT_PULLUPEN_bm;

    /* PIN F1 TX*/
    PORTF.DIR |= PIN0_bm;

    USART2.BAUD = USART_BAUD_RATE(BAUD_RATE);
    USART2.CTRLA = USART_RXCIE_bm;
    USART2.CTRLB = USART_TXEN_bm | USART_RXEN_bm;

    stdout = &uart_stream;
}

static void pwm_init(void)
{
    /* LED on PC0: use PWM */
    PORTC.DIR |= PIN0_bm;

    PORTMUX.TCAROUTEA &= ~PORTMUX_TCA0_gm;
    PORTMUX.TCAROUTEA |= PORTMUX_TCA0_PORTC_gc;

    /* Period */
    TCA0.SINGLE.PER = 0xFF;

    /* Duty cycle */
    TCA0.SINGLE.CMP0 = 0x10;

    TCA0.SINGLE.CTRLB = TCA_SINGLE_CMP0EN_bm | TCA_SINGLE_WGMODE_SINGLESLOPE_gc;

    /* Set divider */
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV2_gc | TCA_SINGLE_ENABLE_bm;
}

static void rtc_waitbsy(void)
{
    while (RTC.STATUS & (RTC_CTRLABUSY_bm | RTC_CNTBUSY_bm | RTC_PERBUSY_bm |
                         RTC_CMPBUSY_bm)) {
        ;
    }
}

static volatile uint8_t nticks_10ms = 0;

ISR_N(RTC_CNT_vect_num)
static void rtc_irq(void)
{
    uint8_t pwm_cmp;

    /* ACK */
    RTC.INTFLAGS = RTC_OVF_bm;

    nticks_10ms++;

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

    /* We want a tic every 10ms */
    rtc_waitbsy();
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

static volatile uint8_t ndsr = 0;

ISR_N(PORTD_PORT_vect_num)
static void portd_irq(void)
{
    uint8_t flags = PORTD.INTFLAGS;

    /* ACK */
    PORTD.INTFLAGS = flags;

    if (flags & PORT_INT_0_bm) {
        TCA0.SINGLE.CMP0BUFL = 0xff;
        TCA0.SINGLE.CMP0BUFH = 0;
        ndsr++;
    }
}

/* Returns true if DSR is active (that is, /DSR is at 0) */
static bool dsr(void)
{
    return !(PORTD.IN & PIN0_bm);
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

    /* /DSR: PD0, IRQ on falling edge, pull up (warning: real hardware uses
     * 1kohm, in my tests this internal PU is not sufficient) */
    PORTD.PIN0CTRL = PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc;

    /* /SEL1: PD1, /SEL2: PD2 */
    PORTD.DIR |= PIN1_bm | PIN2_bm;
    PORTD.OUTSET |= PIN1_bm | PIN2_bm;

    /* Make sure the peripherals see stable values before we attempt to start a
     * command */
    _delay_ms(200);

    sei();
}

static void run_command(enum padmem_slot slot, uint8_t *cmd, uint8_t len)
{
    /* Select port */
    if (slot == SLOT_1) {
        PORTD.OUTCLR = PIN1_bm;
    } else {
        PORTD.OUTCLR = PIN2_bm;
    }
    _delay_us(30);

    /* Send the command, waiting for /DSR between each byte */
    {
        unsigned i;
        uint8_t ndsr_pre = ndsr;

        for (i = 0; i < len; i++) {
            uint8_t tx = cmd[i];

            if (i > 0) {
                // Wait for DSR
                uint8_t timeout = nticks_10ms + 2;

                while (nticks_10ms != timeout && ndsr != ndsr_pre) {
                    ;
                }

                if (ndsr == ndsr_pre) {
                    printf("%d: Didn't get a DSR!\n", slot);
                    goto done;
                }
            }

            /* Make sure DSR is inactive before we start */
            while (dsr()) {
                ;
            }

            _delay_us(4);

            ndsr_pre = ndsr;

            uint8_t rx = spi_exchange(tx);

            printf("%d: Sent %x got %x\n", slot, tx, rx);
        }
    }
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);

done:
    /* Deselect everything */
    PORTD.OUTSET |= PIN1_bm | PIN2_bm;
    _delay_us(20);
}

int main(void)
{
    init();

    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_enable();

    for (;;) {
        cli();
        if (uart_st != WAIT_FOR_CMD) {
            /* This may look race-y but it isn't: the instruction immediately
             * after SEI is guaranteed to be executed before any IRQ is handled.
             * So it's not possible for an IRQ to sneak in-between.
             */
            sei();
            sleep_cpu();
            sleep_disable();
            continue;
        }
        sei();

        {
            uint8_t pad_read[] = {
                0x01, 0x42, 0x00, 0x00, 0x00,
            };

            puts("Handle CMD!");

            run_command(SLOT_1, pad_read, ARRAY_SIZE(pad_read));
            run_command(SLOT_2, pad_read, ARRAY_SIZE(pad_read));
        }
    }

    return 0;
}
