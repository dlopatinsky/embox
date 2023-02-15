/**
 * @file
 *
 * @date Feb 11, 2023
 * @author Anton Bondarev
 */

#include <drivers/serial/uart_dev.h>
#include <drivers/serial/diag_serial.h>

#include <drivers/diag.h>

#include <drivers/common/memory.h>

#include <framework/mod/options.h>

#include <framework/mod/options.h>

#define UART_BASE      OPTION_GET(NUMBER, base_addr)
#define IRQ_NUM        OPTION_GET(NUMBER, irq_num)

#define PINS_INIT      OPTION_GET(NUMBER, pins_init)

extern int elvees_uart_setup_common(struct uart *dev, const struct uart_params *params);
extern int elvees_uart_has_symbol(struct uart *dev);
extern int elvees_uart_getc(struct uart *dev);
extern int elvees_uart_putc(struct uart *dev, int ch);

#if PINS_INIT == 0

static int elvees_uart_setup(struct uart *dev, const struct uart_params *params) {
	return 0;
}

#else

#include <drivers/gpio/gpio_driver.h>
static int elvees_uart_setup(struct uart *dev, const struct uart_params *params) {
#define GPIO_ALT_FUNC_UART   (4)

	gpio_setup_mode(3, 1 << 0, GPIO_MODE_OUT_ALTERNATE | GPIO_ALTERNATE(GPIO_ALT_FUNC_UART));
	gpio_setup_mode(3, 1 << 1, GPIO_MODE_OUT_ALTERNATE | GPIO_ALTERNATE(GPIO_ALT_FUNC_UART));

	elvees_uart_setup_common(dev, params);

	return 0;
}
#endif

static const struct uart_ops elvees_uart_uart_ops = {
		.uart_getc = elvees_uart_getc,
		.uart_putc = elvees_uart_putc,
		.uart_hasrx = elvees_uart_has_symbol,
		.uart_setup = elvees_uart_setup,
};

static struct uart uart0 = {
		.uart_ops = &elvees_uart_uart_ops,
		.irq_num = IRQ_NUM,
		.base_addr = UART_BASE,
};

static const struct uart_params uart_diag_params = {
		.baud_rate = OPTION_GET(NUMBER,baud_rate),
		.uart_param_flags = UART_PARAM_FLAGS_8BIT_WORD,
};

DIAG_SERIAL_DEF(&uart0, &uart_diag_params);

PERIPH_MEMORY_DEFINE(elvees_uart, UART_BASE, 0x1000);
