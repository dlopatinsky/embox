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

#define UART_BASE      OPTION_GET(NUMBER, base_addr)
#define IRQ_NUM        OPTION_GET(NUMBER, irq_num)

extern const struct uart_ops elvees_uart_uart_ops;

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
