// SPDX-License-Identifier: GPL-2.0
/*
 * IKR UART serial driver for the rvemu RISC-V simulator.
 *
 * Register layout (byte-wide MMIO):
 *   offset 0 (IKR_REG_DATA):  R/W - data byte
 *   offset 1 (IKR_REG_RXSTS): R   - bit[0]=1 means RX data available
 *   offset 2 (IKR_REG_TXSTS): R   - 0 means TX ready
 *
 * If the platform device has an IRQ (from DTS "interrupts" property),
 * RX is interrupt-driven: the UART asserts the IRQ line when a byte
 * arrives and the ISR calls ikr_rx_chars() immediately.
 * Without an IRQ the driver falls back to kernel-timer polling.
 */

#define pr_fmt(fmt) "IKR-uart: " fmt

#include <linux/console.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>

#define IKR_UART_DEV_NAME	"ttyIKR"
#define IKR_UART_NR		1
#define DRIVER_NAME		"ikr-uart"

/* Arbitrary non-zero port type ID; PORT_UNKNOWN(0) makes serial_core treat
 * the port as unconfigured and set TTY_IO_ERROR on every open, causing
 * all writes from user space to return -EIO.  Any non-zero ID works. */
#define PORT_IKR_UART		200

/* Register offsets (bytes from MMIO base) */
#define IKR_REG_DATA		0	/* R/W: data byte */
#define IKR_REG_RXSTS		1	/* R:   bit[0]=1 RX byte available */
#define IKR_REG_TXSTS		2	/* R:   0 = TX ready */

#define IKR_RXSTS_AVAIL		BIT(0)

struct ikr_uart {
	struct uart_port port;
	struct task_struct *poll_thread;
};

/* Single static array; only one IKR UART device is expected. */
static struct ikr_uart ikr_ports[IKR_UART_NR];

/* Forward declaration for console */
static struct uart_driver ikr_uart_driver;

/* ---- Register accessors ---- */

static u8 ikr_readb(struct uart_port *port, unsigned int reg)
{
	return readb(port->membase + reg);
}

static void ikr_writeb(struct uart_port *port, u8 val, unsigned int reg)
{
	writeb(val, port->membase + reg);
}

/* ---- uart_ops ---- */

static unsigned int ikr_tx_empty(struct uart_port *port)
{
	return (ikr_readb(port, IKR_REG_TXSTS) == 0) ? TIOCSER_TEMT : 0;
}

static unsigned int ikr_get_mctrl(struct uart_port *port)
{
	return TIOCM_CAR | TIOCM_CTS | TIOCM_DSR;
}

static void ikr_set_mctrl(struct uart_port *port, unsigned int mctrl) {}

static void ikr_stop_tx(struct uart_port *port) {}

static void ikr_rx_chars(struct uart_port *port)
{
	u8 ch;
	int budget = 64;	/* bounded to avoid infinite loop if rvemu's
				 * RXSTS AVAIL bit is stuck asserted */

	while (budget-- && (ikr_readb(port, IKR_REG_RXSTS) & IKR_RXSTS_AVAIL)) {
		ch = ikr_readb(port, IKR_REG_DATA);
		port->icount.rx++;
		if (!uart_handle_sysrq_char(port, ch))
			uart_insert_char(port, 0, 0, ch, TTY_NORMAL);
	}
	tty_flip_buffer_push(&port->state->port);
}

static void ikr_tx_chars(struct uart_port *port)
{
	u8 ch;

	uart_port_tx(port, ch,
		ikr_readb(port, IKR_REG_TXSTS) == 0,
		ikr_writeb(port, ch, IKR_REG_DATA));
}

static void ikr_start_tx(struct uart_port *port)
{
	/*
	 * Drain the transmit buffer immediately so user-space writes don't
	 * sit idle waiting for the next timer tick.  Called with the port
	 * lock held by the serial core.
	 */
	ikr_tx_chars(port);
}

static void ikr_stop_rx(struct uart_port *port) {}

static void ikr_uart_do_poll(struct uart_port *port)
{
	unsigned long flags;

	uart_port_lock_irqsave(port, &flags);
	ikr_rx_chars(port);
	ikr_tx_chars(port);
	uart_port_unlock_irqrestore(port, flags);
}

static int ikr_uart_poll_thread(void *data)
{
	struct ikr_uart *pp = data;

	/* Lowest possible nice value so userspace and other kthreads always
	 * preempt us; we only run when the CPU would otherwise be idle. */
	set_user_nice(current, 19);

	while (!kthread_should_stop()) {
		ikr_uart_do_poll(&pp->port);
		/* Pure busy-poll with explicit yield.  Don't use msleep /
		 * usleep_range / schedule_timeout -- they require timer
		 * interrupts to wake us, which rvemu doesn't deliver. */
		schedule();
	}
	return 0;
}

static int ikr_startup(struct uart_port *port)
{
	struct ikr_uart *pp = container_of(port, struct ikr_uart, port);

	/* rvemu's PLIC delivery for the IKR UART vector is unreliable, so
	 * ignore any IRQ assigned by the DTS and rely on kthread polling. */
	port->irq = 0;

	if (!pp->poll_thread) {
		pp->poll_thread = kthread_run(ikr_uart_poll_thread, pp,
					      "ikr_uart_poll");
		if (IS_ERR(pp->poll_thread)) {
			pr_err("failed to start poll kthread: %ld\n",
			       PTR_ERR(pp->poll_thread));
			pp->poll_thread = NULL;
			return -ENOMEM;
		}
	}
	return 0;
}

static void ikr_shutdown(struct uart_port *port)
{
	/* Don't stop the kthread on shutdown -- the tty layer calls
	 * shutdown between cttyhack respawns and we want polling to stay
	 * alive for the next open. */
}

static void ikr_set_termios(struct uart_port *port,
			     struct ktermios *termios,
			     const struct ktermios *old)
{
	unsigned long flags;

	uart_port_lock_irqsave(port, &flags);
	uart_update_timeout(port, termios->c_cflag, 115200);
	uart_port_unlock_irqrestore(port, flags);
}

static const char *ikr_type(struct uart_port *port)
{
	return DRIVER_NAME;
}

static void ikr_release_port(struct uart_port *port) {}

static int ikr_request_port(struct uart_port *port)
{
	return 0;
}

static void ikr_config_port(struct uart_port *port, int flags)
{
	port->type = PORT_IKR_UART;
}

static int ikr_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	return 0;
}

#ifdef CONFIG_CONSOLE_POLL
static int ikr_poll_get_char(struct uart_port *port)
{
	if (!(ikr_readb(port, IKR_REG_RXSTS) & IKR_RXSTS_AVAIL))
		return NO_POLL_CHAR;
	return ikr_readb(port, IKR_REG_DATA);
}

static void ikr_poll_put_char(struct uart_port *port, unsigned char ch)
{
	while (ikr_readb(port, IKR_REG_TXSTS) != 0)
		cpu_relax();
	ikr_writeb(port, ch, IKR_REG_DATA);
}
#endif /* CONFIG_CONSOLE_POLL */

static const struct uart_ops ikr_uart_ops = {
	.tx_empty	= ikr_tx_empty,
	.set_mctrl	= ikr_set_mctrl,
	.get_mctrl	= ikr_get_mctrl,
	.stop_tx	= ikr_stop_tx,
	.start_tx	= ikr_start_tx,
	.stop_rx	= ikr_stop_rx,
	.startup	= ikr_startup,
	.shutdown	= ikr_shutdown,
	.set_termios	= ikr_set_termios,
	.type		= ikr_type,
	.release_port	= ikr_release_port,
	.request_port	= ikr_request_port,
	.config_port	= ikr_config_port,
	.verify_port	= ikr_verify_port,
#ifdef CONFIG_CONSOLE_POLL
	.poll_get_char	= ikr_poll_get_char,
	.poll_put_char	= ikr_poll_put_char,
#endif
};

/* ---- Console support ---- */

static void ikr_console_putchar(struct uart_port *port, unsigned char ch)
{
	while (ikr_readb(port, IKR_REG_TXSTS) != 0)
		cpu_relax();
	ikr_writeb(port, ch, IKR_REG_DATA);
}

static void ikr_console_write(struct console *co, const char *s,
			       unsigned int count)
{
	struct uart_port *port = &ikr_ports[co->index].port;
	unsigned long flags;
	bool locked;

	/*
	 * The platform device may not have probed yet (membase set in probe).
	 * Drop writes silently until the MMIO mapping is live; once probe runs
	 * membase is non-NULL and all subsequent writes reach the hardware.
	 */
	if (!port->membase)
		return;

	if (oops_in_progress)
		locked = uart_port_trylock_irqsave(port, &flags);
	else {
		uart_port_lock_irqsave(port, &flags);
		locked = true;
	}

	uart_console_write(port, s, count, ikr_console_putchar);

	if (locked)
		uart_port_unlock_irqrestore(port, flags);
}

static int __init ikr_console_setup(struct console *co, char *options)
{
	struct uart_port *port;

	if (co->index < 0 || co->index >= IKR_UART_NR)
		co->index = 0;

	port = &ikr_ports[co->index].port;

	/*
	 * If the platform device hasn't probed yet membase is NULL.  Allow
	 * registration to succeed anyway — ikr_console_write guards every
	 * write with a membase check and silently drops output until the
	 * probe runs and sets membase.  This prevents the kernel from
	 * discarding the console entry (which would cause the
	 * "Warning: unable to open an initial console" at init time because
	 * /dev/console would have no registered tty behind it).
	 */
	if (!port->membase)
		return 0;

	return uart_set_options(port, co, 115200, 'n', 8, 'n');
}

static struct console ikr_console = {
	.name	= IKR_UART_DEV_NAME,
	.write	= ikr_console_write,
	.setup	= ikr_console_setup,
	.device	= uart_console_device,
	.flags	= CON_PRINTBUFFER | CON_ANYTIME,
	.index	= -1,
	.data	= &ikr_uart_driver,
};

/* ---- uart_driver ---- */

static struct uart_driver ikr_uart_driver = {
	.owner		= THIS_MODULE,
	.driver_name	= DRIVER_NAME,
	.dev_name	= IKR_UART_DEV_NAME,
	.major		= 0,	/* dynamic major allocation */
	.minor		= 0,
	.nr		= IKR_UART_NR,
	.cons		= &ikr_console,
};

/* ---- Platform driver ---- */

static int ikr_uart_probe(struct platform_device *pdev)
{
	struct ikr_uart *pp = &ikr_ports[0];
	struct resource *res;
	int ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	/*
	 * The IKR UART MMIO region is at physical address 0x11000, which is
	 * above PAGE_SIZE and not in the kernel's linear map (not RAM).
	 * Standard devm_ioremap_resource() creates a proper kernel VA → PA
	 * mapping that the Sv39 MMU can translate correctly.
	 */
	pp->port.membase = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(pp->port.membase))
		return PTR_ERR(pp->port.membase);

	pp->port.mapbase  = res->start;
	pp->port.iotype   = UPIO_MEM;
	pp->port.irq      = platform_get_irq_optional(pdev, 0);
	if ((int)pp->port.irq < 0)
		pp->port.irq = 0;	/* no IRQ in DTS — use timer polling */
	pp->port.uartclk  = 0;
	pp->port.fifosize = 1;
	pp->port.flags    = UPF_BOOT_AUTOCONF;
	pp->port.ops      = &ikr_uart_ops;
	pp->port.dev      = &pdev->dev;
	pp->port.line     = 0;
	pp->port.type     = PORT_IKR_UART;

	ret = uart_add_one_port(&ikr_uart_driver, &pp->port);
	if (ret)
		return ret;

	/*
	 * ikr_console_setup returns 0 immediately if membase was NULL at
	 * registration time (before this probe ran).  Now that membase is
	 * live, apply the baud/parity/bits settings so the console is fully
	 * configured for user-space (opening /dev/console after initcalls).
	 */
	if (uart_console(&pp->port))
		uart_set_options(&pp->port, ikr_uart_driver.cons,
				 115200, 'n', 8, 'n');

	platform_set_drvdata(pdev, pp);
	return 0;
}

static void ikr_uart_remove(struct platform_device *pdev)
{
	struct ikr_uart *pp = platform_get_drvdata(pdev);

	uart_remove_one_port(&ikr_uart_driver, &pp->port);
}

static const struct of_device_id ikr_uart_of_ids[] = {
	{ .compatible = "ikr,uart" },
	{}
};
MODULE_DEVICE_TABLE(of, ikr_uart_of_ids);

static struct platform_driver ikr_uart_platform_driver = {
	.probe	= ikr_uart_probe,
	.remove	= ikr_uart_remove,
	.driver	= {
		.name		= DRIVER_NAME,
		.of_match_table	= ikr_uart_of_ids,
	},
};

#ifdef CONFIG_SERIAL_IKR_UART_CONSOLE
static int __init ikr_console_init(void)
{
    register_console(&ikr_console);
    return 0;
}
console_initcall(ikr_console_init);
#endif

static int __init ikr_uart_init(void)
{
	int ret;

	ret = uart_register_driver(&ikr_uart_driver);
	if (ret)
		return ret;

	ret = platform_driver_register(&ikr_uart_platform_driver);
	if (ret)
		uart_unregister_driver(&ikr_uart_driver);

	return ret;
}

static void __exit ikr_uart_exit(void)
{
	platform_driver_unregister(&ikr_uart_platform_driver);
	uart_unregister_driver(&ikr_uart_driver);
}

module_init(ikr_uart_init);
module_exit(ikr_uart_exit);

MODULE_DESCRIPTION("IKR UART driver for rvemu RISC-V simulator");
MODULE_LICENSE("GPL v2");
