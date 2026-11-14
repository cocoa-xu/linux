// SPDX-License-Identifier: GPL-2.0-only
/*
 * Axera AX620E watchdog
 *
 * A down counter compared in units of 65536 ticks. In reset mode the SoC
 * resets when it expires; the vendor firmware instead uses an interrupt
 * mode in which the counter never resets the SoC on its own.
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/watchdog.h>

#define AX_WDT_EN			0x00
#define  AX_WDT_EN_BIT			BIT(0)
#define AX_WDT_TIMEOUT			0x0c
#define AX_WDT_TIMEOUT_LATCH		0x18
#define AX_WDT_COUNT			0x24
#define AX_WDT_RESTART			0x30
#define  AX_WDT_RESTART_KEY		0x61696370
#define AX_WDT_INT_CLR			0x3c
#define AX_WDT_MODE			0x54
#define  AX_WDT_MODE_INTERRUPT		BIT(0)

#define AX_WDT_COUNT_SHIFT		16
#define AX_WDT_DEFAULT_TIMEOUT		30

struct ax_wdt {
	struct watchdog_device wdd;
	void __iomem *base;
	unsigned long rate;
	/* Register writes need a few cycles of the (slow) counter clock. */
	unsigned int settle_us;
};

static struct ax_wdt *to_ax_wdt(struct watchdog_device *wdd)
{
	return container_of(wdd, struct ax_wdt, wdd);
}

static void ax_wdt_strobe(struct ax_wdt *wdt, u32 reg, u32 value)
{
	writel(value, wdt->base + reg);
	udelay(wdt->settle_us);
	writel(0, wdt->base + reg);
}

static int ax_wdt_ping(struct watchdog_device *wdd)
{
	ax_wdt_strobe(to_ax_wdt(wdd), AX_WDT_RESTART, AX_WDT_RESTART_KEY);

	return 0;
}

static void ax_wdt_load(struct ax_wdt *wdt, unsigned int timeout)
{
	u64 ticks = (u64)timeout * wdt->rate;

	writel(min_t(u64, ticks, U32_MAX) >> AX_WDT_COUNT_SHIFT,
	       wdt->base + AX_WDT_TIMEOUT);
	ax_wdt_strobe(wdt, AX_WDT_TIMEOUT_LATCH, 1);
	ax_wdt_ping(&wdt->wdd);
}

static int ax_wdt_set_timeout(struct watchdog_device *wdd,
			      unsigned int timeout)
{
	wdd->timeout = timeout;
	ax_wdt_load(to_ax_wdt(wdd), timeout);

	return 0;
}

static int ax_wdt_start(struct watchdog_device *wdd)
{
	struct ax_wdt *wdt = to_ax_wdt(wdd);

	writel(0, wdt->base + AX_WDT_MODE);
	ax_wdt_load(wdt, wdd->timeout);
	writel(AX_WDT_EN_BIT, wdt->base + AX_WDT_EN);
	ax_wdt_ping(wdd);

	return 0;
}

static int ax_wdt_stop(struct watchdog_device *wdd)
{
	writel(0, to_ax_wdt(wdd)->base + AX_WDT_EN);

	return 0;
}

static unsigned int ax_wdt_get_timeleft(struct watchdog_device *wdd)
{
	struct ax_wdt *wdt = to_ax_wdt(wdd);

	return readl(wdt->base + AX_WDT_COUNT) / wdt->rate;
}

static int ax_wdt_restart(struct watchdog_device *wdd, unsigned long action,
			  void *data)
{
	struct ax_wdt *wdt = to_ax_wdt(wdd);

	writel(0, wdt->base + AX_WDT_MODE);
	writel(0, wdt->base + AX_WDT_TIMEOUT);
	ax_wdt_strobe(wdt, AX_WDT_TIMEOUT_LATCH, 1);
	writel(AX_WDT_EN_BIT, wdt->base + AX_WDT_EN);
	ax_wdt_ping(wdd);
	/* Expires within 65536 counter ticks: 2 s at 32 kHz. */
	mdelay(3000);

	return 0;
}

static const struct watchdog_info ax_wdt_info = {
	.options = WDIOF_KEEPALIVEPING | WDIOF_SETTIMEOUT | WDIOF_MAGICCLOSE,
	.identity = "Axera AX620E watchdog",
};

static const struct watchdog_ops ax_wdt_ops = {
	.owner = THIS_MODULE,
	.start = ax_wdt_start,
	.stop = ax_wdt_stop,
	.ping = ax_wdt_ping,
	.set_timeout = ax_wdt_set_timeout,
	.get_timeleft = ax_wdt_get_timeleft,
	.restart = ax_wdt_restart,
};

static int ax_wdt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct reset_control *rst;
	struct ax_wdt *wdt;
	struct clk *clk;
	int ret;

	wdt = devm_kzalloc(dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt)
		return -ENOMEM;

	wdt->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(wdt->base))
		return PTR_ERR(wdt->base);

	clk = devm_clk_get_enabled(dev, "pclk");
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "no pclk\n");

	clk = devm_clk_get_enabled(dev, "wdt");
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "no counter clock\n");
	wdt->rate = clk_get_rate(clk);
	if (!wdt->rate)
		return dev_err_probe(dev, -EINVAL, "counter clock has no rate\n");
	/* A few counter cycles, as the vendor driver waits. */
	wdt->settle_us = DIV_ROUND_UP(4 * USEC_PER_SEC, wdt->rate);

	rst = devm_reset_control_get_optional_exclusive_deasserted(dev, NULL);
	if (IS_ERR(rst))
		return dev_err_probe(dev, PTR_ERR(rst), "no reset\n");

	wdt->wdd.info = &ax_wdt_info;
	wdt->wdd.ops = &ax_wdt_ops;
	wdt->wdd.parent = dev;
	wdt->wdd.min_timeout = 1;
	wdt->wdd.max_hw_heartbeat_ms = div_u64((u64)U32_MAX * MSEC_PER_SEC,
						 wdt->rate);
	wdt->wdd.timeout = AX_WDT_DEFAULT_TIMEOUT;
	watchdog_init_timeout(&wdt->wdd, 0, dev);
	watchdog_set_restart_priority(&wdt->wdd, 128);
	watchdog_stop_on_unregister(&wdt->wdd);

	/* Left running by the boot firmware or a previous kernel. */
	if (readl(wdt->base + AX_WDT_EN) & AX_WDT_EN_BIT) {
		ax_wdt_start(&wdt->wdd);
		set_bit(WDOG_HW_RUNNING, &wdt->wdd.status);
	}

	ret = devm_watchdog_register_device(dev, &wdt->wdd);
	if (ret)
		return ret;

	dev_info(dev, "%lu Hz, timeout %u s%s\n", wdt->rate, wdt->wdd.timeout,
		 watchdog_hw_running(&wdt->wdd) ? ", running" : "");

	return 0;
}

static const struct of_device_id ax_wdt_of_match[] = {
	{ .compatible = "axera,ax620e-wdt" },
	{ }
};
MODULE_DEVICE_TABLE(of, ax_wdt_of_match);

static struct platform_driver ax_wdt_driver = {
	.probe = ax_wdt_probe,
	.driver = {
		.name = "ax620e-wdt",
		.of_match_table = ax_wdt_of_match,
	},
};
module_platform_driver(ax_wdt_driver);

MODULE_DESCRIPTION("Axera AX620E watchdog driver");
MODULE_LICENSE("GPL");
