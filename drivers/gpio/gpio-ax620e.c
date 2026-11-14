// SPDX-License-Identifier: GPL-2.0-only
/*
 * Axera AX620E GPIO controller
 *
 * 32 lines per bank. Every line has its own control register with output
 * value, direction and interrupt configuration; input levels and the
 * interrupt status of the bank are read from shared registers.
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/gpio/driver.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/spinlock.h>

#define AX_GPIO_SECURE_MODE		0x00
#define AX_GPIO_LINE(n)			(0x4 + 4 * (n))
#define  AX_GPIO_DR			BIT(0)
#define  AX_GPIO_DDR			BIT(1)
#define  AX_GPIO_INTEN			BIT(3)
#define  AX_GPIO_INTMASK		BIT(4)
#define  AX_GPIO_INT_EDGE		BIT(5)
#define  AX_GPIO_INT_HIGH		BIT(6)
#define  AX_GPIO_DEBOUNCE		BIT(7)
#define  AX_GPIO_EOI			BIT(8)
#define  AX_GPIO_INT_BOTH_EDGES		BIT(9)
#define  AX_GPIO_INT_CFG		(AX_GPIO_INT_EDGE | AX_GPIO_INT_HIGH | \
					 AX_GPIO_INT_BOTH_EDGES)
#define AX_GPIO_EXT_PORT		0x8c
#define AX_GPIO_INT_STATUS		0xa4

#define AX_GPIO_NGPIO			32

struct ax_gpio {
	struct gpio_chip gc;
	void __iomem *base;
	/* Serializes read-modify-write of the line registers. */
	raw_spinlock_t lock;
};

static void ax_gpio_update(struct ax_gpio *gpio, unsigned int line, u32 clear,
			   u32 set)
{
	void __iomem *reg = gpio->base + AX_GPIO_LINE(line);
	unsigned long flags;

	raw_spin_lock_irqsave(&gpio->lock, flags);
	writel((readl(reg) & ~clear) | set, reg);
	raw_spin_unlock_irqrestore(&gpio->lock, flags);
}

static int ax_gpio_get_direction(struct gpio_chip *gc, unsigned int line)
{
	struct ax_gpio *gpio = gpiochip_get_data(gc);

	if (readl(gpio->base + AX_GPIO_LINE(line)) & AX_GPIO_DDR)
		return GPIO_LINE_DIRECTION_OUT;

	return GPIO_LINE_DIRECTION_IN;
}

static int ax_gpio_direction_input(struct gpio_chip *gc, unsigned int line)
{
	ax_gpio_update(gpiochip_get_data(gc), line, AX_GPIO_DDR, 0);

	return 0;
}

static int ax_gpio_direction_output(struct gpio_chip *gc, unsigned int line,
				    int value)
{
	ax_gpio_update(gpiochip_get_data(gc), line, AX_GPIO_DR,
		       AX_GPIO_DDR | (value ? AX_GPIO_DR : 0));

	return 0;
}

static int ax_gpio_get(struct gpio_chip *gc, unsigned int line)
{
	struct ax_gpio *gpio = gpiochip_get_data(gc);
	u32 val = readl(gpio->base + AX_GPIO_LINE(line));

	if (val & AX_GPIO_DDR)
		return !!(val & AX_GPIO_DR);

	return !!(readl(gpio->base + AX_GPIO_EXT_PORT) & BIT(line));
}

static int ax_gpio_set(struct gpio_chip *gc, unsigned int line, int value)
{
	ax_gpio_update(gpiochip_get_data(gc), line, AX_GPIO_DR,
		       value ? AX_GPIO_DR : 0);

	return 0;
}

static int ax_gpio_set_config(struct gpio_chip *gc, unsigned int line,
			      unsigned long config)
{
	if (pinconf_to_config_param(config) != PIN_CONFIG_INPUT_DEBOUNCE)
		return -ENOTSUPP;

	ax_gpio_update(gpiochip_get_data(gc), line, AX_GPIO_DEBOUNCE,
		       pinconf_to_config_argument(config) ? AX_GPIO_DEBOUNCE : 0);

	return 0;
}

static void ax_gpio_irq_ack(struct irq_data *d)
{
	struct ax_gpio *gpio = gpiochip_get_data(irq_data_get_irq_chip_data(d));
	irq_hw_number_t line = irqd_to_hwirq(d);

	ax_gpio_update(gpio, line, 0, AX_GPIO_EOI);
	ax_gpio_update(gpio, line, AX_GPIO_EOI, 0);
}

static void ax_gpio_irq_mask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	irq_hw_number_t line = irqd_to_hwirq(d);

	ax_gpio_update(gpiochip_get_data(gc), line, 0, AX_GPIO_INTMASK);
	gpiochip_disable_irq(gc, line);
}

static void ax_gpio_irq_unmask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	irq_hw_number_t line = irqd_to_hwirq(d);

	gpiochip_enable_irq(gc, line);
	ax_gpio_update(gpiochip_get_data(gc), line, AX_GPIO_INTMASK,
		       AX_GPIO_INTEN);
}

static int ax_gpio_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct ax_gpio *gpio = gpiochip_get_data(irq_data_get_irq_chip_data(d));
	irq_hw_number_t line = irqd_to_hwirq(d);
	u32 cfg;

	switch (type & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_EDGE_RISING:
		cfg = AX_GPIO_INT_EDGE | AX_GPIO_INT_HIGH;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		cfg = AX_GPIO_INT_EDGE;
		break;
	case IRQ_TYPE_EDGE_BOTH:
		cfg = AX_GPIO_INT_EDGE | AX_GPIO_INT_BOTH_EDGES;
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		cfg = AX_GPIO_INT_HIGH;
		break;
	case IRQ_TYPE_LEVEL_LOW:
		cfg = 0;
		break;
	default:
		return -EINVAL;
	}

	ax_gpio_update(gpio, line, AX_GPIO_INT_CFG, cfg);
	if (type & IRQ_TYPE_EDGE_BOTH)
		irq_set_handler_locked(d, handle_edge_irq);
	else
		irq_set_handler_locked(d, handle_level_irq);

	return 0;
}

static const struct irq_chip ax_gpio_irq_chip = {
	.name = "ax620e-gpio",
	.irq_ack = ax_gpio_irq_ack,
	.irq_mask = ax_gpio_irq_mask,
	.irq_unmask = ax_gpio_irq_unmask,
	.irq_set_type = ax_gpio_irq_set_type,
	.flags = IRQCHIP_IMMUTABLE | IRQCHIP_MASK_ON_SUSPEND,
	GPIOCHIP_IRQ_RESOURCE_HELPERS,
};

static void ax_gpio_irq_handler(struct irq_desc *desc)
{
	struct gpio_chip *gc = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct ax_gpio *gpio = gpiochip_get_data(gc);
	unsigned long status;
	unsigned int line;

	chained_irq_enter(chip, desc);
	status = readl(gpio->base + AX_GPIO_INT_STATUS);
	for_each_set_bit(line, &status, AX_GPIO_NGPIO)
		generic_handle_domain_irq(gc->irq.domain, line);
	chained_irq_exit(chip, desc);
}

static int ax_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct clk_bulk_data *clks;
	struct reset_control *rst;
	struct gpio_irq_chip *girq;
	struct ax_gpio *gpio;
	unsigned int line;
	int irq, ret;

	gpio = devm_kzalloc(dev, sizeof(*gpio), GFP_KERNEL);
	if (!gpio)
		return -ENOMEM;
	raw_spin_lock_init(&gpio->lock);

	gpio->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(gpio->base))
		return PTR_ERR(gpio->base);

	ret = devm_clk_bulk_get_all_enabled(dev, &clks);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to enable clocks\n");

	rst = devm_reset_control_get_optional_exclusive_deasserted(dev, NULL);
	if (IS_ERR(rst))
		return dev_err_probe(dev, PTR_ERR(rst), "failed to get reset\n");

	/* Report every line's interrupt to the non-secure status register. */
	writel(0, gpio->base + AX_GPIO_SECURE_MODE);

	gpio->gc.label = dev_name(dev);
	gpio->gc.parent = dev;
	gpio->gc.owner = THIS_MODULE;
	gpio->gc.base = -1;
	gpio->gc.ngpio = AX_GPIO_NGPIO;
	gpio->gc.get_direction = ax_gpio_get_direction;
	gpio->gc.direction_input = ax_gpio_direction_input;
	gpio->gc.direction_output = ax_gpio_direction_output;
	gpio->gc.get = ax_gpio_get;
	gpio->gc.set = ax_gpio_set;
	gpio->gc.set_config = ax_gpio_set_config;

	irq = platform_get_irq_optional(pdev, 0);
	if (irq > 0) {
		/* Nothing may fire before a consumer asks for it. */
		for (line = 0; line < AX_GPIO_NGPIO; line++)
			ax_gpio_update(gpio, line, AX_GPIO_INTEN,
				       AX_GPIO_INTMASK);

		girq = &gpio->gc.irq;
		gpio_irq_chip_set_chip(girq, &ax_gpio_irq_chip);
		girq->parent_handler = ax_gpio_irq_handler;
		girq->num_parents = 1;
		girq->parents = devm_kcalloc(dev, 1, sizeof(*girq->parents),
					     GFP_KERNEL);
		if (!girq->parents)
			return -ENOMEM;
		girq->parents[0] = irq;
		girq->default_type = IRQ_TYPE_NONE;
		girq->handler = handle_bad_irq;
	}

	return devm_gpiochip_add_data(dev, &gpio->gc, gpio);
}

static const struct of_device_id ax_gpio_of_match[] = {
	{ .compatible = "axera,ax620e-gpio" },
	{ }
};
MODULE_DEVICE_TABLE(of, ax_gpio_of_match);

static struct platform_driver ax_gpio_driver = {
	.probe = ax_gpio_probe,
	.driver = {
		.name = "ax620e-gpio",
		.of_match_table = ax_gpio_of_match,
	},
};
module_platform_driver(ax_gpio_driver);

MODULE_DESCRIPTION("Axera AX620E GPIO driver");
MODULE_LICENSE("GPL");
