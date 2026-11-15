// SPDX-License-Identifier: GPL-2.0-only
/*
 * Sipeed NanoKVM-Go expansion lines
 *
 * The auxiliary USB-C port carries two SoC pads, UART3's TXD (line 0) and
 * RXD (line 1). Each can be a UART line, a PWM output or a GPIO, and the two
 * can be a bit-banged I2C bus. Userspace picks one of these modes through
 * the "mode" attribute; until then both lines are inputs pulled down, so
 * nothing on the port is driven.
 *
 * What a mode does not hand to userspace stays requested here: GPIO lines as
 * inputs, PWM channels disabled. Nothing can then drive a line the mode does
 * not drive, and a mode is only left once userspace has let go of what it
 * was given.
 */

#include <linux/bits.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/sysfs.h>

#define EXPANSION_LINES	2

struct expansion_mode {
	const char *name;	/* also the name of the mode's pin state */
	unsigned long gpios;	/* lines userspace may use as GPIOs */
	unsigned long pwms;	/* lines userspace may use as PWM outputs */
	bool i2c;		/* the lines are the i2c-gpio bus */
};

static const struct expansion_mode expansion_modes[] = {
	{ .name = "off" },
	{ .name = "uart" },
	{ .name = "gpio", .gpios = BIT(0) | BIT(1) },
	{ .name = "pwm", .pwms = BIT(0) | BIT(1) },
	{ .name = "pwm-gpio", .pwms = BIT(0), .gpios = BIT(1) },
	{ .name = "gpio-pwm", .gpios = BIT(0), .pwms = BIT(1) },
	{ .name = "i2c", .i2c = true },
};

#define EXPANSION_OFF	(&expansion_modes[0])

/* The PWM core keeps these as the channels' labels. */
static const char * const expansion_pwm_names[EXPANSION_LINES] = {
	"line0", "line1",
};

struct expansion {
	struct device *dev;
	struct mutex lock;	/* serializes mode changes */
	struct pinctrl *pinctrl;
	struct pinctrl_state *states[ARRAY_SIZE(expansion_modes)];
	const struct expansion_mode *mode;
	struct gpio_desc *gpios[EXPANSION_LINES];
	struct pwm_device *pwms[EXPANSION_LINES];
	struct device_node *i2c_node;
	struct platform_device *i2c;
};

/* Requests the GPIOs and PWM channels @mode keeps from userspace. */
static int expansion_hold(struct expansion *ex,
			  const struct expansion_mode *mode)
{
	unsigned int line;

	for (line = 0; line < EXPANSION_LINES; line++) {
		if (!(mode->gpios & BIT(line)) && !ex->gpios[line]) {
			struct gpio_desc *gpio;

			gpio = gpiod_get_index(ex->dev, "line", line, GPIOD_IN);
			if (IS_ERR(gpio))
				return PTR_ERR(gpio);
			ex->gpios[line] = gpio;
		}

		if (!(mode->pwms & BIT(line)) && !ex->pwms[line]) {
			struct pwm_device *pwm;

			pwm = pwm_get(ex->dev, expansion_pwm_names[line]);
			if (IS_ERR(pwm))
				return PTR_ERR(pwm);
			ex->pwms[line] = pwm;
			pwm_disable(pwm);
		}
	}

	return 0;
}

/* Frees the GPIOs and PWM channels @mode hands to userspace. */
static void expansion_release(struct expansion *ex,
			      const struct expansion_mode *mode)
{
	unsigned int line;

	for (line = 0; line < EXPANSION_LINES; line++) {
		if ((mode->gpios & BIT(line)) && ex->gpios[line]) {
			gpiod_put(ex->gpios[line]);
			ex->gpios[line] = NULL;
		}

		if ((mode->pwms & BIT(line)) && ex->pwms[line]) {
			pwm_put(ex->pwms[line]);
			ex->pwms[line] = NULL;
		}
	}
}

/* Hands the GPIOs, held until now, to a new i2c-gpio bus. */
static int expansion_add_i2c(struct expansion *ex)
{
	unsigned int line;

	for (line = 0; line < EXPANSION_LINES; line++) {
		if (ex->gpios[line])
			gpiod_put(ex->gpios[line]);
		ex->gpios[line] = NULL;
	}

	ex->i2c = of_platform_device_create(ex->i2c_node, NULL, ex->dev);

	return ex->i2c ? 0 : -ENODEV;
}

static void expansion_remove_i2c(struct expansion *ex)
{
	if (!ex->i2c)
		return;

	of_platform_device_destroy(&ex->i2c->dev, NULL);
	ex->i2c = NULL;
}

static int expansion_set_mode(struct expansion *ex,
			      const struct expansion_mode *mode)
{
	const struct expansion_mode *old = ex->mode;
	int ret;

	expansion_remove_i2c(ex);

	ret = expansion_hold(ex, mode);
	if (ret)
		goto undo;

	ret = pinctrl_select_state(ex->pinctrl,
				   ex->states[mode - expansion_modes]);
	if (ret)
		goto undo;

	expansion_release(ex, mode);
	ex->mode = mode;

	if (mode->i2c) {
		ret = expansion_add_i2c(ex);
		if (ret) {
			/* Back to a mode that drives nothing. */
			if (!expansion_hold(ex, EXPANSION_OFF) &&
			    !pinctrl_select_state(ex->pinctrl, ex->states[0]))
				ex->mode = EXPANSION_OFF;
			return ret;
		}
	}

	return 0;

undo:
	if (old) {
		expansion_release(ex, old);
		if (old->i2c && expansion_add_i2c(ex))
			dev_err(ex->dev, "Failed to restore the I2C bus\n");
	}

	return ret;
}

static ssize_t mode_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct expansion *ex = dev_get_drvdata(dev);
	const char *name;

	mutex_lock(&ex->lock);
	name = ex->mode->name;
	mutex_unlock(&ex->lock);

	return sysfs_emit(buf, "%s\n", name);
}

static ssize_t mode_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct expansion *ex = dev_get_drvdata(dev);
	unsigned int i;
	int ret = 0;

	for (i = 0; i < ARRAY_SIZE(expansion_modes); i++)
		if (ex->states[i] && sysfs_streq(buf, expansion_modes[i].name))
			break;

	if (i == ARRAY_SIZE(expansion_modes))
		return -EINVAL;

	mutex_lock(&ex->lock);
	if (ex->mode != &expansion_modes[i])
		ret = expansion_set_mode(ex, &expansion_modes[i]);
	mutex_unlock(&ex->lock);

	return ret ?: count;
}
static DEVICE_ATTR_RW(mode);

static ssize_t modes_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct expansion *ex = dev_get_drvdata(dev);
	unsigned int i;
	int len = 0;

	for (i = 0; i < ARRAY_SIZE(expansion_modes); i++)
		if (ex->states[i])
			len += sysfs_emit_at(buf, len, "%s%s", len ? " " : "",
					     expansion_modes[i].name);

	return len + sysfs_emit_at(buf, len, "\n");
}
static DEVICE_ATTR_RO(modes);

static struct attribute *expansion_attrs[] = {
	&dev_attr_mode.attr,
	&dev_attr_modes.attr,
	NULL
};
ATTRIBUTE_GROUPS(expansion);

static void expansion_teardown(void *data)
{
	struct expansion *ex = data;
	unsigned int line;

	expansion_remove_i2c(ex);
	if (ex->states[0])
		pinctrl_select_state(ex->pinctrl, ex->states[0]);

	for (line = 0; line < EXPANSION_LINES; line++) {
		if (ex->gpios[line])
			gpiod_put(ex->gpios[line]);
		if (ex->pwms[line])
			pwm_put(ex->pwms[line]);
	}

	of_node_put(ex->i2c_node);
}

static int expansion_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct expansion *ex;
	unsigned int i;
	int ret;

	ex = devm_kzalloc(dev, sizeof(*ex), GFP_KERNEL);
	if (!ex)
		return -ENOMEM;

	ex->dev = dev;
	platform_set_drvdata(pdev, ex);

	ret = devm_mutex_init(dev, &ex->lock);
	if (ret)
		return ret;

	ex->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(ex->pinctrl))
		return dev_err_probe(dev, PTR_ERR(ex->pinctrl),
				     "Failed to get the pin states\n");

	ex->i2c_node = of_get_child_by_name(dev->of_node, "i2c");

	for (i = 0; i < ARRAY_SIZE(expansion_modes); i++) {
		struct pinctrl_state *state;

		if (expansion_modes[i].i2c && !ex->i2c_node)
			continue;

		state = pinctrl_lookup_state(ex->pinctrl, expansion_modes[i].name);
		if (!IS_ERR(state))
			ex->states[i] = state;
	}

	ret = devm_add_action_or_reset(dev, expansion_teardown, ex);
	if (ret)
		return ret;

	if (!ex->states[0])
		return dev_err_probe(dev, -EINVAL, "No \"off\" pin state\n");

	ret = expansion_set_mode(ex, EXPANSION_OFF);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to turn the lines off\n");

	return 0;
}

static const struct of_device_id expansion_of_match[] = {
	{ .compatible = "sipeed,nanokvm-go-expansion" },
	{ }
};
MODULE_DEVICE_TABLE(of, expansion_of_match);

static struct platform_driver expansion_driver = {
	.probe = expansion_probe,
	.driver = {
		.name = "nanokvm-go-expansion",
		.of_match_table = expansion_of_match,
		.dev_groups = expansion_groups,
	},
};
module_platform_driver(expansion_driver);

MODULE_DESCRIPTION("Sipeed NanoKVM-Go expansion lines");
MODULE_LICENSE("GPL");
