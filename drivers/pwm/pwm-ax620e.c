// SPDX-License-Identifier: GPL-2.0-only
/*
 * Axera AX620E PWM
 *
 * Each block has four channels, DesignWare APB timers in PWM mode that
 * count the timer clock: Load Count 2 is the time the output is high, Load
 * Count the time it is low. With 0N100PWM_EN either may be 0, for duty
 * cycles of 0% and 100%.
 *
 * Limitations:
 * - A new setting stops the timer and starts it again, so the period in
 *   progress is cut short.
 * - After the timer starts, a period begins with its low part.
 * - The level a disabled channel drives is not known; at 0% and 100% the
 *   timer holds its output and stops counting.
 */

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/reset.h>

#define AX620E_PWM_CHANNELS		4

#define AX620E_PWM_LOW(ch)		((ch) * 0x14)
#define AX620E_PWM_CTRL(ch)		((ch) * 0x14 + 0x08)
#define AX620E_PWM_HIGH(ch)		(0xb0 + (ch) * 4)

#define AX620E_PWM_CTRL_EN		BIT(0)
#define AX620E_PWM_CTRL_USER_MODE	BIT(1)
#define AX620E_PWM_CTRL_INT_MASK	BIT(2)
#define AX620E_PWM_CTRL_PWM		BIT(3)
#define AX620E_PWM_CTRL_0N100		BIT(4)
#define AX620E_PWM_CTRL_MODE		(AX620E_PWM_CTRL_USER_MODE | \
					 AX620E_PWM_CTRL_INT_MASK | \
					 AX620E_PWM_CTRL_PWM | \
					 AX620E_PWM_CTRL_0N100)

struct ax620e_pwm {
	void __iomem *base;
	struct clk *clk[AX620E_PWM_CHANNELS];
	bool enabled[AX620E_PWM_CHANNELS];
	unsigned long rate;
};

struct ax620e_pwm_waveform {
	u32 high;
	u32 low;
	bool enabled;
};

static int ax620e_pwm_round_waveform_tohw(struct pwm_chip *chip,
					  struct pwm_device *pwm,
					  const struct pwm_waveform *wf,
					  void *_wfhw)
{
	struct ax620e_pwm *ax = pwmchip_get_drvdata(chip);
	struct ax620e_pwm_waveform *wfhw = _wfhw;
	u64 period, duty;

	if (wf->period_length_ns == 0) {
		*wfhw = (struct ax620e_pwm_waveform){ };
		return 0;
	}

	/* With ax->rate below NSEC_PER_SEC this won't overflow. */
	period = min_t(u64, mul_u64_u32_div(wf->period_length_ns, ax->rate,
					    NSEC_PER_SEC), U32_MAX);
	if (period == 0) {
		/* Shorter than one clock: round up to it, with no high time. */
		*wfhw = (struct ax620e_pwm_waveform){ .low = 1, .enabled = true };
		return 1;
	}

	duty = min(mul_u64_u32_div(wf->duty_length_ns, ax->rate, NSEC_PER_SEC),
		   period);

	*wfhw = (struct ax620e_pwm_waveform){
		.high = duty,
		.low = period - duty,
		.enabled = true,
	};

	return 0;
}

static int ax620e_pwm_round_waveform_fromhw(struct pwm_chip *chip,
					    struct pwm_device *pwm,
					    const void *_wfhw,
					    struct pwm_waveform *wf)
{
	struct ax620e_pwm *ax = pwmchip_get_drvdata(chip);
	const struct ax620e_pwm_waveform *wfhw = _wfhw;
	u64 period = (u64)wfhw->high + wfhw->low;

	if (!wfhw->enabled || period == 0) {
		*wf = (struct pwm_waveform){ };
		return 0;
	}

	*wf = (struct pwm_waveform){
		.period_length_ns = DIV64_U64_ROUND_UP(period * NSEC_PER_SEC,
						       ax->rate),
		.duty_length_ns = DIV64_U64_ROUND_UP((u64)wfhw->high * NSEC_PER_SEC,
						     ax->rate),
	};

	return 0;
}

static int ax620e_pwm_read_waveform(struct pwm_chip *chip,
				    struct pwm_device *pwm, void *_wfhw)
{
	struct ax620e_pwm *ax = pwmchip_get_drvdata(chip);
	struct ax620e_pwm_waveform *wfhw = _wfhw;
	unsigned int ch = pwm->hwpwm;
	u32 ctrl = readl(ax->base + AX620E_PWM_CTRL(ch));

	*wfhw = (struct ax620e_pwm_waveform){
		.high = readl(ax->base + AX620E_PWM_HIGH(ch)),
		.low = readl(ax->base + AX620E_PWM_LOW(ch)),
		.enabled = ctrl == (AX620E_PWM_CTRL_MODE | AX620E_PWM_CTRL_EN),
	};

	return 0;
}

static int ax620e_pwm_write_waveform(struct pwm_chip *chip,
				     struct pwm_device *pwm,
				     const void *_wfhw)
{
	struct ax620e_pwm *ax = pwmchip_get_drvdata(chip);
	const struct ax620e_pwm_waveform *wfhw = _wfhw;
	unsigned int ch = pwm->hwpwm;
	int ret;

	if (wfhw->enabled && !ax->enabled[ch]) {
		ret = clk_prepare_enable(ax->clk[ch]);
		if (ret)
			return ret;
	}

	/* The databook's order: stop the timer, load the counts, start it. */
	writel(AX620E_PWM_CTRL_MODE, ax->base + AX620E_PWM_CTRL(ch));

	if (wfhw->enabled) {
		writel(wfhw->low, ax->base + AX620E_PWM_LOW(ch));
		writel(wfhw->high, ax->base + AX620E_PWM_HIGH(ch));
		writel(AX620E_PWM_CTRL_MODE | AX620E_PWM_CTRL_EN,
		       ax->base + AX620E_PWM_CTRL(ch));
	} else if (ax->enabled[ch]) {
		clk_disable_unprepare(ax->clk[ch]);
	}

	ax->enabled[ch] = wfhw->enabled;

	return 0;
}

static const struct pwm_ops ax620e_pwm_ops = {
	.sizeof_wfhw = sizeof(struct ax620e_pwm_waveform),
	.round_waveform_tohw = ax620e_pwm_round_waveform_tohw,
	.round_waveform_fromhw = ax620e_pwm_round_waveform_fromhw,
	.read_waveform = ax620e_pwm_read_waveform,
	.write_waveform = ax620e_pwm_write_waveform,
};

static void ax620e_pwm_disable_clocks(void *data)
{
	struct ax620e_pwm *ax = data;
	unsigned int ch;

	for (ch = 0; ch < AX620E_PWM_CHANNELS; ch++)
		if (ax->enabled[ch])
			clk_disable_unprepare(ax->clk[ch]);
}

static int ax620e_pwm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct reset_control *rst;
	struct pwm_chip *chip;
	struct ax620e_pwm *ax;
	struct clk *pclk;
	unsigned int ch;
	int ret;

	chip = devm_pwmchip_alloc(dev, AX620E_PWM_CHANNELS, sizeof(*ax));
	if (IS_ERR(chip))
		return PTR_ERR(chip);
	ax = pwmchip_get_drvdata(chip);

	ax->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ax->base))
		return PTR_ERR(ax->base);

	pclk = devm_clk_get_enabled(dev, "pclk");
	if (IS_ERR(pclk))
		return dev_err_probe(dev, PTR_ERR(pclk),
				     "Failed to get the bus clock\n");

	for (ch = 0; ch < AX620E_PWM_CHANNELS; ch++) {
		char name[] = "ch0";

		name[2] += ch;
		ax->clk[ch] = devm_clk_get(dev, name);
		if (IS_ERR(ax->clk[ch]))
			return dev_err_probe(dev, PTR_ERR(ax->clk[ch]),
					     "Failed to get clock %s\n", name);
	}

	/* The channels count one timer clock: keep it at its rate. */
	ret = devm_clk_rate_exclusive_get(dev, ax->clk[0]);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to keep the clock rate\n");

	ax->rate = clk_get_rate(ax->clk[0]);
	if (!ax->rate || ax->rate > NSEC_PER_SEC)
		return dev_err_probe(dev, -EINVAL, "Invalid clock rate %lu\n",
				     ax->rate);

	rst = devm_reset_control_get_exclusive_deasserted(dev, NULL);
	if (IS_ERR(rst))
		return dev_err_probe(dev, PTR_ERR(rst),
				     "Failed to take the block out of reset\n");

	ret = devm_add_action_or_reset(dev, ax620e_pwm_disable_clocks, ax);
	if (ret)
		return ret;

	chip->ops = &ax620e_pwm_ops;

	ret = devm_pwmchip_add(dev, chip);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to add the PWM chip\n");

	return 0;
}

static const struct of_device_id ax620e_pwm_of_match[] = {
	{ .compatible = "axera,ax620e-pwm" },
	{ }
};
MODULE_DEVICE_TABLE(of, ax620e_pwm_of_match);

static struct platform_driver ax620e_pwm_driver = {
	.probe = ax620e_pwm_probe,
	.driver = {
		.name = "ax620e-pwm",
		.of_match_table = ax620e_pwm_of_match,
	},
};
module_platform_driver(ax620e_pwm_driver);

MODULE_DESCRIPTION("Axera AX620E PWM driver");
MODULE_LICENSE("GPL");
