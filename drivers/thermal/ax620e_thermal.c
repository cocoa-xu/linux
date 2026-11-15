// SPDX-License-Identifier: GPL-2.0-only
/*
 * Axera AX620E temperature sensor
 *
 * The sensor shares a monitor block with the general purpose ADC: the block
 * converts the enabled channels in turn into 10-bit codes, and measures its
 * own reference, which scales the sensor code. The SPL leaves a reference
 * trim from the eFuses in IRAM.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/nvmem-consumer.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/thermal.h>

#define THM_EN			0x00
#define THM_MA_CTRL		0x0c
#define THM_CTRL		0x18
#define  THM_CTRL_VREF_SHIFT	5
#define  THM_CTRL_DEFAULT	0x18
#define THM_CLK_EN		0x1c
#define  THM_CLK_EN_ADC		BIT(0)
#define  THM_CLK_EN_SENSOR	BIT(1)
#define THM_RSTN		0x24
#define THM_FILTER_SEL		0x2c
#define THM_FILTER_EN		0x30
#define THM_MSR_ONE_EN		0x38
#define  THM_MSR_ONE_VREF	BIT(14)
#define THM_DATA0		0x78
#define  THM_DATA_CODE		GENMASK(9, 0)
#define THM_MSR_VREF		0xb0
#define THM_MON_CH		0xc4
#define  THM_MON_CH_SENSOR0	BIT(0)
#define  THM_MON_CH_ADC		GENMASK(13, 10)
#define  THM_MON_CH_VREF	BIT(14)
#define THM_MON_EN		0xc8
#define THM_MON_INTERVAL	0xcc
#define THM_INT_MASK		0x104

/* Without a trim in the eFuses, as the vendor driver does. */
#define THM_VREF_TRIM_DEFAULT	7
/* A typical reference code, for a conversion that has not run yet. */
#define THM_VREF_TYPICAL	0x1f1

struct ax620e_thermal {
	void __iomem *base;
};

static void ax620e_thermal_set_bits(struct ax620e_thermal *thm, u32 reg,
				    u32 bits)
{
	writel(readl(thm->base + reg) | bits, thm->base + reg);
}

/* The vendor's sequence: the monitor converts the ADC channels as well. */
static void ax620e_thermal_start(struct ax620e_thermal *thm, u32 trim)
{
	writel(0, thm->base + THM_RSTN);
	writel(0, thm->base + THM_MON_EN);

	ax620e_thermal_set_bits(thm, THM_CLK_EN, THM_CLK_EN_ADC);
	writel(0, thm->base + THM_FILTER_SEL);
	writel(0xf, thm->base + THM_FILTER_EN);
	writel(0x100, thm->base + THM_MON_INTERVAL);
	ax620e_thermal_set_bits(thm, THM_MON_CH, THM_MON_CH_ADC);

	writel(0, thm->base + THM_INT_MASK);
	writel(1, thm->base + THM_MA_CTRL);
	writel(1, thm->base + THM_EN);
	writel(trim << THM_CTRL_VREF_SHIFT | THM_CTRL_DEFAULT,
	       thm->base + THM_CTRL);
	ax620e_thermal_set_bits(thm, THM_CLK_EN, THM_CLK_EN_SENSOR);
	ax620e_thermal_set_bits(thm, THM_MON_CH, THM_MON_CH_SENSOR0);

	writel(1, thm->base + THM_RSTN);
	writel(THM_MSR_ONE_VREF, thm->base + THM_MSR_ONE_EN);
	usleep_range(100, 200);

	ax620e_thermal_set_bits(thm, THM_MON_CH, THM_MON_CH_VREF);
	writel(1, thm->base + THM_MON_EN);
}

static int ax620e_thermal_get_temp(struct thermal_zone_device *tz, int *temp)
{
	struct ax620e_thermal *thm = thermal_zone_device_priv(tz);
	u32 code = readl(thm->base + THM_DATA0) & THM_DATA_CODE;
	u32 vref = readl(thm->base + THM_MSR_VREF) ?: THM_VREF_TYPICAL;

	/* 90000 * code / (242 * vref) kelvin, less the vendor's 4.5 K offset. */
	*temp = (int)div_u64(90000000ULL * code, 242 * vref) - 273150 - 4500;

	return 0;
}

static const struct thermal_zone_device_ops ax620e_thermal_ops = {
	.get_temp = ax620e_thermal_get_temp,
};

static int ax620e_thermal_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct thermal_zone_device *tz;
	struct ax620e_thermal *thm;
	struct reset_control *rst;
	struct clk_bulk_data *clks;
	u32 trim;
	int ret;

	thm = devm_kzalloc(dev, sizeof(*thm), GFP_KERNEL);
	if (!thm)
		return -ENOMEM;

	thm->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(thm->base))
		return PTR_ERR(thm->base);

	ret = devm_clk_bulk_get_all_enabled(dev, &clks);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to enable the clocks\n");

	rst = devm_reset_control_get_exclusive_deasserted(dev, "apb");
	if (IS_ERR(rst))
		return dev_err_probe(dev, PTR_ERR(rst), "failed to deassert apb\n");
	rst = devm_reset_control_get_exclusive_deasserted(dev, "sensor");
	if (IS_ERR(rst))
		return dev_err_probe(dev, PTR_ERR(rst),
				     "failed to deassert sensor\n");

	ret = nvmem_cell_read_u32(dev, "vref", &trim);
	if (ret == -EPROBE_DEFER)
		return ret;
	if (ret || !trim)
		trim = THM_VREF_TRIM_DEFAULT;

	ax620e_thermal_start(thm, trim);

	tz = devm_thermal_of_zone_register(dev, 0, thm, &ax620e_thermal_ops);
	if (IS_ERR(tz))
		return dev_err_probe(dev, PTR_ERR(tz),
				     "failed to register the thermal zone\n");

	return 0;
}

static const struct of_device_id ax620e_thermal_of_match[] = {
	{ .compatible = "axera,ax620e-tsensor" },
	{ }
};
MODULE_DEVICE_TABLE(of, ax620e_thermal_of_match);

static struct platform_driver ax620e_thermal_driver = {
	.probe = ax620e_thermal_probe,
	.driver = {
		.name = "ax620e-thermal",
		.of_match_table = ax620e_thermal_of_match,
	},
};
module_platform_driver(ax620e_thermal_driver);

MODULE_DESCRIPTION("Axera AX620E temperature sensor driver");
MODULE_LICENSE("GPL");
