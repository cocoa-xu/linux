// SPDX-License-Identifier: GPL-2.0-only
/*
 * Axera AX620E USB 2.0 PHY
 *
 * The UTMI PHY of the DWC3 controller has its reset and reference clock in
 * FLASH_SYS, next to the VBUS-valid override that the device controller
 * needs in order to see a host: the board does not route VBUS to the SoC.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/mfd/syscon.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>

#define AX620E_FLASH_USB2_CTRL_SET	0x4040
#define AX620E_FLASH_USB2_CTRL_CLR	0x8040
#define  AX620E_USB2_VBUSVALID		BIT(6)

struct ax620e_usb2_phy {
	struct regmap *flash_sys;
	struct reset_control *reset;
	struct clk_bulk_data *clks;
	int num_clks;
	enum phy_mode mode;
};

static int ax620e_usb2_phy_update_vbus(struct ax620e_usb2_phy *priv)
{
	unsigned int reg = priv->mode == PHY_MODE_USB_HOST ?
			   AX620E_FLASH_USB2_CTRL_CLR : AX620E_FLASH_USB2_CTRL_SET;

	return regmap_write(priv->flash_sys, reg, AX620E_USB2_VBUSVALID);
}

static int ax620e_usb2_phy_init(struct phy *phy)
{
	struct ax620e_usb2_phy *priv = phy_get_drvdata(phy);
	int ret;

	ret = clk_bulk_prepare_enable(priv->num_clks, priv->clks);
	if (ret)
		return ret;

	reset_control_assert(priv->reset);
	usleep_range(10, 20);
	ret = reset_control_deassert(priv->reset);
	if (ret)
		goto err_clk;

	ret = ax620e_usb2_phy_update_vbus(priv);
	if (ret)
		goto err_reset;

	return 0;

err_reset:
	reset_control_assert(priv->reset);
err_clk:
	clk_bulk_disable_unprepare(priv->num_clks, priv->clks);
	return ret;
}

static int ax620e_usb2_phy_exit(struct phy *phy)
{
	struct ax620e_usb2_phy *priv = phy_get_drvdata(phy);

	reset_control_assert(priv->reset);
	clk_bulk_disable_unprepare(priv->num_clks, priv->clks);

	return 0;
}

static int ax620e_usb2_phy_set_mode(struct phy *phy, enum phy_mode mode,
				    int submode)
{
	struct ax620e_usb2_phy *priv = phy_get_drvdata(phy);

	switch (mode) {
	case PHY_MODE_USB_HOST:
	case PHY_MODE_USB_DEVICE:
	case PHY_MODE_USB_OTG:
		priv->mode = mode;
		return ax620e_usb2_phy_update_vbus(priv);
	default:
		return -EINVAL;
	}
}

static const struct phy_ops ax620e_usb2_phy_ops = {
	.init = ax620e_usb2_phy_init,
	.exit = ax620e_usb2_phy_exit,
	.set_mode = ax620e_usb2_phy_set_mode,
	.owner = THIS_MODULE,
};

static int ax620e_usb2_phy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct phy_provider *provider;
	struct ax620e_usb2_phy *priv;
	struct device_node *np;
	struct phy *phy;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	priv->mode = PHY_MODE_USB_DEVICE;

	np = of_parse_phandle(dev->of_node, "axera,flash-sys", 0);
	if (!np)
		return dev_err_probe(dev, -EINVAL, "missing axera,flash-sys\n");
	priv->flash_sys = device_node_to_regmap(np);
	of_node_put(np);
	if (IS_ERR(priv->flash_sys))
		return dev_err_probe(dev, PTR_ERR(priv->flash_sys),
				     "no FLASH_SYS registers\n");

	priv->reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(priv->reset))
		return dev_err_probe(dev, PTR_ERR(priv->reset), "no reset\n");

	priv->num_clks = devm_clk_bulk_get_all(dev, &priv->clks);
	if (priv->num_clks < 0)
		return dev_err_probe(dev, priv->num_clks, "no clocks\n");

	phy = devm_phy_create(dev, NULL, &ax620e_usb2_phy_ops);
	if (IS_ERR(phy))
		return PTR_ERR(phy);
	phy_set_drvdata(phy, priv);

	provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);

	return PTR_ERR_OR_ZERO(provider);
}

static const struct of_device_id ax620e_usb2_phy_of_match[] = {
	{ .compatible = "axera,ax620e-usb2-phy" },
	{ }
};
MODULE_DEVICE_TABLE(of, ax620e_usb2_phy_of_match);

static struct platform_driver ax620e_usb2_phy_driver = {
	.probe = ax620e_usb2_phy_probe,
	.driver = {
		.name = "ax620e-usb2-phy",
		.of_match_table = ax620e_usb2_phy_of_match,
	},
};
module_platform_driver(ax620e_usb2_phy_driver);

MODULE_DESCRIPTION("Axera AX620E USB 2.0 PHY driver");
MODULE_LICENSE("GPL");
