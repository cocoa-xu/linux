// SPDX-License-Identifier: GPL-2.0-only
/*
 * Axera AX620E chip information in IRAM
 *
 * The SPL copies eFuse data to a fixed IRAM area: board and chip type, the
 * unique ID and sensor calibration. Key material sits right before those
 * fields, so the device tree node covers the fields only.
 */

#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/nvmem-provider.h>
#include <linux/platform_device.h>

static int axera_misc_info_read(void *priv, unsigned int offset, void *val,
				size_t bytes)
{
	void __iomem *base = (void __iomem __force *)priv;

	memcpy_fromio(val, base + offset, bytes);

	return 0;
}

static int axera_misc_info_probe(struct platform_device *pdev)
{
	struct nvmem_config config = {
		.dev = &pdev->dev,
		.name = "axera-misc-info",
		.id = NVMEM_DEVID_NONE,
		.owner = THIS_MODULE,
		.read_only = true,
		.root_only = true,
		.word_size = 4,
		.stride = 4,
		.reg_read = axera_misc_info_read,
	};
	struct resource *res;
	void __iomem *base;

	base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	config.priv = (void __force *)base;
	config.size = resource_size(res);

	return PTR_ERR_OR_ZERO(devm_nvmem_register(&pdev->dev, &config));
}

static const struct of_device_id axera_misc_info_of_match[] = {
	{ .compatible = "axera,ax620e-misc-info" },
	{ }
};
MODULE_DEVICE_TABLE(of, axera_misc_info_of_match);

static struct platform_driver axera_misc_info_driver = {
	.probe = axera_misc_info_probe,
	.driver = {
		.name = "axera-misc-info",
		.of_match_table = axera_misc_info_of_match,
	},
};
module_platform_driver(axera_misc_info_driver);

MODULE_DESCRIPTION("Axera AX620E chip information in IRAM");
MODULE_LICENSE("GPL");
