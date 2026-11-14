// SPDX-License-Identifier: GPL-2.0-only
/*
 * Axera AX620E VPU_SYS clock and reset controller
 *
 * The codec clock gates and resets are changed through separate SET and
 * CLR registers, so no read-modify-write is needed for them; the source
 * muxes are plain fields of one register.
 */

#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <linux/spinlock.h>

#include <dt-bindings/clock/axera,ax620e-vpu-sys.h>
#include <dt-bindings/reset/axera,ax620e-vpu-sys.h>

#define VPU_CLK_MUX		0x000
#define VPU_CLK_EB		0x008
#define VPU_CLK_EB_SET		0x114
#define VPU_CLK_EB_CLR		0x118
#define VPU_RST_ASSERT		0x11c
#define VPU_RST_DEASSERT	0x120

#define VPU_NUM_CLKS		(AX620E_VPU_CLK_VENC + 1)

struct ax620e_vpu_sys {
	void __iomem *base;
	/* Protects the mux register. */
	spinlock_t lock;
	struct reset_controller_dev rcdev;
	struct clk_hw_onecell_data *clk_data;
};

struct ax620e_vpu_gate {
	struct clk_hw hw;
	void __iomem *base;
	u32 mask;
};

struct ax620e_vpu_mux_desc {
	unsigned int id;
	const char *name;
	u8 shift;
};

struct ax620e_vpu_gate_desc {
	unsigned int id;
	const char *name;
	unsigned int parent;
	u8 bit;
};

static const struct clk_parent_data ax620e_vpu_src_parents[] = {
	{ .fw_name = "cpll_208m" },
	{ .fw_name = "cpll_312m" },
	{ .fw_name = "epll_375m" },
	{ .fw_name = "cpll_416m" },
	{ .fw_name = "epll_500m" },
	{ .fw_name = "npll_533m" },
};

static const struct ax620e_vpu_mux_desc ax620e_vpu_muxes[] = {
	{ AX620E_VPU_CLK_JENC_SEL, "vpu_jenc_sel", 0 },
	{ AX620E_VPU_CLK_VDEC_SEL, "vpu_vdec_sel", 3 },
	{ AX620E_VPU_CLK_GLB_SEL, "vpu_glb_sel", 6 },
};

static const struct ax620e_vpu_gate_desc ax620e_vpu_gates[] = {
	{ AX620E_VPU_CLK_JENC, "vpu_jenc", AX620E_VPU_CLK_JENC_SEL, 4 },
	{ AX620E_VPU_CLK_VDEC, "vpu_vdec", AX620E_VPU_CLK_VDEC_SEL, 6 },
	{ AX620E_VPU_CLK_VENC, "vpu_venc", AX620E_VPU_CLK_GLB_SEL, 7 },
};

static const u8 ax620e_vpu_reset_bits[] = {
	[AX620E_VPU_RST_JENC] = 4,
	[AX620E_VPU_RST_VDEC] = 6,
	[AX620E_VPU_RST_VENC] = 7,
};

static struct ax620e_vpu_gate *to_ax620e_vpu_gate(struct clk_hw *hw)
{
	return container_of(hw, struct ax620e_vpu_gate, hw);
}

static int ax620e_vpu_gate_enable(struct clk_hw *hw)
{
	struct ax620e_vpu_gate *gate = to_ax620e_vpu_gate(hw);

	writel(gate->mask, gate->base + VPU_CLK_EB_SET);

	return 0;
}

static void ax620e_vpu_gate_disable(struct clk_hw *hw)
{
	struct ax620e_vpu_gate *gate = to_ax620e_vpu_gate(hw);

	writel(gate->mask, gate->base + VPU_CLK_EB_CLR);
}

static int ax620e_vpu_gate_is_enabled(struct clk_hw *hw)
{
	struct ax620e_vpu_gate *gate = to_ax620e_vpu_gate(hw);

	return !!(readl(gate->base + VPU_CLK_EB) & gate->mask);
}

static const struct clk_ops ax620e_vpu_gate_ops = {
	.enable = ax620e_vpu_gate_enable,
	.disable = ax620e_vpu_gate_disable,
	.is_enabled = ax620e_vpu_gate_is_enabled,
};

static struct ax620e_vpu_sys *to_ax620e_vpu_sys(struct reset_controller_dev *rcdev)
{
	return container_of(rcdev, struct ax620e_vpu_sys, rcdev);
}

static int ax620e_vpu_reset_assert(struct reset_controller_dev *rcdev,
				   unsigned long id)
{
	struct ax620e_vpu_sys *vpu = to_ax620e_vpu_sys(rcdev);

	writel(BIT(ax620e_vpu_reset_bits[id]), vpu->base + VPU_RST_ASSERT);

	return 0;
}

static int ax620e_vpu_reset_deassert(struct reset_controller_dev *rcdev,
				     unsigned long id)
{
	struct ax620e_vpu_sys *vpu = to_ax620e_vpu_sys(rcdev);

	writel(BIT(ax620e_vpu_reset_bits[id]), vpu->base + VPU_RST_DEASSERT);

	return 0;
}

static int ax620e_vpu_reset_reset(struct reset_controller_dev *rcdev,
				  unsigned long id)
{
	ax620e_vpu_reset_assert(rcdev, id);
	udelay(1);
	return ax620e_vpu_reset_deassert(rcdev, id);
}

static const struct reset_control_ops ax620e_vpu_reset_ops = {
	.assert = ax620e_vpu_reset_assert,
	.deassert = ax620e_vpu_reset_deassert,
	.reset = ax620e_vpu_reset_reset,
};

static int ax620e_vpu_sys_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ax620e_vpu_sys *vpu;
	struct clk_hw *hw;
	unsigned int i;
	int ret;

	vpu = devm_kzalloc(dev, sizeof(*vpu), GFP_KERNEL);
	if (!vpu)
		return -ENOMEM;
	spin_lock_init(&vpu->lock);

	vpu->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(vpu->base))
		return PTR_ERR(vpu->base);

	vpu->clk_data = devm_kzalloc(dev, struct_size(vpu->clk_data, hws,
						      VPU_NUM_CLKS),
				     GFP_KERNEL);
	if (!vpu->clk_data)
		return -ENOMEM;
	vpu->clk_data->num = VPU_NUM_CLKS;

	for (i = 0; i < ARRAY_SIZE(ax620e_vpu_muxes); i++) {
		const struct ax620e_vpu_mux_desc *mux = &ax620e_vpu_muxes[i];

		hw = devm_clk_hw_register_mux_parent_data_table(dev, mux->name,
				ax620e_vpu_src_parents,
				ARRAY_SIZE(ax620e_vpu_src_parents),
				CLK_SET_RATE_NO_REPARENT,
				vpu->base + VPU_CLK_MUX, mux->shift, 3, 0,
				NULL, &vpu->lock);
		if (IS_ERR(hw))
			return PTR_ERR(hw);
		vpu->clk_data->hws[mux->id] = hw;
	}

	for (i = 0; i < ARRAY_SIZE(ax620e_vpu_gates); i++) {
		const struct ax620e_vpu_gate_desc *desc = &ax620e_vpu_gates[i];
		struct clk_init_data init = { };
		struct ax620e_vpu_gate *gate;
		const struct clk_hw *parent = vpu->clk_data->hws[desc->parent];

		gate = devm_kzalloc(dev, sizeof(*gate), GFP_KERNEL);
		if (!gate)
			return -ENOMEM;
		gate->base = vpu->base;
		gate->mask = BIT(desc->bit);
		init.name = desc->name;
		init.ops = &ax620e_vpu_gate_ops;
		init.parent_hws = &parent;
		init.num_parents = 1;
		init.flags = CLK_SET_RATE_PARENT;
		gate->hw.init = &init;

		ret = devm_clk_hw_register(dev, &gate->hw);
		if (ret)
			return ret;
		vpu->clk_data->hws[desc->id] = &gate->hw;
	}

	ret = devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get,
					  vpu->clk_data);
	if (ret)
		return ret;

	vpu->rcdev.owner = THIS_MODULE;
	vpu->rcdev.ops = &ax620e_vpu_reset_ops;
	vpu->rcdev.of_node = dev->of_node;
	vpu->rcdev.nr_resets = ARRAY_SIZE(ax620e_vpu_reset_bits);

	return devm_reset_controller_register(dev, &vpu->rcdev);
}

static const struct of_device_id ax620e_vpu_sys_of_match[] = {
	{ .compatible = "axera,ax620e-vpu-sys" },
	{ }
};
MODULE_DEVICE_TABLE(of, ax620e_vpu_sys_of_match);

static struct platform_driver ax620e_vpu_sys_driver = {
	.probe = ax620e_vpu_sys_probe,
	.driver = {
		.name = "ax620e-vpu-sys",
		.of_match_table = ax620e_vpu_sys_of_match,
	},
};
module_platform_driver(ax620e_vpu_sys_driver);

MODULE_DESCRIPTION("Axera AX620E VPU clock and reset controller");
MODULE_LICENSE("GPL");
