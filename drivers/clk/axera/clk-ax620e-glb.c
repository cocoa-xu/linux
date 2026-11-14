// SPDX-License-Identifier: GPL-2.0-only
/*
 * Axera AX620E clock and reset controllers (VPU_SYS, PERIPH_SYS)
 *
 * Each subsystem has a "GLB" register block. Every state register R has a
 * SET and a CLR alias at set_base + 2 * R and set_base + 2 * R + 4: writing
 * a 1 there sets or clears that bit, so gates, resets and mux fields can be
 * changed without a read-modify-write.
 */

#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/reset-controller.h>

#include <dt-bindings/clock/axera,ax620e-periph-sys.h>
#include <dt-bindings/clock/axera,ax620e-vpu-sys.h>
#include <dt-bindings/reset/axera,ax620e-periph-sys.h>
#include <dt-bindings/reset/axera,ax620e-vpu-sys.h>

#define AX_GLB_NO_PARENT		(-1)

struct ax620e_glb_mux {
	unsigned int id;
	const char *name;
	const char * const *parents;
	u8 num_parents;
	u16 reg;
	u8 shift;
	u8 width;
};

struct ax620e_glb_gate {
	unsigned int id;
	const char *name;
	/* A mux of this block, or AX_GLB_NO_PARENT to use parent_fw. */
	int parent;
	const char *parent_fw;
	u16 reg;
	u8 bit;
};

struct ax620e_glb_reset {
	u16 reg;
	u32 mask;
};

struct ax620e_glb_data {
	u16 set_base;
	const struct ax620e_glb_mux *muxes;
	unsigned int num_muxes;
	const struct ax620e_glb_gate *gates;
	unsigned int num_gates;
	const struct ax620e_glb_reset *resets;
	unsigned int num_resets;
	unsigned int num_clks;
};

struct ax620e_glb {
	void __iomem *base;
	const struct ax620e_glb_data *data;
	struct reset_controller_dev rcdev;
};

struct ax620e_glb_clk {
	struct clk_hw hw;
	struct ax620e_glb *glb;
	u16 reg;
	u8 shift;
	u32 mask;
};

static void __iomem *ax620e_glb_set(struct ax620e_glb *glb, u16 reg)
{
	return glb->base + glb->data->set_base + 2 * reg;
}

static void __iomem *ax620e_glb_clr(struct ax620e_glb *glb, u16 reg)
{
	return ax620e_glb_set(glb, reg) + 4;
}

static struct ax620e_glb_clk *to_ax620e_glb_clk(struct clk_hw *hw)
{
	return container_of(hw, struct ax620e_glb_clk, hw);
}

static int ax620e_glb_gate_enable(struct clk_hw *hw)
{
	struct ax620e_glb_clk *clk = to_ax620e_glb_clk(hw);

	writel(clk->mask, ax620e_glb_set(clk->glb, clk->reg));

	return 0;
}

static void ax620e_glb_gate_disable(struct clk_hw *hw)
{
	struct ax620e_glb_clk *clk = to_ax620e_glb_clk(hw);

	writel(clk->mask, ax620e_glb_clr(clk->glb, clk->reg));
}

static int ax620e_glb_gate_is_enabled(struct clk_hw *hw)
{
	struct ax620e_glb_clk *clk = to_ax620e_glb_clk(hw);

	return !!(readl(clk->glb->base + clk->reg) & clk->mask);
}

static const struct clk_ops ax620e_glb_gate_ops = {
	.enable = ax620e_glb_gate_enable,
	.disable = ax620e_glb_gate_disable,
	.is_enabled = ax620e_glb_gate_is_enabled,
};

static u8 ax620e_glb_mux_get_parent(struct clk_hw *hw)
{
	struct ax620e_glb_clk *clk = to_ax620e_glb_clk(hw);

	return (readl(clk->glb->base + clk->reg) & clk->mask) >> clk->shift;
}

static int ax620e_glb_mux_set_parent(struct clk_hw *hw, u8 index)
{
	struct ax620e_glb_clk *clk = to_ax620e_glb_clk(hw);

	writel(clk->mask, ax620e_glb_clr(clk->glb, clk->reg));
	writel(index << clk->shift, ax620e_glb_set(clk->glb, clk->reg));

	return 0;
}

static const struct clk_ops ax620e_glb_mux_ops = {
	.get_parent = ax620e_glb_mux_get_parent,
	.set_parent = ax620e_glb_mux_set_parent,
	.determine_rate = __clk_mux_determine_rate,
};

static struct ax620e_glb *to_ax620e_glb(struct reset_controller_dev *rcdev)
{
	return container_of(rcdev, struct ax620e_glb, rcdev);
}

static int ax620e_glb_reset_assert(struct reset_controller_dev *rcdev,
				   unsigned long id)
{
	struct ax620e_glb *glb = to_ax620e_glb(rcdev);
	const struct ax620e_glb_reset *rst = &glb->data->resets[id];

	writel(rst->mask, ax620e_glb_set(glb, rst->reg));

	return 0;
}

static int ax620e_glb_reset_deassert(struct reset_controller_dev *rcdev,
				     unsigned long id)
{
	struct ax620e_glb *glb = to_ax620e_glb(rcdev);
	const struct ax620e_glb_reset *rst = &glb->data->resets[id];

	writel(rst->mask, ax620e_glb_clr(glb, rst->reg));

	return 0;
}

static int ax620e_glb_reset_reset(struct reset_controller_dev *rcdev,
				  unsigned long id)
{
	ax620e_glb_reset_assert(rcdev, id);
	udelay(1);
	return ax620e_glb_reset_deassert(rcdev, id);
}

static int ax620e_glb_reset_status(struct reset_controller_dev *rcdev,
				   unsigned long id)
{
	struct ax620e_glb *glb = to_ax620e_glb(rcdev);
	const struct ax620e_glb_reset *rst = &glb->data->resets[id];

	return !!(readl(glb->base + rst->reg) & rst->mask);
}

static const struct reset_control_ops ax620e_glb_reset_ops = {
	.assert = ax620e_glb_reset_assert,
	.deassert = ax620e_glb_reset_deassert,
	.reset = ax620e_glb_reset_reset,
	.status = ax620e_glb_reset_status,
};

static struct clk_hw *ax620e_glb_register(struct device *dev,
					  struct ax620e_glb *glb,
					  struct clk_init_data *init,
					  u16 reg, u8 shift, u32 mask)
{
	struct ax620e_glb_clk *clk;
	int ret;

	clk = devm_kzalloc(dev, sizeof(*clk), GFP_KERNEL);
	if (!clk)
		return ERR_PTR(-ENOMEM);
	clk->glb = glb;
	clk->reg = reg;
	clk->shift = shift;
	clk->mask = mask;
	clk->hw.init = init;

	ret = devm_clk_hw_register(dev, &clk->hw);

	return ret ? ERR_PTR(ret) : &clk->hw;
}

static int ax620e_glb_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct ax620e_glb_data *data;
	struct clk_hw_onecell_data *clk_data;
	struct ax620e_glb *glb;
	unsigned int i, j;
	struct clk_hw *hw;
	int ret;

	data = device_get_match_data(dev);
	glb = devm_kzalloc(dev, sizeof(*glb), GFP_KERNEL);
	if (!glb)
		return -ENOMEM;
	glb->data = data;

	glb->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(glb->base))
		return PTR_ERR(glb->base);

	clk_data = devm_kzalloc(dev, struct_size(clk_data, hws, data->num_clks),
				GFP_KERNEL);
	if (!clk_data)
		return -ENOMEM;
	clk_data->num = data->num_clks;
	for (i = 0; i < data->num_clks; i++)
		clk_data->hws[i] = ERR_PTR(-ENOENT);

	for (i = 0; i < data->num_muxes; i++) {
		const struct ax620e_glb_mux *mux = &data->muxes[i];
		struct clk_parent_data *parents;
		struct clk_init_data init = { };

		parents = devm_kcalloc(dev, mux->num_parents, sizeof(*parents),
				       GFP_KERNEL);
		if (!parents)
			return -ENOMEM;
		for (j = 0; j < mux->num_parents; j++)
			parents[j].fw_name = mux->parents[j];

		init.name = mux->name;
		init.ops = &ax620e_glb_mux_ops;
		init.parent_data = parents;
		init.num_parents = mux->num_parents;
		init.flags = CLK_SET_RATE_NO_REPARENT;
		hw = ax620e_glb_register(dev, glb, &init, mux->reg, mux->shift,
					 GENMASK(mux->shift + mux->width - 1,
						 mux->shift));
		if (IS_ERR(hw))
			return PTR_ERR(hw);
		clk_data->hws[mux->id] = hw;
	}

	for (i = 0; i < data->num_gates; i++) {
		const struct ax620e_glb_gate *gate = &data->gates[i];
		struct clk_parent_data parent = { };
		struct clk_init_data init = { };

		if (gate->parent == AX_GLB_NO_PARENT)
			parent.fw_name = gate->parent_fw;
		else
			parent.hw = clk_data->hws[gate->parent];

		init.name = gate->name;
		init.ops = &ax620e_glb_gate_ops;
		init.parent_data = &parent;
		init.num_parents = 1;
		init.flags = CLK_SET_RATE_PARENT;
		hw = ax620e_glb_register(dev, glb, &init, gate->reg, 0,
					 BIT(gate->bit));
		if (IS_ERR(hw))
			return PTR_ERR(hw);
		clk_data->hws[gate->id] = hw;
	}

	ret = devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get, clk_data);
	if (ret)
		return ret;

	glb->rcdev.owner = THIS_MODULE;
	glb->rcdev.ops = &ax620e_glb_reset_ops;
	glb->rcdev.of_node = dev->of_node;
	glb->rcdev.nr_resets = data->num_resets;

	return devm_reset_controller_register(dev, &glb->rcdev);
}

static const char * const vpu_src_parents[] = {
	"cpll_208m", "cpll_312m", "epll_375m", "cpll_416m", "epll_500m",
	"npll_533m",
};

static const struct ax620e_glb_mux vpu_muxes[] = {
	{ AX620E_VPU_CLK_JENC_SEL, "vpu_jenc_sel", vpu_src_parents, 6, 0x0, 0, 3 },
	{ AX620E_VPU_CLK_VDEC_SEL, "vpu_vdec_sel", vpu_src_parents, 6, 0x0, 3, 3 },
	{ AX620E_VPU_CLK_GLB_SEL, "vpu_glb_sel", vpu_src_parents, 6, 0x0, 6, 3 },
};

static const struct ax620e_glb_gate vpu_gates[] = {
	{ AX620E_VPU_CLK_JENC, "vpu_jenc", AX620E_VPU_CLK_JENC_SEL, NULL, 0x8, 4 },
	{ AX620E_VPU_CLK_VDEC, "vpu_vdec", AX620E_VPU_CLK_VDEC_SEL, NULL, 0x8, 6 },
	{ AX620E_VPU_CLK_VENC, "vpu_venc", AX620E_VPU_CLK_GLB_SEL, NULL, 0x8, 7 },
};

static const struct ax620e_glb_reset vpu_resets[] = {
	[AX620E_VPU_RST_JENC] = { 0xc, BIT(4) },
	[AX620E_VPU_RST_VDEC] = { 0xc, BIT(6) },
	[AX620E_VPU_RST_VENC] = { 0xc, BIT(7) },
};

static const struct ax620e_glb_data ax620e_vpu_sys_data = {
	.set_base = 0x104,
	.muxes = vpu_muxes,
	.num_muxes = ARRAY_SIZE(vpu_muxes),
	.gates = vpu_gates,
	.num_gates = ARRAY_SIZE(vpu_gates),
	.resets = vpu_resets,
	.num_resets = ARRAY_SIZE(vpu_resets),
	.num_clks = AX620E_VPU_CLK_VENC + 1,
};

static const char * const periph_gpio_parents[] = { "rtc_32k", "cpll_24m" };
static const char * const periph_i2c_parents[] = {
	"cpll_24m", "epll_50m", "cpll_156m", "cpll_208m",
};

static const struct ax620e_glb_mux periph_muxes[] = {
	{ AX620E_PERIPH_CLK_GPIO_SEL, "periph_gpio_sel", periph_gpio_parents, 2, 0x0, 2, 1 },
	{ AX620E_PERIPH_CLK_I2C_SEL, "periph_i2c_sel", periph_i2c_parents, 4, 0x0, 3, 2 },
};

#define PERIPH_GATE(_id, _name, _parent, _reg, _bit) \
	{ _id, _name, _parent, "pclk_top", _reg, _bit }

static const struct ax620e_glb_gate periph_gates[] = {
	PERIPH_GATE(AX620E_PERIPH_CLK_GPIO0, "gpio0", AX620E_PERIPH_CLK_GPIO_SEL, 0x8, 4),
	PERIPH_GATE(AX620E_PERIPH_CLK_GPIO1, "gpio1", AX620E_PERIPH_CLK_GPIO_SEL, 0x8, 5),
	PERIPH_GATE(AX620E_PERIPH_CLK_GPIO2, "gpio2", AX620E_PERIPH_CLK_GPIO_SEL, 0x8, 6),
	PERIPH_GATE(AX620E_PERIPH_CLK_GPIO3, "gpio3", AX620E_PERIPH_CLK_GPIO_SEL, 0x8, 7),
	PERIPH_GATE(AX620E_PERIPH_CLK_I2C0, "i2c0", AX620E_PERIPH_CLK_I2C_SEL, 0x8, 8),
	PERIPH_GATE(AX620E_PERIPH_CLK_I2C1, "i2c1", AX620E_PERIPH_CLK_I2C_SEL, 0x8, 9),
	PERIPH_GATE(AX620E_PERIPH_CLK_I2C2, "i2c2", AX620E_PERIPH_CLK_I2C_SEL, 0x8, 10),
	PERIPH_GATE(AX620E_PERIPH_CLK_I2C3, "i2c3", AX620E_PERIPH_CLK_I2C_SEL, 0x8, 11),
	PERIPH_GATE(AX620E_PERIPH_CLK_I2C4, "i2c4", AX620E_PERIPH_CLK_I2C_SEL, 0x8, 12),
	PERIPH_GATE(AX620E_PERIPH_CLK_I2C5, "i2c5", AX620E_PERIPH_CLK_I2C_SEL, 0x8, 13),
	PERIPH_GATE(AX620E_PERIPH_CLK_I2C6, "i2c6", AX620E_PERIPH_CLK_I2C_SEL, 0x8, 14),
	PERIPH_GATE(AX620E_PERIPH_CLK_I2C7, "i2c7", AX620E_PERIPH_CLK_I2C_SEL, 0x8, 15),
	PERIPH_GATE(AX620E_PERIPH_PCLK_GPIO0, "pclk_gpio0", AX_GLB_NO_PARENT, 0xc, 13),
	PERIPH_GATE(AX620E_PERIPH_PCLK_GPIO1, "pclk_gpio1", AX_GLB_NO_PARENT, 0xc, 14),
	PERIPH_GATE(AX620E_PERIPH_PCLK_GPIO2, "pclk_gpio2", AX_GLB_NO_PARENT, 0xc, 15),
	PERIPH_GATE(AX620E_PERIPH_PCLK_GPIO3, "pclk_gpio3", AX_GLB_NO_PARENT, 0xc, 16),
	PERIPH_GATE(AX620E_PERIPH_PCLK_I2C0, "pclk_i2c0", AX_GLB_NO_PARENT, 0xc, 17),
	PERIPH_GATE(AX620E_PERIPH_PCLK_I2C1, "pclk_i2c1", AX_GLB_NO_PARENT, 0xc, 18),
	PERIPH_GATE(AX620E_PERIPH_PCLK_I2C2, "pclk_i2c2", AX_GLB_NO_PARENT, 0xc, 19),
	PERIPH_GATE(AX620E_PERIPH_PCLK_I2C3, "pclk_i2c3", AX_GLB_NO_PARENT, 0xc, 20),
	PERIPH_GATE(AX620E_PERIPH_PCLK_I2C4, "pclk_i2c4", AX_GLB_NO_PARENT, 0xc, 21),
	PERIPH_GATE(AX620E_PERIPH_PCLK_I2C5, "pclk_i2c5", AX_GLB_NO_PARENT, 0xc, 22),
	PERIPH_GATE(AX620E_PERIPH_PCLK_I2C6, "pclk_i2c6", AX_GLB_NO_PARENT, 0xc, 23),
	PERIPH_GATE(AX620E_PERIPH_PCLK_I2C7, "pclk_i2c7", AX_GLB_NO_PARENT, 0xc, 24),
};

/* Each block has an APB reset and a core reset; they are used together. */
static const struct ax620e_glb_reset periph_resets[] = {
	[AX620E_PERIPH_RST_GPIO0] = { 0x18, BIT(10) | BIT(11) },
	[AX620E_PERIPH_RST_GPIO1] = { 0x18, BIT(12) | BIT(13) },
	[AX620E_PERIPH_RST_GPIO2] = { 0x18, BIT(14) | BIT(15) },
	[AX620E_PERIPH_RST_GPIO3] = { 0x18, BIT(16) | BIT(17) },
	[AX620E_PERIPH_RST_I2C0] = { 0x18, BIT(18) | BIT(19) },
	[AX620E_PERIPH_RST_I2C1] = { 0x18, BIT(20) | BIT(21) },
	[AX620E_PERIPH_RST_I2C2] = { 0x18, BIT(22) | BIT(23) },
	[AX620E_PERIPH_RST_I2C3] = { 0x18, BIT(24) | BIT(25) },
	[AX620E_PERIPH_RST_I2C4] = { 0x18, BIT(26) | BIT(27) },
	[AX620E_PERIPH_RST_I2C5] = { 0x18, BIT(28) | BIT(29) },
};

static const struct ax620e_glb_data ax620e_periph_sys_data = {
	.set_base = 0xa8,
	.muxes = periph_muxes,
	.num_muxes = ARRAY_SIZE(periph_muxes),
	.gates = periph_gates,
	.num_gates = ARRAY_SIZE(periph_gates),
	.resets = periph_resets,
	.num_resets = ARRAY_SIZE(periph_resets),
	.num_clks = AX620E_PERIPH_PCLK_I2C7 + 1,
};

static const struct of_device_id ax620e_glb_of_match[] = {
	{ .compatible = "axera,ax620e-periph-sys", .data = &ax620e_periph_sys_data },
	{ .compatible = "axera,ax620e-vpu-sys", .data = &ax620e_vpu_sys_data },
	{ }
};
MODULE_DEVICE_TABLE(of, ax620e_glb_of_match);

static struct platform_driver ax620e_glb_driver = {
	.probe = ax620e_glb_probe,
	.driver = {
		.name = "ax620e-glb",
		.of_match_table = ax620e_glb_of_match,
	},
};
module_platform_driver(ax620e_glb_driver);

MODULE_DESCRIPTION("Axera AX620E clock and reset controllers");
MODULE_LICENSE("GPL");
