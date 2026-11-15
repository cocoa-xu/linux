// SPDX-License-Identifier: GPL-2.0-only
/*
 * Axera AX620E peripheral DMA controller
 *
 * Sixteen channels, each walking a list of 64-byte aligned descriptors. The
 * peripheral request that paces a channel is chosen outside the controller,
 * in the peripheral system controller (PERIPH_SYS), reached as a syscon.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/dmapool.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of_dma.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/slab.h>

#include "dmaengine.h"
#include "virt-dma.h"

#define PDMA_CHN_EN			0x000
#define PDMA_CTRL			0x004
#define  PDMA_CTRL_SUSPEND(ch)		BIT(ch)
#define  PDMA_CTRL_SUSPEND_ALL		BIT(16)
#define  PDMA_CTRL_LLI_PREFETCH		BIT(17)
#define PDMA_START			0x00c
#define PDMA_STA			0x010
#define  PDMA_STA_SUSPENDED(ch)		BIT(ch)
#define  PDMA_STA_ALL_SUSPENDED		BIT(19)
#define PDMA_CHN_LLI_H(ch)		(0x020 + (ch) * 8)
#define PDMA_CHN_LLI_L(ch)		(0x024 + (ch) * 8)
#define PDMA_CLEAR			0x0a0
#define PDMA_INT_GLB_MASK		0x0d8
#define PDMA_INT_GLB_STA		0x0e0
#define  PDMA_INT_GLB_CHANNELS		GENMASK(15, 0)
#define  PDMA_INT_GLB_RESP_ERR		BIT(16)
#define PDMA_INT_RESP_ERR_CLR		0x0e8
#define PDMA_INT_RESP_ERR_STA		0x0f0
#define PDMA_INT_MASK(ch)		(0x0f4 + (ch) * 0x10)
#define PDMA_INT_CLR(ch)		(0x0f8 + (ch) * 0x10)
#define PDMA_INT_STA(ch)		(0x100 + (ch) * 0x10)
#define  PDMA_INT_TRANSFER_DONE		BIT(0)
#define  PDMA_INT_BLOCK_DONE		BIT(1)
#define  PDMA_INT_ERRORS		GENMASK(5, 3)
#define  PDMA_INT_ALL			GENMASK(5, 0)
#define PDMA_SRC_ADDR_H(ch)		(0x1f4 + (ch) * 0x10)
#define PDMA_DST_ADDR_H(ch)		(0x1fc + (ch) * 0x10)

/* PERIPH_SYS, SET and CLR aliases: the request each channel follows. */
#define PERIPH_REQ_SEL_H_SET		0x130
#define PERIPH_REQ_SEL_H_CLR		0x134
#define PERIPH_REQ_SEL_L_SET		0x138
#define PERIPH_REQ_SEL_L_CLR		0x13c
#define PERIPH_REQ_HS_SET(ch)		(0x158 - (ch) / 5 * 8)
#define PERIPH_REQ_HS_CLR(ch)		(0x15c - (ch) / 5 * 8)
#define PERIPH_REQ_HS_SHIFT(ch)		((ch) % 5 * 6)
#define PDMA_REQ_NONE			0x3f

#define LLI_CTRL_LAST			BIT_ULL(1)
#define LLI_CTRL_IOC			BIT_ULL(2)
#define LLI_CTRL_SRC_PER		BIT_ULL(5)
#define LLI_CTRL_AWLEN			GENMASK_ULL(7, 6)
#define LLI_CTRL_ARLEN			GENMASK_ULL(9, 8)
#define LLI_CTRL_DST_WIDTH		GENMASK_ULL(26, 24)
#define LLI_CTRL_SRC_WIDTH		GENMASK_ULL(29, 27)
#define LLI_CTRL_DINC			BIT_ULL(30)
#define LLI_CTRL_SINC			BIT_ULL(31)
#define LLI_CTRL_DST_OSD		GENMASK_ULL(35, 32)
#define LLI_CTRL_SRC_OSD		GENMASK_ULL(39, 36)
#define LLI_CTRL_DST_MSIZE		GENMASK_ULL(51, 42)
#define LLI_CTRL_SRC_MSIZE		GENMASK_ULL(61, 52)
#define LLI_DATA_LLP			GENMASK_ULL(36, 0)
#define LLI_DATA_BLOCK_TS		GENMASK_ULL(58, 37)

#define PDMA_CHANNELS			16
#define PDMA_MAX_BLOCK_TS		0x3ffff0
#define PDMA_MAX_BURST			1023
#define PDMA_LLI_ALIGN			64
#define PDMA_BUSWIDTHS			(BIT(DMA_SLAVE_BUSWIDTH_1_BYTE) | \
					 BIT(DMA_SLAVE_BUSWIDTH_2_BYTES) | \
					 BIT(DMA_SLAVE_BUSWIDTH_4_BYTES) | \
					 BIT(DMA_SLAVE_BUSWIDTH_8_BYTES))

struct ax620e_pdma_lli {
	__le64 sar;
	__le64 dar;
	__le64 ctrl;
	__le64 data;
};

struct ax620e_pdma_block {
	struct ax620e_pdma_lli *lli;
	dma_addr_t phys;
	dma_addr_t mem;
	u32 len;
};

struct ax620e_pdma_desc {
	struct virt_dma_desc vd;
	struct dma_pool *pool;
	enum dma_transfer_direction dir;
	bool cyclic;
	size_t len;
	unsigned int count;
	struct ax620e_pdma_block blocks[] __counted_by(count);
};

struct ax620e_pdma;

struct ax620e_pdma_chan {
	struct virt_dma_chan vc;
	struct ax620e_pdma *pdma;
	unsigned int id;
	unsigned int request;
	struct dma_slave_config config;
	/* The running descriptor, protected by vc.lock. */
	struct ax620e_pdma_desc *desc;
};

struct ax620e_pdma {
	struct dma_device dma;
	struct device *dev;
	void __iomem *base;
	struct regmap *periph;
	struct dma_pool *pool;
	/* Read-modify-writes of the registers the channels share. */
	spinlock_t lock;
	struct ax620e_pdma_chan chans[PDMA_CHANNELS];
};

static struct ax620e_pdma_chan *to_pdma_chan(struct dma_chan *c)
{
	return container_of(c, struct ax620e_pdma_chan, vc.chan);
}

static struct ax620e_pdma_desc *to_pdma_desc(struct virt_dma_desc *vd)
{
	return container_of(vd, struct ax620e_pdma_desc, vd);
}

static void ax620e_pdma_update(struct ax620e_pdma *pdma, u32 reg, u32 mask,
			       u32 val)
{
	unsigned long flags;
	u32 tmp;

	spin_lock_irqsave(&pdma->lock, flags);
	tmp = readl(pdma->base + reg);
	writel((tmp & ~mask) | (val & mask), pdma->base + reg);
	spin_unlock_irqrestore(&pdma->lock, flags);
}

static u64 ax620e_pdma_read64(struct ax620e_pdma *pdma, u32 reg)
{
	u32 hi = readl(pdma->base + reg);

	return (u64)hi << 32 | readl(pdma->base + reg + 4);
}

static void ax620e_pdma_route(struct ax620e_pdma_chan *chan,
			      unsigned int request)
{
	struct regmap *map = chan->pdma->periph;
	unsigned int shift = PERIPH_REQ_HS_SHIFT(chan->id);

	regmap_write(map, request >= 32 ? PERIPH_REQ_SEL_H_SET :
		     PERIPH_REQ_SEL_L_SET, BIT(request % 32));
	regmap_write(map, PERIPH_REQ_HS_CLR(chan->id), PDMA_REQ_NONE << shift);
	regmap_write(map, PERIPH_REQ_HS_SET(chan->id), request << shift);
}

static void ax620e_pdma_unroute(struct ax620e_pdma_chan *chan)
{
	struct regmap *map = chan->pdma->periph;

	if (chan->request != PDMA_REQ_NONE)
		regmap_write(map, chan->request >= 32 ? PERIPH_REQ_SEL_H_CLR :
			     PERIPH_REQ_SEL_L_CLR, BIT(chan->request % 32));
	regmap_write(map, PERIPH_REQ_HS_SET(chan->id),
		     PDMA_REQ_NONE << PERIPH_REQ_HS_SHIFT(chan->id));
}

/* Called with vc.lock held. */
static void ax620e_pdma_start_next(struct ax620e_pdma_chan *chan)
{
	struct ax620e_pdma *pdma = chan->pdma;
	struct virt_dma_desc *vd = vchan_next_desc(&chan->vc);
	unsigned int id = chan->id;
	dma_addr_t first;

	chan->desc = vd ? to_pdma_desc(vd) : NULL;
	if (!chan->desc)
		return;

	first = chan->desc->blocks[0].phys;
	ax620e_pdma_route(chan, chan->request);
	writel(upper_32_bits(first), pdma->base + PDMA_CHN_LLI_H(id));
	writel(lower_32_bits(first), pdma->base + PDMA_CHN_LLI_L(id));
	ax620e_pdma_update(pdma, PDMA_CHN_EN, BIT(id), BIT(id));
	writel(PDMA_INT_ALL, pdma->base + PDMA_INT_MASK(id));
	ax620e_pdma_update(pdma, PDMA_INT_GLB_MASK, BIT(id), BIT(id));
	writel(BIT(id), pdma->base + PDMA_START);
}

static int ax620e_pdma_suspend_chan(struct ax620e_pdma_chan *chan)
{
	struct ax620e_pdma *pdma = chan->pdma;
	u32 sta;

	ax620e_pdma_update(pdma, PDMA_CTRL, PDMA_CTRL_SUSPEND(chan->id),
			   PDMA_CTRL_SUSPEND(chan->id));

	return readl_poll_timeout_atomic(pdma->base + PDMA_STA, sta,
					 sta & PDMA_STA_SUSPENDED(chan->id),
					 1, 20);
}

/* Called with vc.lock held. */
static void ax620e_pdma_stop(struct ax620e_pdma_chan *chan)
{
	struct ax620e_pdma *pdma = chan->pdma;
	unsigned int id = chan->id;

	if (ax620e_pdma_suspend_chan(chan))
		dev_dbg(pdma->dev, "channel %u did not suspend\n", id);
	ax620e_pdma_update(pdma, PDMA_CLEAR, BIT(id), BIT(id));
	ax620e_pdma_update(pdma, PDMA_CTRL, PDMA_CTRL_SUSPEND(id), 0);
	ax620e_pdma_update(pdma, PDMA_CHN_EN, BIT(id), 0);
	ax620e_pdma_update(pdma, PDMA_INT_GLB_MASK, BIT(id), 0);
	writel(0, pdma->base + PDMA_INT_MASK(id));
	writel(PDMA_INT_ALL, pdma->base + PDMA_INT_CLR(id));
}

static void ax620e_pdma_desc_free(struct virt_dma_desc *vd)
{
	struct ax620e_pdma_desc *desc = to_pdma_desc(vd);
	unsigned int i;

	for (i = 0; i < desc->count; i++)
		if (desc->blocks[i].lli)
			dma_pool_free(desc->pool, desc->blocks[i].lli,
				      desc->blocks[i].phys);
	kfree(desc);
}

/*
 * One descriptor: the device side moves at the configured width and burst,
 * the memory side as wide as the address, the length and the burst allow.
 */
static int ax620e_pdma_fill(struct ax620e_pdma_chan *chan,
			    struct ax620e_pdma_desc *desc, unsigned int i,
			    dma_addr_t mem, u32 len, bool ioc)
{
	struct dma_slave_config *cfg = &chan->config;
	struct ax620e_pdma_block *block = &desc->blocks[i];
	bool to_mem = desc->dir == DMA_DEV_TO_MEM;
	u32 width = to_mem ? cfg->src_addr_width : cfg->dst_addr_width;
	u32 burst = to_mem ? cfg->src_maxburst : cfg->dst_maxburst;
	u32 dev_log2, mem_log2, burst_bytes, src_log2;
	struct ax620e_pdma_lli *lli;
	u64 ctrl;

	if (!is_power_of_2(width) || width > DMA_SLAVE_BUSWIDTH_8_BYTES ||
	    !len || len % width)
		return -EINVAL;

	burst = clamp(burst, 1U, PDMA_MAX_BURST);
	dev_log2 = __ffs(width);
	burst_bytes = burst << dev_log2;
	mem_log2 = __ffs(len | lower_32_bits(mem) | 8 | burst_bytes);
	src_log2 = to_mem ? dev_log2 : mem_log2;
	if (len >> src_log2 > PDMA_MAX_BLOCK_TS)
		return -EINVAL;

	lli = dma_pool_zalloc(desc->pool, GFP_NOWAIT, &block->phys);
	if (!lli)
		return -ENOMEM;
	block->lli = lli;
	block->mem = mem;
	block->len = len;

	ctrl = FIELD_PREP(LLI_CTRL_DST_OSD, 0xf) |
	       FIELD_PREP(LLI_CTRL_SRC_OSD, 0xf);
	if (ioc)
		ctrl |= LLI_CTRL_IOC;

	if (to_mem) {
		lli->sar = cpu_to_le64(cfg->src_addr);
		lli->dar = cpu_to_le64(mem);
		ctrl |= LLI_CTRL_SRC_PER | LLI_CTRL_DINC |
			FIELD_PREP(LLI_CTRL_ARLEN, 1) |
			FIELD_PREP(LLI_CTRL_AWLEN, 2) |
			FIELD_PREP(LLI_CTRL_SRC_WIDTH, dev_log2) |
			FIELD_PREP(LLI_CTRL_DST_WIDTH, mem_log2) |
			FIELD_PREP(LLI_CTRL_SRC_MSIZE, burst) |
			FIELD_PREP(LLI_CTRL_DST_MSIZE, burst_bytes >> mem_log2);
	} else {
		lli->sar = cpu_to_le64(mem);
		lli->dar = cpu_to_le64(cfg->dst_addr);
		ctrl |= LLI_CTRL_SINC |
			FIELD_PREP(LLI_CTRL_ARLEN, 2) |
			FIELD_PREP(LLI_CTRL_AWLEN, 1) |
			FIELD_PREP(LLI_CTRL_SRC_WIDTH, mem_log2) |
			FIELD_PREP(LLI_CTRL_DST_WIDTH, dev_log2) |
			FIELD_PREP(LLI_CTRL_SRC_MSIZE, burst_bytes >> mem_log2) |
			FIELD_PREP(LLI_CTRL_DST_MSIZE, burst);
	}
	lli->ctrl = cpu_to_le64(ctrl);
	lli->data = cpu_to_le64(FIELD_PREP(LLI_DATA_BLOCK_TS, len >> src_log2));

	return 0;
}

/* Chains the blocks, back to the first for a cyclic transfer. */
static void ax620e_pdma_link(struct ax620e_pdma_desc *desc)
{
	unsigned int i;

	for (i = 0; i < desc->count; i++) {
		struct ax620e_pdma_lli *lli = desc->blocks[i].lli;
		bool last = i + 1 == desc->count;
		dma_addr_t next = desc->blocks[last ? 0 : i + 1].phys;
		u64 data = le64_to_cpu(lli->data);

		if (last && !desc->cyclic) {
			lli->ctrl |= cpu_to_le64(LLI_CTRL_LAST | LLI_CTRL_IOC);
			next = 0;
		}
		data |= FIELD_PREP(LLI_DATA_LLP, next);
		lli->data = cpu_to_le64(data);
	}
}

static struct ax620e_pdma_desc *
ax620e_pdma_alloc_desc(struct ax620e_pdma_chan *chan, unsigned int count,
		       enum dma_transfer_direction dir, bool cyclic)
{
	struct ax620e_pdma_desc *desc;

	if (!is_slave_direction(dir) || !count)
		return NULL;

	desc = kzalloc(struct_size(desc, blocks, count), GFP_NOWAIT);
	if (!desc)
		return NULL;
	desc->count = count;
	desc->pool = chan->pdma->pool;
	desc->dir = dir;
	desc->cyclic = cyclic;

	return desc;
}

static struct dma_async_tx_descriptor *
ax620e_pdma_prep_slave_sg(struct dma_chan *c, struct scatterlist *sgl,
			  unsigned int sg_len, enum dma_transfer_direction dir,
			  unsigned long flags, void *context)
{
	struct ax620e_pdma_chan *chan = to_pdma_chan(c);
	struct ax620e_pdma_desc *desc;
	struct scatterlist *sg;
	unsigned int i;

	desc = ax620e_pdma_alloc_desc(chan, sg_len, dir, false);
	if (!desc)
		return NULL;

	for_each_sg(sgl, sg, sg_len, i) {
		if (ax620e_pdma_fill(chan, desc, i, sg_dma_address(sg),
				     sg_dma_len(sg), false))
			goto err;
		desc->len += sg_dma_len(sg);
	}
	ax620e_pdma_link(desc);

	return vchan_tx_prep(&chan->vc, &desc->vd, flags);
err:
	ax620e_pdma_desc_free(&desc->vd);
	return NULL;
}

static struct dma_async_tx_descriptor *
ax620e_pdma_prep_cyclic(struct dma_chan *c, dma_addr_t buf, size_t buf_len,
			size_t period_len, enum dma_transfer_direction dir,
			unsigned long flags)
{
	struct ax620e_pdma_chan *chan = to_pdma_chan(c);
	struct ax620e_pdma_desc *desc;
	unsigned int i, periods;

	if (!period_len || buf_len % period_len)
		return NULL;
	periods = buf_len / period_len;

	desc = ax620e_pdma_alloc_desc(chan, periods, dir, true);
	if (!desc)
		return NULL;

	for (i = 0; i < periods; i++)
		if (ax620e_pdma_fill(chan, desc, i, buf + i * period_len,
				     period_len, true))
			goto err;
	desc->len = buf_len;
	ax620e_pdma_link(desc);

	return vchan_tx_prep(&chan->vc, &desc->vd, flags);
err:
	ax620e_pdma_desc_free(&desc->vd);
	return NULL;
}

static int ax620e_pdma_config(struct dma_chan *c, struct dma_slave_config *cfg)
{
	struct ax620e_pdma_chan *chan = to_pdma_chan(c);

	chan->config = *cfg;

	return 0;
}

static void ax620e_pdma_issue_pending(struct dma_chan *c)
{
	struct ax620e_pdma_chan *chan = to_pdma_chan(c);
	unsigned long flags;

	spin_lock_irqsave(&chan->vc.lock, flags);
	if (vchan_issue_pending(&chan->vc) && !chan->desc)
		ax620e_pdma_start_next(chan);
	spin_unlock_irqrestore(&chan->vc.lock, flags);
}

/* Bytes left of the running descriptor, from the channel's address. */
static size_t ax620e_pdma_residue(struct ax620e_pdma_chan *chan,
				  struct ax620e_pdma_desc *desc)
{
	u32 reg = desc->dir == DMA_DEV_TO_MEM ? PDMA_DST_ADDR_H(chan->id) :
						PDMA_SRC_ADDR_H(chan->id);
	u64 addr = ax620e_pdma_read64(chan->pdma, reg);
	size_t done = 0;
	unsigned int i;

	for (i = 0; i < desc->count; i++) {
		struct ax620e_pdma_block *block = &desc->blocks[i];

		if (addr >= block->mem && addr < block->mem + block->len)
			return desc->len - done - (addr - block->mem);
		done += block->len;
	}

	return desc->len;
}

static enum dma_status ax620e_pdma_tx_status(struct dma_chan *c,
					     dma_cookie_t cookie,
					     struct dma_tx_state *state)
{
	struct ax620e_pdma_chan *chan = to_pdma_chan(c);
	struct virt_dma_desc *vd;
	enum dma_status status;
	unsigned long flags;
	size_t residue = 0;

	status = dma_cookie_status(c, cookie, state);
	if (status == DMA_COMPLETE || !state)
		return status;

	spin_lock_irqsave(&chan->vc.lock, flags);
	vd = vchan_find_desc(&chan->vc, cookie);
	if (chan->desc && chan->desc->vd.tx.cookie == cookie)
		residue = ax620e_pdma_residue(chan, chan->desc);
	else if (vd)
		residue = to_pdma_desc(vd)->len;
	spin_unlock_irqrestore(&chan->vc.lock, flags);

	dma_set_residue(state, residue);

	return status;
}

static int ax620e_pdma_pause(struct dma_chan *c)
{
	struct ax620e_pdma_chan *chan = to_pdma_chan(c);
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&chan->vc.lock, flags);
	ret = ax620e_pdma_suspend_chan(chan);
	spin_unlock_irqrestore(&chan->vc.lock, flags);

	return ret ? -EAGAIN : 0;
}

static int ax620e_pdma_resume(struct dma_chan *c)
{
	struct ax620e_pdma_chan *chan = to_pdma_chan(c);

	ax620e_pdma_update(chan->pdma, PDMA_CTRL, PDMA_CTRL_SUSPEND(chan->id), 0);

	return 0;
}

static int ax620e_pdma_terminate_all(struct dma_chan *c)
{
	struct ax620e_pdma_chan *chan = to_pdma_chan(c);
	unsigned long flags;
	LIST_HEAD(head);

	spin_lock_irqsave(&chan->vc.lock, flags);
	ax620e_pdma_stop(chan);
	if (chan->desc) {
		/* Freed by synchronize, after a callback that may be running. */
		list_del(&chan->desc->vd.node);
		vchan_terminate_vdesc(&chan->desc->vd);
		chan->desc = NULL;
	}
	vchan_get_all_descriptors(&chan->vc, &head);
	spin_unlock_irqrestore(&chan->vc.lock, flags);

	vchan_dma_desc_free_list(&chan->vc, &head);

	return 0;
}

static void ax620e_pdma_synchronize(struct dma_chan *c)
{
	vchan_synchronize(&to_pdma_chan(c)->vc);
}

static void ax620e_pdma_free_chan_resources(struct dma_chan *c)
{
	struct ax620e_pdma_chan *chan = to_pdma_chan(c);

	ax620e_pdma_terminate_all(c);
	ax620e_pdma_unroute(chan);
	vchan_free_chan_resources(&chan->vc);
	chan->request = PDMA_REQ_NONE;
}

static void ax620e_pdma_chan_irq(struct ax620e_pdma_chan *chan, u32 status)
{
	struct ax620e_pdma_desc *desc;

	spin_lock(&chan->vc.lock);
	desc = chan->desc;
	if (!desc)
		goto out;

	if (status & PDMA_INT_ERRORS)
		dev_err_ratelimited(chan->pdma->dev,
				    "channel %u: transfer error %#x\n",
				    chan->id, status);

	if (desc->cyclic) {
		if (status & (PDMA_INT_BLOCK_DONE | PDMA_INT_TRANSFER_DONE))
			vchan_cyclic_callback(&desc->vd);
	} else if (status & (PDMA_INT_TRANSFER_DONE | PDMA_INT_ERRORS)) {
		ax620e_pdma_update(chan->pdma, PDMA_CHN_EN, BIT(chan->id), 0);
		if (status & PDMA_INT_ERRORS)
			desc->vd.tx_result.result = DMA_TRANS_ABORTED;
		list_del(&desc->vd.node);
		vchan_cookie_complete(&desc->vd);
		ax620e_pdma_start_next(chan);
	}
out:
	spin_unlock(&chan->vc.lock);
}

static irqreturn_t ax620e_pdma_irq(int irq, void *data)
{
	struct ax620e_pdma *pdma = data;
	u32 pending = readl(pdma->base + PDMA_INT_GLB_STA);
	unsigned long channels = pending & PDMA_INT_GLB_CHANNELS;
	unsigned int id;

	if (!pending)
		return IRQ_NONE;

	if (pending & PDMA_INT_GLB_RESP_ERR) {
		u32 err = readl(pdma->base + PDMA_INT_RESP_ERR_STA);

		writel(err, pdma->base + PDMA_INT_RESP_ERR_CLR);
		dev_err_ratelimited(pdma->dev, "bus error %#x\n", err);
	}

	for_each_set_bit(id, &channels, PDMA_CHANNELS) {
		struct ax620e_pdma_chan *chan = &pdma->chans[id];
		u32 status = readl(pdma->base + PDMA_INT_STA(id));

		writel(status, pdma->base + PDMA_INT_CLR(id));
		ax620e_pdma_chan_irq(chan, status);
	}

	return IRQ_HANDLED;
}

static struct dma_chan *ax620e_pdma_xlate(struct of_phandle_args *spec,
					  struct of_dma *ofdma)
{
	struct ax620e_pdma *pdma = ofdma->of_dma_data;
	struct dma_chan *c;

	if (spec->args_count != 1 || spec->args[0] >= PDMA_REQ_NONE)
		return NULL;

	c = dma_get_any_slave_channel(&pdma->dma);
	if (c)
		to_pdma_chan(c)->request = spec->args[0];

	return c;
}

static void ax620e_pdma_of_free(void *data)
{
	of_dma_controller_free(data);
}

static int ax620e_pdma_init_hw(struct ax620e_pdma *pdma)
{
	unsigned int id;
	u32 sta;

	for (id = 0; id < PDMA_CHANNELS; id++)
		ax620e_pdma_unroute(&pdma->chans[id]);

	ax620e_pdma_update(pdma, PDMA_CTRL, PDMA_CTRL_LLI_PREFETCH, 0);
	writel(PDMA_INT_GLB_RESP_ERR, pdma->base + PDMA_INT_GLB_MASK);
	ax620e_pdma_update(pdma, PDMA_CTRL, PDMA_CTRL_SUSPEND_ALL, 0);

	return readl_poll_timeout(pdma->base + PDMA_STA, sta,
				  !(sta & PDMA_STA_ALL_SUSPENDED), 100, 10000);
}

static int ax620e_pdma_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct reset_control *rst;
	struct clk_bulk_data *clks;
	struct ax620e_pdma *pdma;
	struct dma_device *dd;
	unsigned int id;
	int irq, ret;

	pdma = devm_kzalloc(dev, sizeof(*pdma), GFP_KERNEL);
	if (!pdma)
		return -ENOMEM;
	pdma->dev = dev;
	spin_lock_init(&pdma->lock);

	pdma->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(pdma->base))
		return PTR_ERR(pdma->base);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	pdma->periph = syscon_regmap_lookup_by_phandle(dev->of_node,
						       "axera,periph-sys");
	if (IS_ERR(pdma->periph))
		return dev_err_probe(dev, PTR_ERR(pdma->periph),
				     "no PERIPH_SYS syscon\n");

	ret = devm_clk_bulk_get_all_enabled(dev, &clks);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to enable the clocks\n");

	rst = devm_reset_control_get_exclusive_deasserted(dev, "axi");
	if (IS_ERR(rst))
		return dev_err_probe(dev, PTR_ERR(rst), "failed to deassert axi\n");
	rst = devm_reset_control_get_exclusive_deasserted(dev, "apb");
	if (IS_ERR(rst))
		return dev_err_probe(dev, PTR_ERR(rst), "failed to deassert apb\n");

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	pdma->pool = dmam_pool_create(dev_name(dev), dev,
				      sizeof(struct ax620e_pdma_lli),
				      PDMA_LLI_ALIGN, 0);
	if (!pdma->pool)
		return -ENOMEM;

	dd = &pdma->dma;
	INIT_LIST_HEAD(&dd->channels);
	for (id = 0; id < PDMA_CHANNELS; id++) {
		struct ax620e_pdma_chan *chan = &pdma->chans[id];

		chan->pdma = pdma;
		chan->id = id;
		chan->request = PDMA_REQ_NONE;
		chan->vc.desc_free = ax620e_pdma_desc_free;
		vchan_init(&chan->vc, dd);
	}

	ret = ax620e_pdma_init_hw(pdma);
	if (ret)
		return dev_err_probe(dev, ret, "the controller stays suspended\n");

	ret = devm_request_irq(dev, irq, ax620e_pdma_irq, 0, dev_name(dev),
			       pdma);
	if (ret)
		return ret;

	dma_cap_set(DMA_SLAVE, dd->cap_mask);
	dma_cap_set(DMA_CYCLIC, dd->cap_mask);
	dd->dev = dev;
	dd->src_addr_widths = PDMA_BUSWIDTHS;
	dd->dst_addr_widths = PDMA_BUSWIDTHS;
	dd->directions = BIT(DMA_DEV_TO_MEM) | BIT(DMA_MEM_TO_DEV);
	dd->residue_granularity = DMA_RESIDUE_GRANULARITY_BURST;
	dd->max_burst = PDMA_MAX_BURST;
	dd->device_free_chan_resources = ax620e_pdma_free_chan_resources;
	dd->device_prep_slave_sg = ax620e_pdma_prep_slave_sg;
	dd->device_prep_dma_cyclic = ax620e_pdma_prep_cyclic;
	dd->device_config = ax620e_pdma_config;
	dd->device_pause = ax620e_pdma_pause;
	dd->device_resume = ax620e_pdma_resume;
	dd->device_terminate_all = ax620e_pdma_terminate_all;
	dd->device_synchronize = ax620e_pdma_synchronize;
	dd->device_tx_status = ax620e_pdma_tx_status;
	dd->device_issue_pending = ax620e_pdma_issue_pending;

	ret = dmaenginem_async_device_register(dd);
	if (ret)
		return ret;

	ret = of_dma_controller_register(dev->of_node, ax620e_pdma_xlate, pdma);
	if (ret)
		return ret;

	return devm_add_action_or_reset(dev, ax620e_pdma_of_free, dev->of_node);
}

static const struct of_device_id ax620e_pdma_of_match[] = {
	{ .compatible = "axera,ax620e-pdma" },
	{ }
};
MODULE_DEVICE_TABLE(of, ax620e_pdma_of_match);

static struct platform_driver ax620e_pdma_driver = {
	.probe = ax620e_pdma_probe,
	.driver = {
		.name = "ax620e-pdma",
		.of_match_table = ax620e_pdma_of_match,
	},
};
module_platform_driver(ax620e_pdma_driver);

MODULE_DESCRIPTION("Axera AX620E peripheral DMA controller driver");
MODULE_LICENSE("GPL");
