// SPDX-License-Identifier: GPL-2.0
/*
 * Hantro VPU codec driver: baseline JPEG encoding on VeriSilicon VC8000E
 *
 * The register file is the one of the full VC8000E encoder; JPEG mode only
 * uses a small part of it. Values marked as vendor defaults come from the
 * Axera AX620E vendor stack, their meaning is unknown.
 */

#include <linux/bitfield.h>
#include <linux/reset.h>
#include <linux/unaligned.h>
#include <media/v4l2-mem2mem.h>
#include "hantro_jpeg.h"
#include "hantro.h"
#include "hantro_v4l2.h"
#include "hantro_hw.h"

#define VC8000E_REG(n)				((n) * 4)
#define VC8000E_NUM_REGS			512

#define VC8000E_REG_INTERRUPT			VC8000E_REG(1)
#define   VC8000E_INT_IRQ			BIT(0)
#define   VC8000E_INT_IRQ_DIS			BIT(1)
#define   VC8000E_INT_FRAME_RDY			BIT(2)
#define   VC8000E_INT_STATUS_ALL		(BIT(0) | GENMASK(8, 2))
#define   VC8000E_INT_TIMEOUT_EN		BIT(11)
#define VC8000E_REG_SWAP			VC8000E_REG(2)
#define VC8000E_REG_AXI_CLK_GATE		VC8000E_REG(3)
#define VC8000E_REG_MODE			VC8000E_REG(4)
#define   VC8000E_MODE				GENMASK(31, 29)
#define   VC8000E_MODE_JPEG			4
#define VC8000E_REG_ENC_CTRL			VC8000E_REG(5)
#define   VC8000E_ENC_CTRL_EN			BIT(0)
#define   VC8000E_ENC_CTRL_JPEG_HEIGHT		GENMASK(19, 8)
#define   VC8000E_ENC_CTRL_JPEG_WIDTH		GENMASK(31, 20)
#define VC8000E_REG_OUTPUT_STRM_BASE		VC8000E_REG(8)
#define VC8000E_REG_OUTPUT_STRM_LIMIT		VC8000E_REG(9)
#define VC8000E_REG_INPUT_Y_BASE		VC8000E_REG(12)
#define VC8000E_REG_JPEG_ROW			VC8000E_REG(20)
#define   VC8000E_JPEG_ROWLENGTH		GENMASK(31, 17)
#define VC8000E_REG_INPUT_FMT			VC8000E_REG(38)
#define   VC8000E_INPUT_FORMAT			GENMASK(31, 28)
#define VC8000E_REG_AXI_BURST			VC8000E_REG(81)
#define   VC8000E_MAX_BURST			GENMASK(31, 24)
#define VC8000E_REG_JPEG_LUMA_QUANT(i)		VC8000E_REG(83 + (i))
#define VC8000E_REG_JPEG_CHROMA_QUANT(i)	VC8000E_REG(99 + (i))
#define VC8000E_REG_SYNC			VC8000E_REG(196)
#define VC8000E_REG_INPUT_STRIDE		VC8000E_REG(210)
#define   VC8000E_INPUT_LU_STRIDE		GENMASK(31, 12)
#define VC8000E_REG_CONST_CHROMA		VC8000E_REG(224)
#define VC8000E_REG_AXI_WR_OUTSTANDING		VC8000E_REG(246)
#define VC8000E_REG_AXI_RD_OUTSTANDING		VC8000E_REG(261)
#define VC8000E_REG_URGENT			VC8000E_REG(272)
#define VC8000E_REG_BURST_ALIGN_FUSE		VC8000E_REG(319)
#define VC8000E_REG_BURST_ALIGN			VC8000E_REG(320)

#define VC8000E_JPEG_QUANT_TABLE_COUNT		16

static const struct {
	u32 reg;
	u32 value;
} vc8000e_jpeg_defaults[] = {
	{ VC8000E_REG_SWAP, 0x000000f0 },
	/* AXI read/write IDs and all internal clock gates enabled. */
	{ VC8000E_REG_AXI_CLK_GATE, 0xff000000 },
	{ VC8000E_REG_SYNC, 0x00100000 },
	/* Constant chroma value 128 (only used when enabled). */
	{ VC8000E_REG_CONST_CHROMA, 0x00020080 },
	/* 64 outstanding AXI writes and reads. */
	{ VC8000E_REG_AXI_WR_OUTSTANDING, 0x00001000 },
	{ VC8000E_REG_AXI_RD_OUTSTANDING, 0x00000400 },
	/* Urgent signalling disabled. */
	{ VC8000E_REG_URGENT, 0xffffffff },
	{ VC8000E_REG_BURST_ALIGN_FUSE, 0x00004000 },
	{ VC8000E_REG_BURST_ALIGN, 0x00004000 },
	/* Vendor defaults. */
	{ VC8000E_REG(349), 0x80000000 },
	{ VC8000E_REG(430), 0x20000000 },
};

static void vc8000e_jpeg_enc_set_qtable(struct hantro_dev *vpu,
					unsigned char *luma_qtable,
					unsigned char *chroma_qtable)
{
	unsigned int i;

	for (i = 0; i < VC8000E_JPEG_QUANT_TABLE_COUNT; i++)
		vepu_write_relaxed(vpu, get_unaligned_be32(&luma_qtable[4 * i]),
				   VC8000E_REG_JPEG_LUMA_QUANT(i));
	for (i = 0; i < VC8000E_JPEG_QUANT_TABLE_COUNT; i++)
		vepu_write_relaxed(vpu, get_unaligned_be32(&chroma_qtable[4 * i]),
				   VC8000E_REG_JPEG_CHROMA_QUANT(i));
}

int hantro_vc8000e_jpeg_enc_run(struct hantro_ctx *ctx)
{
	struct hantro_dev *vpu = ctx->dev;
	struct v4l2_pix_format_mplane *src_fmt = &ctx->src_fmt;
	unsigned int header_size = ctx->vpu_dst_fmt->header_size;
	struct vb2_v4l2_buffer *src_buf, *dst_buf;
	struct hantro_jpeg_ctx jpeg_ctx;
	u32 row_pixels, size_left;
	dma_addr_t dst;
	unsigned int i;

	src_buf = hantro_get_src_buf(ctx);
	dst_buf = hantro_get_dst_buf(ctx);

	hantro_start_prepare_run(ctx);

	memset(&jpeg_ctx, 0, sizeof(jpeg_ctx));
	jpeg_ctx.buffer = vb2_plane_vaddr(&dst_buf->vb2_buf, 0);
	jpeg_ctx.width = ctx->dst_fmt.width;
	jpeg_ctx.height = ctx->dst_fmt.height;
	jpeg_ctx.quality = ctx->jpeg_quality;
	hantro_jpeg_header_assemble(&jpeg_ctx);

	/* Start from a clean register file, as the vendor stack does. */
	for (i = 2; i < VC8000E_NUM_REGS; i++)
		vepu_write_relaxed(vpu, 0, VC8000E_REG(i));
	vepu_write_relaxed(vpu, VC8000E_INT_STATUS_ALL | VC8000E_INT_TIMEOUT_EN,
			   VC8000E_REG_INTERRUPT);
	vepu_write_relaxed(vpu, FIELD_PREP(VC8000E_MODE, VC8000E_MODE_JPEG),
			   VC8000E_REG_MODE);
	for (i = 0; i < ARRAY_SIZE(vc8000e_jpeg_defaults); i++)
		vepu_write_relaxed(vpu, vc8000e_jpeg_defaults[i].value,
				   vc8000e_jpeg_defaults[i].reg);

	dst = vb2_dma_contig_plane_dma_addr(&dst_buf->vb2_buf, 0);
	size_left = vb2_plane_size(&dst_buf->vb2_buf, 0) - header_size;
	vepu_write_relaxed(vpu, dst + header_size, VC8000E_REG_OUTPUT_STRM_BASE);
	vepu_write_relaxed(vpu, size_left, VC8000E_REG_OUTPUT_STRM_LIMIT);
	vepu_write_relaxed(vpu,
			   vb2_dma_contig_plane_dma_addr(&src_buf->vb2_buf, 0),
			   VC8000E_REG_INPUT_Y_BASE);

	/* Only packed 4:2:2 input: two bytes per pixel. */
	row_pixels = src_fmt->plane_fmt[0].bytesperline / 2;
	vepu_write_relaxed(vpu, FIELD_PREP(VC8000E_JPEG_ROWLENGTH, row_pixels),
			   VC8000E_REG_JPEG_ROW);
	vepu_write_relaxed(vpu, FIELD_PREP(VC8000E_INPUT_LU_STRIDE, row_pixels),
			   VC8000E_REG_INPUT_STRIDE);
	vepu_write_relaxed(vpu, FIELD_PREP(VC8000E_INPUT_FORMAT,
					   ctx->vpu_src_fmt->enc_fmt),
			   VC8000E_REG_INPUT_FMT);
	vepu_write_relaxed(vpu, FIELD_PREP(VC8000E_MAX_BURST, 16),
			   VC8000E_REG_AXI_BURST);

	vc8000e_jpeg_enc_set_qtable(vpu, jpeg_ctx.hw_luma_qtable,
				    jpeg_ctx.hw_chroma_qtable);

	hantro_end_prepare_run(ctx);

	vepu_write(vpu, FIELD_PREP(VC8000E_ENC_CTRL_JPEG_WIDTH,
				   src_fmt->width / 8) |
			FIELD_PREP(VC8000E_ENC_CTRL_JPEG_HEIGHT,
				   src_fmt->height / 8) |
			VC8000E_ENC_CTRL_EN,
		   VC8000E_REG_ENC_CTRL);

	return 0;
}

void hantro_vc8000e_jpeg_enc_done(struct hantro_ctx *ctx)
{
	struct hantro_dev *vpu = ctx->dev;
	/* Reads back as the number of bytes written. */
	u32 bytesused = vepu_read(vpu, VC8000E_REG_OUTPUT_STRM_LIMIT);
	struct vb2_v4l2_buffer *dst_buf = hantro_get_dst_buf(ctx);

	vb2_set_plane_payload(&dst_buf->vb2_buf, 0,
			      ctx->vpu_dst_fmt->header_size + bytesused);
}

irqreturn_t hantro_vc8000e_irq(int irq, void *dev_id)
{
	struct hantro_dev *vpu = dev_id;
	enum vb2_buffer_state state;
	u32 status;

	status = vepu_read(vpu, VC8000E_REG_INTERRUPT);
	if (!(status & VC8000E_INT_STATUS_ALL))
		return IRQ_NONE;

	state = (status & VC8000E_INT_FRAME_RDY) ?
		VB2_BUF_STATE_DONE : VB2_BUF_STATE_ERROR;
	vepu_write(vpu, status & VC8000E_INT_STATUS_ALL, VC8000E_REG_INTERRUPT);

	hantro_irq_done(vpu, state);

	return IRQ_HANDLED;
}

void hantro_vc8000e_reset(struct hantro_ctx *ctx)
{
	struct hantro_dev *vpu = ctx->dev;

	vepu_write(vpu, VC8000E_INT_IRQ_DIS | VC8000E_INT_STATUS_ALL,
		   VC8000E_REG_INTERRUPT);
	vepu_write(vpu, 0, VC8000E_REG_ENC_CTRL);
	reset_control_reset(vpu->resets);
}
