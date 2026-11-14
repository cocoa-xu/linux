// SPDX-License-Identifier: GPL-2.0
/*
 * Early console into a reserved RAM region. A kernel that hangs before its
 * real console is up leaves the log behind for the next boot to read, which
 * is all a board without a serial console can offer during bring-up.
 *
 *   earlycon=ramlog,<page-aligned phys> keep_bootcon
 *
 * Layout: struct ramlog_header, then a ring of text; head counts every byte
 * ever written, so the ring starts at head % size once it has wrapped.
 */

#include <linux/console.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/serial_core.h>
#include <asm/fixmap.h>

#define RAMLOG_MAGIC	0x474f4c52	/* "RLOG" */

struct ramlog_header {
	u32 magic;
	u32 size;
	u32 head;
	u32 boots;
};

static struct ramlog_header __iomem *ramlog;

static void ramlog_write(struct console *con, const char *s, unsigned int n)
{
	char __iomem *text = (char __iomem *)(ramlog + 1);
	u32 size = readl_relaxed(&ramlog->size);
	u32 head = readl_relaxed(&ramlog->head);

	for (; n; n--, head++)
		writeb_relaxed(*s++, text + head % size);
	writel(head, &ramlog->head);
}

static int __init ramlog_setup(struct earlycon_device *dev, const char *opt)
{
	phys_addr_t base = dev->port.mapbase;
	unsigned int i;
	u32 boots;

	if (!base || !PAGE_ALIGNED(base))
		return -EINVAL;

	for (i = 0; i < RAMLOG_PAGES; i++)
		__set_fixmap(FIX_RAMLOG_END - i, base + i * PAGE_SIZE, FIXMAP_PAGE_IO);
	ramlog = (void __iomem *)__fix_to_virt(FIX_RAMLOG_END);

	boots = readl_relaxed(&ramlog->magic) == RAMLOG_MAGIC ? readl_relaxed(&ramlog->boots) : 0;
	writel_relaxed(RAMLOG_PAGES * PAGE_SIZE - sizeof(*ramlog), &ramlog->size);
	writel_relaxed(0, &ramlog->head);
	writel_relaxed(boots + 1, &ramlog->boots);
	writel(RAMLOG_MAGIC, &ramlog->magic);

	dev->con->write = ramlog_write;
	return 0;
}
EARLYCON_DECLARE(ramlog, ramlog_setup);
