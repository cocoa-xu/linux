/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Secondary CPU entry for Axera AX620E: the boot ROM releases the core here
 * with the MMU off. AArch32 kernels do not set CPUECTLR.SMPEN on Cortex-A53,
 * so join the coherency domain before secondary_startup turns caches on.
 */

#include <linux/linkage.h>
#include <asm/assembler.h>

ENTRY(ax620e_secondary_startup)
 ARM_BE8(setend	be)
	mrrc	p15, 1, r0, r1, c15		@ CPUECTLR
	orr	r0, r0, #(1 << 6)		@ SMPEN
	mcrr	p15, 1, r0, r1, c15
	isb
	b	secondary_startup
ENDPROC(ax620e_secondary_startup)
