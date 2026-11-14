// SPDX-License-Identifier: GPL-2.0-only
/*
 * Axera AX620E SMP bring-up
 *
 * Secondary cores wait in the boot ROM until COMM_GLB holds their entry
 * point and an event is sent.
 */

#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/smp.h>

#include <asm/cacheflush.h>
#include <asm/smp_plat.h>

#define COMM_GLB_SECONDARY_ENTRY	0xdc

extern void ax620e_secondary_startup(void);

static void __iomem *comm_glb;

static int ax620e_boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	if (!comm_glb)
		return -ENODEV;

	writel(__pa_symbol(ax620e_secondary_startup),
	       comm_glb + COMM_GLB_SECONDARY_ENTRY);
	dsb_sev();

	return 0;
}

static void __init ax620e_smp_prepare_cpus(unsigned int max_cpus)
{
	struct device_node *np;

	np = of_find_compatible_node(NULL, NULL, "axera,ax620e-comm-glb");
	comm_glb = of_iomap(np, 0);
	of_node_put(np);
	if (!comm_glb)
		pr_err("ax620e: no COMM_GLB, secondary CPUs stay offline\n");
}

static const struct smp_operations ax620e_smp_ops __initconst = {
	.smp_prepare_cpus = ax620e_smp_prepare_cpus,
	.smp_boot_secondary = ax620e_boot_secondary,
};
CPU_METHOD_OF_DECLARE(ax620e_smp, "axera,ax620e-smp", &ax620e_smp_ops);
