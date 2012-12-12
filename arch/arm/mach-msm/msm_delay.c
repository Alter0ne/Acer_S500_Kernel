/*
 *  Copyright (C) 2012 Acer Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/io.h>
#include <mach/msm_iomap.h>

void __mdelay(unsigned long msecs)
{
	unsigned long t, timeout;
	void __iomem *addr = MSM_TMR0_BASE + 0x28;

	msecs = msecs * 6750;
	timeout = __raw_readl_no_log(addr) + msecs;
	do {
		t = __raw_readl_no_log(addr);
	} while (time_before(t, timeout));
}
EXPORT_SYMBOL(__mdelay);
