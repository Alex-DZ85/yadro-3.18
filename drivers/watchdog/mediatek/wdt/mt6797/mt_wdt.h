/*
* Copyright (C) 2016 MediaTek Inc.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See http://www.gnu.org/licenses/gpl-2.0.html for more details.
*/


#ifndef __WDT_HW_H__
#define __WDT_HW_H__

#define MTK_WDT_BASE			toprgu_base

#define MTK_WDT_MODE			(MTK_WDT_BASE+0x0000)
#define MTK_WDT_LENGTH			(MTK_WDT_BASE+0x0004)
#define MTK_WDT_RESTART			(MTK_WDT_BASE+0x0008)
#define MTK_WDT_STATUS			(MTK_WDT_BASE+0x000C)
#define MTK_WDT_INTERVAL		(MTK_WDT_BASE+0x0010)
#define MTK_WDT_SWRST			(MTK_WDT_BASE+0x0014)
#define MTK_WDT_SWSYSRST		(MTK_WDT_BASE+0x0018)
#define MTK_WDT_NONRST_REG		(MTK_WDT_BASE+0x0020)
#define MTK_WDT_NONRST_REG2		(MTK_WDT_BASE+0x0024)
#define MTK_WDT_REQ_MODE		(MTK_WDT_BASE+0x0030)
#define MTK_WDT_REQ_IRQ_EN		(MTK_WDT_BASE+0x0034)
#define MTK_WDT_EXT_REQ_CON		(MTK_WDT_BASE+0x0038)
#define MTK_WDT_DRAMC_CTL		(MTK_WDT_BASE+0x0040)
#define MTK_WDT_LATCH_CTL		(MTK_WDT_BASE+0x0044)
#define MTK_WDT_DEBUG_2_REG		(MTK_WDT_BASE+0x0508)

#define MTK_WDT_RSTDEG_EN1		(MTK_WDT_BASE+0x0080)
#define MTK_WDT_RSTDEG_EN2		(MTK_WDT_BASE+0x0084)
#define MTK_WDT_SYSDBG_DEG_EN1		(MTK_WDT_BASE+0x0088)
#define MTK_WDT_SYSDBG_DEG_EN2		(MTK_WDT_BASE+0x008C)


/*WDT_MODE*/
#define MTK_WDT_MODE_KEYMASK		(0xff00)
#define MTK_WDT_MODE_KEY		(0x22000000)

#define MTK_WDT_MODE_DDR_RESERVE  (0x0080)
#define MTK_WDT_MODE_DUAL_MODE  (0x0040)
#define MTK_WDT_MODE_IN_DIS		(0x0020) /* Reserved */
#define MTK_WDT_MODE_AUTO_RESTART	(0x0010) /* Reserved */
#define MTK_WDT_MODE_IRQ		(0x0008)
#define MTK_WDT_MODE_EXTEN		(0x0004)
#define MTK_WDT_MODE_EXT_POL		(0x0002)
#define MTK_WDT_MODE_ENABLE		(0x0001)


/*WDT_LENGTH*/
#define MTK_WDT_LENGTH_TIME_OUT		(0xffe0)
#define MTK_WDT_LENGTH_KEYMASK		(0x001f)
#define MTK_WDT_LENGTH_KEY		(0x0008)
#define MTK_WDT_LENGTH_CTL_KEY		(0x95<<24)

/*WDT_RESTART*/
#define MTK_WDT_RESTART_KEY		(0x1971)

/*WDT_STATUS*/
#define MTK_WDT_STATUS_HWWDT_RST	(0x80000000)
#define MTK_WDT_STATUS_SWWDT_RST	(0x40000000)
#define MTK_WDT_STATUS_IRQWDT_RST	(0x20000000)
#define MTK_WDT_STATUS_DEBUGWDT_RST	(0x00080000)
#define MTK_WDT_STATUS_SPMWDT_RST	(0x0002)
#define MTK_WDT_STATUS_SPM_THERMAL_RST	(0x0001)
#define MTK_WDT_STATUS_THERMAL_DIRECT_RST	(1<<18)
#define MTK_WDT_STATUS_SECURITY_RST	(1<<28)





/*WDT_INTERVAL*/
#define MTK_WDT_INTERVAL_MASK		(0x0fff)

/*WDT_SWRST*/
#define MTK_WDT_SWRST_KEY		(0x1209)

/*WDT_SWSYSRST*/
#define MTK_WDT_SWSYS_RST_PWRAP_SPI_CTL_RST	(0x0800)
#define MTK_WDT_SWSYS_RST_APMIXED_RST	(0x0400)
#define MTK_WDT_SWSYS_RST_MD_LITE_RST	(0x0200)
#define MTK_WDT_SWSYS_RST_INFRA_AO_RST	(0x0100)
#define MTK_WDT_SWSYS_RST_MD_RST	(0x0080)
#define MTK_WDT_SWSYS_RST_DDRPHY_RST	(0x0040)
#define MTK_WDT_SWSYS_RST_IMG_RST	(0x0020)
#define MTK_WDT_SWSYS_RST_VDEC_RST	(0x0010)
#define MTK_WDT_SWSYS_RST_VENC_RST	(0x0008)
#define MTK_WDT_SWSYS_RST_MFG_RST	(0x0004)
#define MTK_WDT_SWSYS_RST_DISP_RST	(0x0002)
#define MTK_WDT_SWSYS_RST_INFRA_RST	(0x0001)
#define MTK_WDT_SWSYS_RST_C2KSYS_RST	(0x10000)
#define MTK_WDT_SWSYS_RST_C2KWDT_RST	(0x20000)
#define MTK_WDT_SWSYS_RST_CONMCU_RST	(0x1000)


/* #define MTK_WDT_SWSYS_RST_KEY		(0x1500) */
#define MTK_WDT_SWSYS_RST_KEY		(0x88000000)

/*MTK_WDT_REQ_IRQ*/
#define MTK_WDT_REQ_IRQ_KEY		(0x44000000)
#define MTK_WDT_REQ_IRQ_DEBUG_EN		(0x80000)
#define MTK_WDT_REQ_IRQ_SPM_THERMAL_EN		(0x0001)
#define MTK_WDT_REQ_IRQ_SPM_SCPSYS_EN		(0x0002)
#define MTK_WDT_REQ_IRQ_EINT_EN			(1<<2)
#define MTK_WDT_REQ_IRQ_SYSRST_EN		(1<<3)
#define MTK_WDT_REQ_IRQ_THERMAL_EN		(1<<18)


/*MTK_WDT_REQ_MODE*/
#define MTK_WDT_REQ_MODE_KEY		(0x33000000)
#define MTK_WDT_REQ_MODE_DEBUG_EN		(0x80000)
#define MTK_WDT_REQ_MODE_SPM_THERMAL		(0x0001)
#define MTK_WDT_REQ_MODE_SPM_SCPSYS		(0x0002)
#define MTK_WDT_REQ_MODE_EINT			(1<<2)
#define MTK_WDT_REQ_MODE_SYSRST			(1<<3)
#define MTK_WDT_REQ_MODE_THERMAL		(1<<18)

/*MTK_WDT_RSTDEG_EN*/
#define MTK_WDT_RSTDEG_EN1_KEY			(0xa357)
#define MTK_WDT_RSTDEG_EN2_KEY			(0x67d2)

/*MTK_WDT_SYSDBG_DEG_EN*/
#define MTK_WDT_SYSDBG_DEG_EN1_KEY		(0x1b2a)
#define MTK_WDT_SYSDBG_DEG_EN2_KEY		(0x4f59)


#endif   /*__WDT_HW_H__*/
