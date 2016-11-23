/*
 * Copyright (c) 2014, ARM Limited and Contributors. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of ARM nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific
 * prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __PLAT_DEF_H__
#define __PLAT_DEF_H__

#include <platform_def.h> /* for TZROM_SIZE */

/* Firmware Image Package */
#define FIP_IMAGE_NAME			"fip.bin"
#define FVP_PRIMARY_CPU			0x0

/* Memory location options for Shared data and TSP in FVP */
#define FVP_IN_TRUSTED_SRAM		0
#define FVP_IN_TRUSTED_DRAM		1

/*******************************************************************************
 * FVP memory map related constants
 ******************************************************************************/

#define FVP_TRUSTED_ROM_BASE	0x00000000
#define FVP_TRUSTED_ROM_SIZE	0x04000000	/* 64 MB */

#define FVP_TRUSTED_SRAM_BASE	0x04000000
#define FVP_TRUSTED_SRAM_SIZE	0x00040000	/* 256 KB */

#define FVP_TRUSTED_DRAM_BASE	0x06000000
#define FVP_TRUSTED_DRAM_SIZE	0x02000000	/* 32 MB */

#define FLASH0_BASE		0x08000000
#define FLASH0_SIZE		0x04000000

#define FLASH1_BASE		0x0c000000
#define FLASH1_SIZE		0x04000000

#define PSRAM_BASE		0x14000000
#define PSRAM_SIZE		0x04000000

#define VRAM_BASE		0x18000000
#define VRAM_SIZE		0x02000000

/* Aggregate of all devices in the first GB */
#define DEVICE0_BASE		0x1a000000
#define DEVICE0_SIZE		0x12200000

#define DEVICE1_BASE		0x2f000000
#define DEVICE1_SIZE		0x200000

#define NSRAM_BASE		0x2e000000
#define NSRAM_SIZE		0x10000

/* Aggregate of all devices in the first GB */
#define MTK_DEVICE_BASE		0x11000000 //[FIXME]
#define MTK_DEVICE_SIZE		0x1000000

#define MT_DEV_BASE 0x10000000
#define MT_DEV_SIZE   0x400000

#define MT_GIC_BASE 0x19000000
#define MT_GIC_SIZE   0x400000
#define MT_GIC_RDIST_BASE	0x19200000

#define EINT_BASE 0x1000b000

/* 4KB shared memory */
#define FVP_SHARED_RAM_SIZE	0x1000

/* Location of shared memory */
#if (FVP_SHARED_DATA_LOCATION_ID == FVP_IN_TRUSTED_DRAM)
/* Shared memory at the base of Trusted DRAM */
# define FVP_SHARED_RAM_BASE		FVP_TRUSTED_DRAM_BASE
# define FVP_TRUSTED_SRAM_LIMIT		(FVP_TRUSTED_SRAM_BASE \
					+ FVP_TRUSTED_SRAM_SIZE)
#elif (FVP_SHARED_DATA_LOCATION_ID == FVP_IN_TRUSTED_SRAM)
# if (FVP_TSP_RAM_LOCATION_ID == FVP_IN_TRUSTED_DRAM)
#  error "Shared data in Trusted SRAM and TSP in Trusted DRAM is not supported"
# endif
/* Shared memory at the top of the Trusted SRAM */
# define FVP_SHARED_RAM_BASE		(FVP_TRUSTED_SRAM_BASE \
					+ FVP_TRUSTED_SRAM_SIZE \
					- FVP_SHARED_RAM_SIZE)
# define FVP_TRUSTED_SRAM_LIMIT		FVP_SHARED_RAM_BASE
#else
# error "Unsupported FVP_SHARED_DATA_LOCATION_ID value"
#endif

#define DRAM1_BASE		0x41000000ull
#define DRAM1_SIZE		0x1E000000ull
#define DRAM1_END		(DRAM1_BASE + DRAM1_SIZE - 1)
#define DRAM1_SEC_SIZE		0x01000000ull

#define DRAM_BASE		DRAM1_BASE
#define DRAM_SIZE		DRAM1_SIZE

#define DRAM2_BASE		0x880000000ull
#define DRAM2_SIZE		0x780000000ull
#define DRAM2_END		(DRAM2_BASE + DRAM2_SIZE - 1)

#define PCIE_EXP_BASE		0x40000000
#define TZRNG_BASE		0x7fe60000
#define TZNVCTR_BASE		0x7fe70000
#define TZROOTKEY_BASE		0x7fe80000

/* Memory mapped Generic timer interfaces  */
#define SYS_CNTCTL_BASE		0x2a430000
#define SYS_CNTREAD_BASE	0x2a800000
#define SYS_TIMCTL_BASE		0x2a810000

/* V2M motherboard system registers & offsets */
#define VE_SYSREGS_BASE		0x1c010000
#define V2M_SYS_ID		0x0
#define V2M_SYS_SWITCH		0x4
#define V2M_SYS_LED		0x8
#define V2M_SYS_CFGDATA		0xa0
#define V2M_SYS_CFGCTRL		0xa4
#define V2M_SYS_CFGSTATUS	0xa8

#define CFGCTRL_START		(1 << 31)
#define CFGCTRL_RW		(1 << 30)
#define CFGCTRL_FUNC_SHIFT	20
#define CFGCTRL_FUNC(fn)	(fn << CFGCTRL_FUNC_SHIFT)
#define FUNC_CLK_GEN		0x01
#define FUNC_TEMP		0x04
#define FUNC_DB_RESET		0x05
#define FUNC_SCC_CFG		0x06
#define FUNC_SHUTDOWN		0x08
#define FUNC_REBOOT		0x09

/* Load address of BL33 in the FVP port */
#define NS_IMAGE_OFFSET		(0x41E00000) /* DRAM + 128MB */

/* Special value used to verify platform parameters from BL2 to BL3-1 */
#define FVP_BL31_PLAT_PARAM_VAL	0x0f1e2d3c4b5a6978ULL

/*
 * V2M sysled bit definitions. The values written to this
 * register are defined in arch.h & runtime_svc.h. Only
 * used by the primary cpu to diagnose any cold boot issues.
 *
 * SYS_LED[0]   - Security state (S=0/NS=1)
 * SYS_LED[2:1] - Exception Level (EL3-EL0)
 * SYS_LED[7:3] - Exception Class (Sync/Async & origin)
 *
 */
#define SYS_LED_SS_SHIFT		0x0
#define SYS_LED_EL_SHIFT		0x1
#define SYS_LED_EC_SHIFT		0x3

#define SYS_LED_SS_MASK		0x1
#define SYS_LED_EL_MASK		0x3
#define SYS_LED_EC_MASK		0x1f

/* V2M sysid register bits */
#define SYS_ID_REV_SHIFT	28
#define SYS_ID_HBI_SHIFT	16
#define SYS_ID_BLD_SHIFT	12
#define SYS_ID_ARCH_SHIFT	8
#define SYS_ID_FPGA_SHIFT	0

#define SYS_ID_REV_MASK	0xf
#define SYS_ID_HBI_MASK	0xfff
#define SYS_ID_BLD_MASK	0xf
#define SYS_ID_ARCH_MASK	0xf
#define SYS_ID_FPGA_MASK	0xff

#define SYS_ID_BLD_LENGTH	4

#define REV_MT		0x0
#define HBI_MT_BASE		0x020
#define HBI_FOUNDATION		0x010

#define HBI_FOUNDATION		0x010
#define REV_FOUNDATION_V2_0	0x0
#define REV_FOUNDATION_V2_1	0x1

#define BLD_GIC_VE_MMAP	0x0
#define BLD_GIC_A53A57_MMAP	0x1

#define ARCH_MODEL		0x1

/* FVP Power controller base address*/
#define PWRC_BASE		0x1c100000


/*******************************************************************************
 * CCI-400 related constants
 ******************************************************************************/
#define CCI400_BASE			0x10390000  //[FIXME]
#define CCI400_SL_IFACE3_CLUSTER_IX	1
#define CCI400_SL_IFACE4_CLUSTER_IX	0
#define CCI400_SL_IFACE5_CLUSTER_IX	2
#define CCI_SEC_ACCESS_OFFSET   (0x8)

/*******************************************************************************
 * GIC-400 & interrupt handling related constants
 ******************************************************************************/
/* VE compatible GIC memory map */
//useless, should move to a meanful place
//#define VE_GICD_BASE			0x2c001000
#define VE_GICC_BASE			0x2c002000  //[FIXME]
//#define VE_GICH_BASE			0x2c004000
//#define VE_GICV_BASE			0x2c006000

/* Base MTK_platform compatible GIC memory map */
#define BASE_GICD_BASE  (MT_GIC_BASE)
//#define BASE_GICR_BASE			0x2f100000
#define BASE_GICC_BASE  (MT_GIC_BASE + 0x200000)
#define BASE_GICH_BASE  (MT_GIC_BASE + 0x4000)
#define BASE_GICV_BASE  (MT_GIC_BASE + 0x6000)
#define INT_POL_CTL0        0x10220620

#define MT_EDGE_SENSITIVE 1
#define MT_LEVEL_SENSITIVE 0
#define MT_POLARITY_LOW   0
#define MT_POLARITY_HIGH  1


#define GIC_PRIVATE_SIGNALS     (32)
#define NR_GIC_SGI              (16)
#define NR_GIC_PPI              (16)
#define GIC_PPI_OFFSET          (27)
#define MT_NR_PPI               (5)
#define MT_NR_SPI               (241)
#define NR_MT_IRQ_LINE          (GIC_PPI_OFFSET + MT_NR_PPI + MT_NR_SPI)

#define IRQ_TZ_WDOG			56
#define IRQ_SEC_PHY_TIMER		29
#define IRQ_SEC_SGI_0			8
#define IRQ_SEC_SGI_1			9
#define IRQ_SEC_SGI_2			10
#define IRQ_SEC_SGI_3			11
#define IRQ_SEC_SGI_4			12
#define IRQ_SEC_SGI_5			13
#define IRQ_SEC_SGI_6			14
#define IRQ_SEC_SGI_7			15
#define IRQ_SEC_SGI_8			16

/*******************************************************************************
 * PL011 related constants
 ******************************************************************************/
#define PL011_UART0_BASE		0x1c090000
#define PL011_UART1_BASE		0x1c0a0000
#define PL011_UART2_BASE		0x1c0b0000
#define PL011_UART3_BASE		0x1c0c0000

#define PL011_BAUDRATE  115200

#define PL011_UART0_CLK_IN_HZ 24000000
#define PL011_UART1_CLK_IN_HZ 24000000
#define PL011_UART2_CLK_IN_HZ 24000000
#define PL011_UART3_CLK_IN_HZ 24000000
#define IO_PHYS             (0x10000000)
#define PAGE_ADDR_MASK          (0xFFF00000)

#define UART0_BASE          (IO_PHYS + 0x01002000)
#define UART1_BASE          (IO_PHYS + 0x01003000)
#define UART2_BASE          (IO_PHYS + 0x01004000)
#define UART3_BASE          (IO_PHYS + 0x01005000)
#define PERICFG_BASE        (IO_PHYS + 0x2000)

#define MT6797_UART_CLOCK		26000000
#define MT6797_UART_BAUDRATE		921600

/*******************************************************************************
 * TrustZone address space controller related constants
 ******************************************************************************/
#define TZC400_BASE			0x2a4a0000

/*
 * The NSAIDs for this platform as used to program the TZC400.
 */

/* NSAIDs used by devices in TZC filter 0 on FVP */
#define FVP_NSAID_DEFAULT		0
#define FVP_NSAID_PCI			1
#define FVP_NSAID_VIRTIO		8  /* from FVP v5.6 onwards */
#define FVP_NSAID_AP			9  /* Application Processors */
#define FVP_NSAID_VIRTIO_OLD		15 /* until FVP v5.5 */

/* NSAIDs used by devices in TZC filter 2 on FVP */
#define FVP_NSAID_HDLCD0		2
#define FVP_NSAID_CLCD			7

/*******************************************************************************
 *  Shared Data
 ******************************************************************************/

/* Entrypoint mailboxes */
// #define MBOX_BASE		FVP_SHARED_RAM_BASE

#ifndef __ASSEMBLY__
extern unsigned long mt_mbox[];
#endif
#define MBOX_BASE			mt_mbox//FVP_SHARED_RAM_BASE

#define MBOX_SIZE		0x300

/* Base address where parameters to BL31 are stored */
#define PARAMS_BASE		(MBOX_BASE + MBOX_SIZE)

/*******************************************************************************
 * TRNG Registers
 ******************************************************************************/
#define TRNG_base               (0x1020F000)// TRNG Physical Address
#define TRNG_BASE_ADDR          TRNG_base
#define TRNG_BASE_SIZE          (0x1000)
#define TRNG_CTRL               (TRNG_base+0x0000)
#define TRNG_TIME               (TRNG_base+0x0004)
#define TRNG_DATA               (TRNG_base+0x0008)
#define TRNG_PDN_base           (0x10001000)
#define TRNG_PDN_BASE_ADDR      TRNG_PDN_BASE_ADDR
#define TRNG_PDN_BASE_SIZE      (0x1000)
#define TRNG_PDN_SET            (TRNG_PDN_base +0x0088)
#define TRNG_PDN_CLR            (TRNG_PDN_base +0x008C)
#define TRNG_PDN_STATUS			(TRNG_PDN_base +0x0094)
#define TRNG_CTRL_RDY			0x80000000
#define TRNG_CTRL_START			0x00000001
#define TRNG_PDN_VALUE			0x200

/*******************************************************************************
 * WDT Registers
 ******************************************************************************/
#define MTK_WDT_BASE            (IO_PHYS + 0x00007000)
#define MTK_WDT_SIZE            (0x1000)
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
#define MTK_WDT_DEBUG_CTL		(MTK_WDT_BASE+0x0040)

#define MTK_WDT_MODE_DUAL_MODE		0x0040
#define MTK_WDT_MODE_IRQ		0x0008
#define MTK_WDT_MODE_KEY		0x22000000
#define MTK_WDT_MODE_EXTEN		0x0004
#define MTK_WDT_SWRST_KEY		0x1209
#define MTK_WDT_RESTART_KEY		(0x1971)

/*WDT_STATUS*/
#define MTK_WDT_STATUS_HWWDT_RST    (0x80000000)
#define MTK_WDT_STATUS_SWWDT_RST    (0x40000000)
#define MTK_WDT_STATUS_IRQWDT_RST   (0x20000000)
#define MTK_WDT_STATUS_DEBUGWDT_RST (0x00080000)
#define MTK_WDT_STATUS_SPMWDT_RST   (0x0002)
#define MTK_WDT_STATUS_SPM_THERMAL_RST      (0x0001)
#define MTK_WDT_STATUS_THERMAL_DIRECT_RST   (1<<18)
#define MTK_WDT_STATUS_SECURITY_RST         (1<<28)

#endif /* __PLAT_DEF_H__ */
