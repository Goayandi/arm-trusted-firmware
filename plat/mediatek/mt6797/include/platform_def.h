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

#ifndef __PLATFORM_DEF_H__
#define __PLATFORM_DEF_H__

#include <arch.h>

#define CFG_FPGA_PLATFORM 0
#define PLAT_USE_EL1_AARCH32
#define DEBUG_XLAT_TABLE 0

/*******************************************************************************
 * Platform binary types for linking
 ******************************************************************************/
#define PLATFORM_LINKER_FORMAT          "elf64-littleaarch64"
#define PLATFORM_LINKER_ARCH            aarch64

/*******************************************************************************
 * Generic platform constants
 ******************************************************************************/

/* Size of cacheable stacks */
#if DEBUG_XLAT_TABLE
#define PLATFORM_STACK_SIZE 0x800
#elif IMAGE_BL31
#define PLATFORM_STACK_SIZE 0x800
#endif

#define FIRMWARE_WELCOME_STR		"Booting Trusted Firmware\n"

/* EL3 Runtime Firmware BL31 */
#define BL31_IMAGE_NAME			"bl31.bin"

#define PLATFORM_CACHE_LINE_SIZE	64
#define PLATFORM_MAX_AFFLVL		2
#define PLATFORM_SYSTEM_COUNT		1
#define PLATFORM_CLUSTER_COUNT		3ull
#define PLATFORM_CLUSTER0_CORE_COUNT	4
#define PLATFORM_CLUSTER1_CORE_COUNT	4
#define PLATFORM_CLUSTER2_CORE_COUNT	2
#define PLATFORM_CORE_COUNT		(PLATFORM_CLUSTER0_CORE_COUNT + \
					PLATFORM_CLUSTER1_CORE_COUNT + \
					PLATFORM_CLUSTER2_CORE_COUNT)
#define PLATFORM_MAX_CPUS_PER_CLUSTER	4
#define PLATFORM_NUM_AFFS		(PLATFORM_SYSTEM_COUNT + \
					 PLATFORM_CLUSTER_COUNT + \
					 PLATFORM_CORE_COUNT)
#define MAX_IO_DEVICES			3
#define MAX_IO_HANDLES			4

#define HI_DWORD(l64) ((unsigned int)(((unsigned long)(l64)>>32)&0xFFFFFFFF))
#define LO_DWORD(l64) ((unsigned int)((unsigned long)(l64)))

/*******************************************************************************
 * Platform memory map related constants
 ******************************************************************************/
#define TZROM_BASE		0x00000000
#define TZROM_SIZE		0x04000000


//ATF Argument
#define ATF_ARG_BASE		0x00100000
#define ATF_ARG_SIZE		0x800

//ATF txet, ro, rw, MEMBASE + 70MB, Size: 256KB
#define TZRAM_BASE		0x00100000
#define TZRAM_SIZE		0x22000

#define TZRAM2_BASE		(TZRAM_BASE + TZRAM_SIZE)
#define TZRAM2_SIZE		0x8000

/*useless, Location of trusted dram on the base mtk_platform */
#define TZDRAM_BASE		0x40020000
#define TZDRAM_SIZE		0x0000C000
#define DRAM_EMI_BASE  		0x40000000

#define RAM_CONSOLE_BASE	0x0012D000
#define RAM_CONSOLE_SIZE	0x00001000

/*******************************************************************************
 * BL31 specific defines.
 ******************************************************************************/
/*
 * Put BL3-1 at the top of the Trusted SRAM (just below the shared memory, if
 * present). BL31_BASE is calculated using the current BL3-1 debug size plus a
 * little space for growth.
 */
#define BL31_BASE		(TZRAM_BASE + ATF_ARG_SIZE)
#define BL31_LIMIT		(TZRAM_BASE + TZRAM_SIZE)
#define BL31_TZRAM_SIZE		(TZRAM_SIZE - ATF_ARG_SIZE)
#define TZRAM2_LIMIT		(TZRAM2_BASE + TZRAM2_SIZE)

/*
 * ID of the secure physical generic timer interrupt used by the TSP.
 */
#define TSP_IRQ_SEC_PHY_TIMER	IRQ_SEC_PHY_TIMER

/*******************************************************************************
 * Platform specific page table and MMU setup constants
 ******************************************************************************/
#define ADDR_SPACE_SIZE		(1ull << 32)
#define MAX_XLAT_TABLES		7
#define MAX_MMAP_REGIONS	16

/*******************************************************************************
 * ID of the secure physical generic timer interrupt.
 ******************************************************************************/
#define IRQ_SEC_PHY_TIMER	29

#define IO_PHYS			0x10000000
#define VER_BASE		0x08000000

#define INFRACFG_AO_BASE	(IO_PHYS + 0x00001000)
#define MCUCFG_BASE		(IO_PHYS + 0x00220000)
#define EMI_MPU_BASE		(IO_PHYS + 0x00200000)

/*******************************************************************************
 * BOOTROM Related Registers
 ******************************************************************************/
#define BOOTROM_BOOT_ADDR	(INFRACFG_AO_BASE + 0x800)
#define BOOTROM_SEC_CTRL	(INFRACFG_AO_BASE + 0x804)
#define SW_ROM_PD		(1U << 31)

//#define MP0_MISC_CONFIG_BASE	(IO_PHYS + 0x00200000)
#define MP0_MISC_CONFIG_BOOT_ADDR(cpu)		(MCUCFG_BASE + 0x38 + ((cpu) * 8))
#define MP1_MISC_CONFIG_BOOT_ADDR(cpu)		(MCUCFG_BASE + 0x238 + ((cpu) * 8))
#define MP2_MISC_CONFIG_BOOT_ADDR_L(cpu)	(MCUCFG_BASE + 0x2290 + ((cpu) * 8))
#define MP2_MISC_CONFIG_BOOT_ADDR_H(cpu)	(MCUCFG_BASE + 0x2294 + ((cpu) * 8))
#define MP0_MISC_CONFIG3        		(MCUCFG_BASE + 0x3c)
#define MP1_MISC_CONFIG3			(MCUCFG_BASE + 0x23c)
#define MP2_MISC_CONFIG3			(MCUCFG_BASE + 0x2294)

#define MP0_MISC_CONFIG0 			(MCUCFG_BASE + 0x0030)
#define MP0_MISC_CONFIG9 			(MCUCFG_BASE + 0x0054)
#define MP1_MISC_CONFIG0 			(MCUCFG_BASE + 0x0230)
#define MP1_MISC_CONFIG9 			(MCUCFG_BASE + 0x0254)

/*******************************************************************************
 * SCU Related Registers
 ******************************************************************************/
#define MP0_AXI_CONFIG		(MCUCFG_BASE + 0x002C)
#define MP1_AXI_CONFIG		(MCUCFG_BASE + 0x022C)
#define MP2_AXI_CONFIG		(MCUCFG_BASE + 0x222C)
#define ACINACTM		(1U << 4)

/*******************************************************************************
 * Declarations and constants to access the mailboxes safely. Each mailbox is
 * aligned on the biggest cache line size in the platform. This is known only
 * to the platform as it might have a combination of integrated and external
 * caches. Such alignment ensures that two maiboxes do not sit on the same cache
 * line at any cache level. They could belong to different cpus/clusters &
 * get written while being protected by different locks causing corruption of
 * a valid mailbox address.
 ******************************************************************************/
#define CACHE_WRITEBACK_SHIFT		6
#define CACHE_WRITEBACK_GRANULE		(1 << CACHE_WRITEBACK_SHIFT)

/* FIQ platform related define */
#define PLAT_DEFINED_FIQ_SMP_CALL	1
#define WDT_IRQ_BIT_ID			69
#define FIQ_SMP_CALL_SGI		13

#define MP0_CA7L_CACHE_CONFIG		(MCUCFG_BASE + 0)
#define MP1_CA7L_CACHE_CONFIG		(MCUCFG_BASE + 0x200)
#define L2RSTDISABLE 			(1 << 4)

/*=======================================================================*/
/* AP HW code offset                                                     */
/*=======================================================================*/
#define APHW_CODE		(VER_BASE)
#define APHW_SUBCODE		(VER_BASE + 0x04)
#define APHW_VER		(VER_BASE + 0x08)
#define APSW_VER		(VER_BASE + 0x0C)

#define SCP_CFGREG_BASE		0x100A0000
#define SCP_RESET_REG		SCP_CFGREG_BASE

#define SCP_RESET_EN		(1 << 0)

#define WDT_SWSYSRST			0x10007018
#define PLL_DIV_MUXL_SEL		0x1001a270
#define MP2_SNOOP_CTRL			0x10396000
#define MP2_SNOOP_STATUS		0x1039000C

#define INFRA_TOPAXI_PROTECTEN1		0x10001234
#define INFRA_TOPAXI_PROTEXTSTA3	0x1000123C

// SPM_x: SPM_ is a prefix, x is the name in CODA
#define SPM_BASE		(0x10006000)
#define SPM_CPU_IDLE_STA2	(SPM_BASE + 0x178)
#define SPM_CPU_PWR_STATUS	(SPM_BASE + 0x188)
#define SPM_CPU_PWR_STATUS_2ND	(SPM_BASE + 0x18c)
#define SPM_CPU_RET_STATUS	(SPM_BASE + 0x1e4)
#define SPM_MP0_CPUSYS_PWR_CON	(SPM_BASE + 0x210)
#define SPM_MP1_CPUSYS_PWR_CON	(SPM_BASE + 0x214)
#define SPM_MP2_CPUSYS_PWR_CON	(SPM_BASE + 0x218)
#define SPM_MP2_CPU0_PWR_CON	(SPM_BASE + 0x240)
#define SPM_CPU_EXT_BUCK_ISO	(SPM_BASE + 0x290)

#define MP0_CA7L_MISC_CONFIG	(MCUCFG_BASE + 0x064)
#define MP1_CA7L_MISC_CONFIG	(MCUCFG_BASE + 0x264)
#define CCI_CONFIG		(MCUCFG_BASE + 0x0654)
#define CPUCFG			(MCUCFG_BASE + 0x2208)
#define ARMPLL_CON0		(MCUCFG_BASE + 0x24a0)
#define CPUSYS0_SPARKVRETCNTRL	(MCUCFG_BASE + 0x1c00)
#define CPUSYS0_SPARKEN		(MCUCFG_BASE + 0x1c04)
#define CPUSYS1_SPARKVRETCNTRL	(MCUCFG_BASE + 0x3c00)
#define CPUSYS1_SPARKEN		(MCUCFG_BASE + 0x3c04)
#define MISCDBG			(MCUCFG_BASE + 0x220c)
#define PTP3_CPUTOP_SPMC	(MCUCFG_BASE + 0x22A0)
#define SRAMLDO			(MCUCFG_BASE + 0x22B0)
#define PTP3_CPU0_SPMC		(MCUCFG_BASE + 0x2430)
#define PTP3_CPU1_SPMC		(MCUCFG_BASE + 0x2434)
#define SPARK2LDO		(MCUCFG_BASE + 0x2700)
#define FSM_STATE_OUT_OFFSET		(23)
#define FSM_STATE_OUT_MASK		(0x1f <<FSM_STATE_OUT_OFFSET)
#define SW_LOGIC_PDBO_ALL_OFF_ACK	(1<<22)
#define SW_LOGIC_PDBO_ALL_ON_ACK	(1<<21)
#define SW_LOGIC_PRE2_PDBO_ALL_ON_ACK	(1<<20)
#define SW_LOGIC_PRE1_PDBO_ALL_ON_ACK	(1<<19)
#define SW_HOT_PLUG_RESET		(1<<16)
#define SRAMPD_OFFSET			(10)
#define SRAMPD				(0x3f<<SRAMPD_OFFSET)
#define SW_SRAM_ISOINTB			(1<<9)
#define SW_ISO				(1<<8)
#define SW_SRAM_SLEEPB			(1<<7)
#define SW_LOGIC_PDB			(1<<6)
#define SW_LOGIC_PRE2_PDB		(1<<5)
#define SW_LOGIC_PRE1_PDB		(1<<4)
#define SW_FSM_OVERRIDE			(1<<3)
#define SW_PWR_ON			(1<<2)
#define SW_PWR_ON_OVERRIDE_EN		(1<<1)
#define HW_SEM_ENABLE_WORKAROUND_1axxx	(0x11015000)
#define HW_SEM_WORKAROUND_1axxx		(0x11015448)
#define FSM_OFF				(0<<FSM_STATE_OUT_OFFSET)
#define FSM_ON				(1<<FSM_STATE_OUT_OFFSET)
#define FSM_RET				(2<<FSM_STATE_OUT_OFFSET)

#define TWO_MB_SHIFT			21
#define PAGE_SIZE_2MB_SHIFT		TWO_MB_SHIFT
#define PAGE_SIZE_2MB			(1 << PAGE_SIZE_2MB_SHIFT)
#define PAGE_SIZE_2MB_MASK		(PAGE_SIZE_2MB - 1)
#define IS_PAGE_2MB_ALIGNED(addr)	(((addr) & PAGE_SIZE_2MB_MASK) == 0)

#define UART_CLOCK             		26000000
#define UART_BAUDRATE			921600

#define	SYS_COUNTER_FREQ_IN_TICKS	13000000

#if !ENABLE_PLAT_COMPAT
#define PLAT_MAX_OFF_STATE	2
#define PLAT_MAX_RET_STATE	1
#define PLAT_MAX_PWR_LVL	PLAT_MAX_OFF_STATE

/* Local power state for power domains in Run state. */
#define MTK_LOCAL_STATE_RUN     0
/* Local power state for retention. Valid only for CPU power domains */
#define MTK_LOCAL_STATE_RET     1
/* Local power state for OFF/power-down. Valid for CPU and cluster power
   domains */
#define MTK_LOCAL_STATE_OFF     2
#endif

#endif /* __PLATFORM_DEF_H__ */
