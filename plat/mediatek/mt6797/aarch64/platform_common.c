/*
 * Copyright (c) 2013-2014, ARM Limited and Contributors. All rights reserved.
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

#include <arch.h>
#include <arch_helpers.h>
#include <arm_gic.h>
#include <bl_common.h>
#include <cci.h>
#include <debug.h>

#include <mtk_plat_common.h>
#include <mmio.h>
#include <platform.h>
#include <xlat_tables.h>
#include <plat_def.h>
#include <plat_private.h>

/*
 * Table of regions to map using the MMU.
 * This doesn't include TZRAM as the 'mem_layout' argument passed to
 * configure_mmu_elx() will give the available subset of that,
 */
const mmap_region_t plat_mmap[] = {
	/* for ATF text, RO, RW */
#if 0
	{ (TZRAM_BASE & PAGE_ADDR_MASK), (TZRAM_BASE & PAGE_ADDR_MASK),
		PAGE_SIZE_2MB, MT_MEMORY | MT_RW | MT_SECURE },
#endif
	// ((TZRAM_SIZE & ~(PAGE_SIZE_MASK)) + PAGE_SIZE), MT_MEMORY |
	// MT_RW | MT_SECURE },

	/* for T-OS, but we do not load T-OS in ATF. do not need it */
	// { TZDRAM_BASE, TZDRAM_SIZE, MT_MEMORY | MT_RW | MT_SECURE }

	/* UART address mapping */
	{ MTK_DEVICE_BASE, MTK_DEVICE_BASE, MTK_DEVICE_SIZE,
	  MT_DEVICE | MT_RW | MT_SECURE },

	/* CCI 400 address mapping */
	{ (MT_DEV_BASE & PAGE_ADDR_MASK), (MT_DEV_BASE & PAGE_ADDR_MASK),
	  MT_DEV_SIZE, MT_DEVICE | MT_RW | MT_SECURE },

	/* GIC address mapping */
	{ (MT_GIC_BASE & PAGE_ADDR_MASK), (MT_GIC_BASE & PAGE_ADDR_MASK),
	  MT_GIC_SIZE, MT_DEVICE | MT_RW | MT_SECURE },

	/* For TRNG and Clock Control address mapping */
	// including in MT_DEV_BASE and MT_DEV_SIZE, limit 4 XLAT_TABLES
	// {TRNG_BASE_ADDR, TRNG_BASE_SIZE, MT_DEVICE | MT_RW | MT_SECURE},
	// {TRNG_PDN_BASE_ADDR, TRNG_PDN_BASE_SIZE,
	//  MT_DEVICE | MT_RW | MT_SECURE},

	/* Top-level Reset Generator - WDT */
	/* including in MT_DEV_BASE and MT_DEV_SIZE */
	// { MTK_WDT_BASE, MTK_WDT_BASE, ((MTK_WDT_SIZE & ~(PAGE_SIZE_MASK)) +
	//	PAGE_SIZE), MT_DEVICE | MT_RW | MT_SECURE },

	/* 2nd GB as device for now...*/
	/* TZC-400 setting, we use Device-APC instead, do not use it yet */
	/* { DRAM1_BASE, DRAM1_SIZE, MT_MEMORY | MT_RW | MT_NS }, */
	{0}
};

/* Array of secure interrupts to be configured by the gic driver */
const unsigned int irq_sec_array[] = {
	IRQ_TZ_WDOG,
	IRQ_SEC_PHY_TIMER,
	IRQ_SEC_SGI_0,
	IRQ_SEC_SGI_1,
	IRQ_SEC_SGI_2,
	IRQ_SEC_SGI_3,
	IRQ_SEC_SGI_4,
	IRQ_SEC_SGI_5,
	IRQ_SEC_SGI_6,
	IRQ_SEC_SGI_7
};

const unsigned int num_sec_irqs = sizeof(irq_sec_array) /
	sizeof(irq_sec_array[0]);

/*******************************************************************************
 * Macro generating the code for the function setting up the pagetables as per
 * the platform memory map & initialize the mmu, for the given exception level
 ******************************************************************************/
#define DEFINE_CONFIGURE_MMU_EL(_el)					\
	void plat_configure_mmu_el##_el(unsigned long total_base,	\
				   unsigned long total_size,		\
				   unsigned long ro_start,		\
				   unsigned long ro_limit,		\
				   unsigned long coh_start,		\
				   unsigned long coh_limit)		\
	{								\
		mmap_add_region(total_base, total_base,			\
				total_size,				\
				MT_MEMORY | MT_RW | MT_SECURE);		\
		mmap_add_region(ro_start, ro_start,			\
				ro_limit - ro_start,			\
				MT_MEMORY | MT_RO | MT_SECURE);		\
		mmap_add_region(coh_start, coh_start,			\
				coh_limit - coh_start,			\
				MT_DEVICE | MT_RW | MT_SECURE);		\
		mmap_add(plat_mmap);					\
		init_xlat_tables();					\
									\
		enable_mmu_el##_el(0);					\
	}

/* Define EL1 and EL3 variants of the function initialising the MMU */
DEFINE_CONFIGURE_MMU_EL(1)
DEFINE_CONFIGURE_MMU_EL(3)

unsigned long plat_get_ns_image_entrypoint(void)
{
	// return NS_IMAGE_OFFSET;
	// return BL33_START_ADDRESS;
	return 0x46000000;
}

unsigned long long plat_get_syscnt_freq(void)
{
	uint64_t counter_base_frequency;

	/* Read the frequency from Frequency modes table */
	// counter_base_frequency = mmio_read_32(SYS_CNTCTL_BASE + CNTFID_OFF);
	counter_base_frequency = 13000000; //FIXME, 13 MHz

	/* The first entry of the frequency modes table must not be 0 */
	if (counter_base_frequency == 0)
		panic();

	return counter_base_frequency;
}

static const int cci_map[] = {
	4,
	3,
	5,
};

void plat_cci_init(void)
{
	/*
	 * Initialize MCSI-A driver, which is a variant of CCI-400
	 */
	cci_init(CCI400_BASE, cci_map, ARRAY_SIZE(cci_map));
}

void plat_cci_enable(void)
{
	/*
	 * Enable MCSI-A coherency for this cluster. No need
	 * for locks as no other cpu is active at the
	 * moment
	 */
	mmio_write_32(CCI400_BASE, (mmio_read_32(CCI400_BASE)|0xffff0000));
	cci_enable_snoop_dvm_reqs(MPIDR_AFFLVL1_VAL(read_mpidr()));
}

void plat_cci_disable(void)
{
	cci_disable_snoop_dvm_reqs(MPIDR_AFFLVL1_VAL(read_mpidr()));
}

/*******************************************************************************
 * Gets SPSR for BL32 entry
 ******************************************************************************/
uint32_t plat_get_spsr_for_bl32_entry(void)
{
	/*
	 * The Secure Payload Dispatcher service is responsible for
	 * setting the SPSR prior to entry into the BL32 image.
	 */
	return 0;
}

uint32_t get_devinfo_with_index(uint32_t i)
{
	struct atf_arg_t *teearg = &gteearg;

	if (i < DEVINFO_SIZE)
		return teearg->devinfo[i];
	else
		return 0;
}

int is_mp0_off(void)
{
	/* cluster 0 disabled or not? */
	if (get_devinfo_with_index(2) & 0x4)
		return 1;
	else
		return 0;
}
