/*
 * Copyright (c) 2013-2016, ARM Limited and Contributors. All rights reserved.
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
#include <arch_helpers.h>
#include <arm_gic.h>
#include <cci.h>
#include <debug.h>
#include <mmio.h>
#include <mt8173_def.h>
#include <platform_def.h>
#include <utils.h>
#include <xlat_tables.h>

static const int cci_map[] = {
	PLAT_MT_CCI_CLUSTER0_SL_IFACE_IX,
	PLAT_MT_CCI_CLUSTER1_SL_IFACE_IX
};

/* Table of regions to map using the MMU.  */
const mmap_region_t plat_mmap[] = {
	/* for TF text, RO, RW */
	MAP_REGION_FLAT(TZRAM_BASE, TZRAM_SIZE,
			MT_MEMORY | MT_RW | MT_SECURE),
	MAP_REGION_FLAT(MTK_DEV_RNG0_BASE, MTK_DEV_RNG0_SIZE,
			MT_DEVICE | MT_RW | MT_SECURE),
	MAP_REGION_FLAT(MTK_DEV_RNG1_BASE, MTK_DEV_RNG1_SIZE,
			MT_DEVICE | MT_RW | MT_SECURE),
	{ 0 }

};

/*******************************************************************************
 * Macro generating the code for the function setting up the pagetables as per
 * the platform memory map & initialize the mmu, for the given exception level
 ******************************************************************************/
#define DEFINE_CONFIGURE_MMU_EL(_el)					\
	void plat_configure_mmu_el ## _el(unsigned long total_base,	\
					  unsigned long total_size,	\
					  unsigned long ro_start,	\
					  unsigned long ro_limit,	\
					  unsigned long coh_start,	\
					  unsigned long coh_limit)	\
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
		enable_mmu_el ## _el(0);				\
	}

/* Define EL3 variants of the function initialising the MMU */
DEFINE_CONFIGURE_MMU_EL(3)

unsigned int plat_get_syscnt_freq2(void)
{
	return SYS_COUNTER_FREQ_IN_TICKS;
}

static char *revisions[] = {
	"r0p0",
	"r0p1",
	"r0p2",
	"r0p3",
	"r0p4",
	"r1p0",
	"r1p1",
	"r1p2",
	"r1p3",
	"r1p4",
	"r1p5",
};

static void plat_cci_revision() {
	unsigned int revision;
	int i = 0;

	revision =  mmio_read_32(PLAT_MT_CCI_BASE + PERIPHERAL_ID2) >> 4;
	ERROR("CCI-400 revision = %s\n", revisions[revision]);
	INFO("CCI-400 revision = %s\n", revisions[revision]);
	INFO("CCI-400 ctrl overdride = %x\n",
			mmio_read_32(PLAT_MT_CCI_BASE + CTRL_OVERRIDE_REG));
	INFO("CCI-400 secure access = %x\n",
			mmio_read_32(PLAT_MT_CCI_BASE + SECURE_ACCESS_REG));

	mmio_write_32(PLAT_MT_CCI_BASE + SECURE_ACCESS_REG, 0x1);
	INFO("CCI-400 secure access = %x\n",
			mmio_read_32(PLAT_MT_CCI_BASE + SECURE_ACCESS_REG));
	for (i = 0; i < 3; i++) {
		INFO("CCI-400 interface %d\n", i);
		INFO("\tsnoop control = %x\n",
			mmio_read_32(PLAT_MT_CCI_BASE + SLAVE_IFACE_OFFSET(i)));
	}
	INFO("CCI-400 interface 3 snoop control = %x\n",
			mmio_read_32(PLAT_MT_CCI_BASE + SLAVE_IFACE3_OFFSET));
	INFO("CCI-400 interface 3 sh override = %x\n",
			mmio_read_32(PLAT_MT_CCI_BASE + SLAVE_IFACE3_OFFSET
			+ SH_OVERRIDE_REG));
	INFO("CCI-400 interface 4 snoop control = %x\n",
			mmio_read_32(PLAT_MT_CCI_BASE + SLAVE_IFACE4_OFFSET));
	INFO("CCI-400 interface 4 sh override = %x\n",
			mmio_read_32(PLAT_MT_CCI_BASE + SLAVE_IFACE4_OFFSET
			+ SH_OVERRIDE_REG));
}

void plat_cci_init(void)
{
	/* Initialize CCI driver */
	cci_init(PLAT_MT_CCI_BASE, cci_map, ARRAY_SIZE(cci_map));
	plat_cci_revision();
}

void plat_cci_enable(void)
{
	/*
	 * Enable CCI coherency for this cluster.
	 * No need for locks as no other cpu is active at the moment.
	 */
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
	 * setting the SPSR prior to entry into the BL3-2 image.
	 */
	return 0;
}

/*******************************************************************************
 * Gets SPSR for kernel entry
 ******************************************************************************/
uint32_t plat_get_spsr_for_kernel_entry(void)
{
	unsigned long el_status;
	unsigned int mode;

	el_status = read_id_aa64pfr0_el1() >> ID_AA64PFR0_EL2_SHIFT;
	el_status &= ID_AA64PFR0_ELX_MASK;

	mode = el_status ? MODE_EL2 : MODE_EL1;
	INFO("Kernel: 64bit, mode: %d\n", mode);

	return SPSR_64(mode, MODE_SP_ELX, DISABLE_ALL_EXCEPTIONS);
}
