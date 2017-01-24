#include <arch.h>
#include <assert.h>
#include <debug.h>
#include <mmio.h>
#include <platform_def.h>
#include <stdio.h>
#include <stdarg.h>

void disable_scu(unsigned int id)
{
	uint32_t axi_config = 0;

	VERBOSE("enable_scu(0x%x)\n", id);
	if (id < 4)
		axi_config = MP0_AXI_CONFIG;
	else if (id < 8)
		axi_config = MP1_AXI_CONFIG;
	else if (id < 10)
		axi_config = MP2_AXI_CONFIG;
	else
		VERBOSE("wrong core, core id = %x\n", id);

	mmio_write_32(axi_config, mmio_read_32(axi_config) | ACINACTM);
}

void enable_scu(unsigned long mpidr)
{
	uint32_t axi_config = 0;

	VERBOSE("enable_scu(0x%x)\n", mpidr);
	switch (mpidr & MPIDR_CLUSTER_MASK) {
	case 0x000:
		axi_config = MP0_AXI_CONFIG;
		break;
	case 0x100:
		axi_config = MP1_AXI_CONFIG;
		break;
	case 0x200:
		axi_config = MP2_AXI_CONFIG;
		break;
	default:
		printf("wrong mpidr\n");
		assert(0);
	}
	mmio_write_32(axi_config, mmio_read_32(axi_config) & ~ACINACTM);
}
