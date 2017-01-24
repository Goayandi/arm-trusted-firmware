#include <platform_def.h>
#include <arch.h>
#include <arch_helpers.h>
#include <mmio.h>
#include <sip_error.h>
#include <spinlock.h>
#include <debug.h>
#include "plat_private.h"
#include "l2c.h"

spinlock_t l2_share_lock;

void config_L2_size(void)
{
	unsigned int cache_cfg0;

	/* D1: mp0 L2$ 512KB; D2: mp0 L2$ 256KB */
	cache_cfg0 = mmio_read_32(MP0_CA7L_CACHE_CONFIG) &
			(0xF << L2C_SIZE_CFG_OFF);
	cache_cfg0 = (cache_cfg0 << 1) | (0x1 << L2C_SIZE_CFG_OFF);
	cache_cfg0 = (mmio_read_32(MP0_CA7L_CACHE_CONFIG) &
			~(0xF << L2C_SIZE_CFG_OFF)) | cache_cfg0;
	mmio_write_32(MP0_CA7L_CACHE_CONFIG, cache_cfg0);
	cache_cfg0 = mmio_read_32(MP0_CA7L_CACHE_CONFIG) &
			~(0x1 << L2C_SHARE_ENABLE);
	mmio_write_32(MP0_CA7L_CACHE_CONFIG, cache_cfg0);
}
