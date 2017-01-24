#include <arch_helpers.h>
#include <console.h>
#include <plat_def.h>
#include <stdio.h>
#include <stdint.h>
#include <xlat_tables.h>

#include "plat_cache.h"

static void write_ramindex_ca72(uint64_t v)
{
	// printf("tag:0x%lx\n", v);
	mt_write_ramindex_ca72(v);
	// __asm__ ("sys #0, c15, c4, #0, %0" : : "r" (v));
	// dsb();
	// isb();
}

static uint32_t read_il1_data0(void)
{
	uint64_t v;
	// __asm__ ("mrs %0, s3_0_c15_c0_0;" : "=r" (v));
	v = mt_read_il1_data0_ca72();
	// printf("data0:0x%lx\n", v);
	isb();
	dsb();
	return (v & 0xFFFFFFFF);
}

static uint32_t read_il1_data1(void)
{
	uint64_t v;
	// __asm__ ("mrs %0, s3_0_c15_c0_1;" : "=r" (v));
	v = mt_read_il1_data1_ca72();
	// printf("data1:0x%lx\n", v);
	isb();
	dsb();
	return (v & 0xFFFFFFFF);
}

struct cache_dump_header {
	uint32_t cpu_id;
	uint32_t way;
	uint32_t set;
	uint32_t tag_size;
	uint32_t line_size;
	uint32_t dump_size;
};

struct ca72_icache_dump {
	uint32_t ramidx;
	uint32_t tag[2];
	uint32_t data[16];
};


static int mt_icache_dump_ca72(uint64_t addr, uint64_t size)
{
	uint32_t id, way, set, bank_v;
	uint32_t ramidx;
	struct cache_dump_header *header;
	/* 32 bit pointer */
	uint32_t *ptr = (uint32_t *)addr;
	struct ca72_icache_dump *dump;

	printf("addr=0x%lx, size=0x%lx\n", addr, size);
	header = (struct cache_dump_header *)ptr;

	/* 0xD08 CA72 processor */
	header->cpu_id = 0xD08;
	header->way = 3;
	header->set = 256;
	header->tag_size = 8; /* 8 byte per icache tag */
	header->line_size = 64; /* 64 byte per line */

	ptr += sizeof(struct cache_dump_header) / sizeof(uint32_t);

	/*
	 * dump way value: 3, 2, 1 because ca72 trm:
	 * Setting the way field to a value of 3, reads way 2 of the cache.
	 */

	dump = (struct ca72_icache_dump *)ptr;

	for (way = header->way; way > 0; way--) {
		for (set = 0; set < header->set; set++) {
			/* dump tag */
			id = 0x0;
			ramidx = ((id & 0xFF) << 24) | ((way & 0x3) << 18) |
				((set & 0xFF) << 6);

			write_ramindex_ca72(ramidx);
			dump->ramidx = ramidx;
			dump->tag[0] = read_il1_data0();
			dump->tag[1] = read_il1_data1();

			for (bank_v = 0; bank_v <= 0x7; bank_v++) {
				/* dump data */
				id = 0x1;
				/* VA[3] Upper or lower doubleword within
				 * the quadword.
				 */
				ramidx = ((id & 0xFF) << 24) |
					((way & 0x3) << 18)  |
					((set & 0xFF) << 6)  |
					((bank_v & 0x7) << 3);

				write_ramindex_ca72(ramidx);
				dump->data[2 * bank_v] = read_il1_data0();
				dump->data[2 * bank_v + 1] = read_il1_data1();
			}
			dump++;
			ptr = (uint32_t *)dump;
		}
	}

	*ptr = 0x12357BD;
	*(ptr + 1) = ~(*ptr);

	// printf("magic:0x%x, 0x%x\n", *ptr, *(ptr + 1));
	ptr += 2;

	if ((unsigned long) ptr > (addr + size)) {
		printf("dump cache buffer overflow....\n");
		return -1;
	}

	header->dump_size = (uint64_t)ptr - addr;
	flush_dcache_range(addr, size);
	return 0;
}

int mt_icache_dump(unsigned long addr, unsigned long size)
{
	static int region_added;
	uint64_t midr_partnum;
	int ret = 0;

	midr_partnum = (read_midr() >> 4) & 0xFFF;

	/* Log starts here... */
	// set_uart_flag();

	if (!region_added) {
		printf("mmap cache dump buffer : 0x%lx, 0x%lx\n\r",
			addr, size);

		mmap_add_region(addr & ~(PAGE_SIZE_2MB_MASK),
				addr & ~(PAGE_SIZE_2MB_MASK),
				PAGE_SIZE_2MB,
				MT_MEMORY | MT_RW | MT_NS);

		/* re-fill translation table */
		init_xlat_tables();
		region_added = 1;
		printf("cache dump buffer (2MB aligned): 0x%lx, 0x%x\n\r",
		       addr & ~(PAGE_SIZE_2MB_MASK), PAGE_SIZE_2MB);
	}

	switch (midr_partnum) {
	case 0xD03: /* cortex ca53 */
		printf("Cortex-A53 icache dump doesn't support currently\n");
		ret = 0;
		break;

	case 0xD08: /* cortex ca72 */
		printf("Cortex-A72 icache dump start ...\n");
		ret = mt_icache_dump_ca72(addr, size);
		break;

	default:
		ret = -1;
		break;
	}

	return ret;
}
