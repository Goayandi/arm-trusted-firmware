#ifndef __MT_CACHE_H__
#define __MT_CACHE_H__

int mt_icache_dump(uint64_t addr, uint64_t size);
extern void mt_write_ramindex_ca72(unsigned long ramidx);
extern uint64_t mt_read_il1_data0_ca72(void);
extern uint64_t mt_read_il1_data1_ca72(void);

#endif //end of __MT_CACHE_H__
