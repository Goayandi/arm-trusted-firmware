#ifndef POWER_H
#define POWER_H


#ifndef __ASSEMBLY__

int power_on_cl3(void);
int power_off_cl3(void);
int power_on_big(const unsigned int no);
int power_off_big(const unsigned int no);
int big_spmc_status(int select);
int big_spmc_sw_pwr_cntrl_disable(int select);
int big_spmc_sw_pwr_off(int select);
int big_spmc_sw_pwr_on(int select);
int big_spmc_sw_pwr_seq_en(int select);
void big_spark2_setldo(unsigned int cpu0_amuxsel, unsigned int cpu1_amuxsel);
int big_spark2_core(unsigned int core, unsigned int sw);
void big_spmc_info(void);
void power_off_little(unsigned int core);
void power_on_little_cl(unsigned int cl);
void power_off_little_cl(unsigned int cl);
void power_on_little(unsigned int core);
int little_spark2_core(unsigned int core, unsigned int sw);
int little_spark2_setldo(unsigned int core);
extern char big_on;
extern char little_on;
extern int pend_off;
#define DSB __asm__("dsb sy":::"memory")
unsigned int spmc_sw_mode_enable;

extern void dfd_setup(void);
extern void bl31_warm_entrypoint(void);

/*
 * Declarations of linker defined symbols which will help us find the layout
 * of trusted SRAM
 */
extern unsigned long __RO_START__;
extern unsigned long __RO_END__;
extern unsigned long __COHERENT_RAM_START__;
extern unsigned long __COHERENT_RAM_END__;

#endif

#define FPGA_SMP	1
#define SPMC_SW_MODE	0
#define SPMC_DEBUG	1
#define SPMC_DVT	0
#define SPMC_DVT_UDELAY	0
#define SPMC_SPARK2	1

#endif
