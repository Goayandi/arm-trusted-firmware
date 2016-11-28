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

#include <aarch64/plat_helpers.h>
#include <arch_helpers.h>
#include <arm_gic.h>
#include <assert.h>
#include <bakery_lock.h>
#include <cci.h>
#include <debug.h>
#include <mmio.h>
#include <platform.h>
#include <psci.h>
#include <errno.h>

#include <mt_cpuxgpt.h> //  generic_timer_backup()
#include <plat_def.h>
#include <plat_private.h>
#include <platform_def.h>
#include <power.h>
#include <scu.h>

extern void dfd_setup(void);
extern void gic_cpuif_init(void);
extern void gic_rdist_save(void);
extern void gic_rdist_restore(void);

unsigned long g_dormant_log_base = 0;
void dormant_log(int tag)
{
        int cpuid = plat_core_pos_by_mpidr(read_mpidr());

        if (g_dormant_log_base == 0)
                return;

        ((long *)g_dormant_log_base)[cpuid] = tag | (cpuid << 12);

        dsb();
}

#if SPMC_SPARK2
void set_cpu_retention_control(int retention_value)
{
	uint64_t cpuectlr;

	cpuectlr = read_cpuectlr();
	cpuectlr = ((cpuectlr >> 3) << 3);
	cpuectlr |= retention_value;
	write_cpuectlr(cpuectlr);
}

void plat_pre_wdt_dump(void)
{
	set_cpu_retention_control(0x0);
}
#endif

static struct _el3_dormant_data {
        unsigned long mp0_l2actlr_el1;
        unsigned long mp0_l2ectlr_el1;
        unsigned long mp0_l2rstdisable;
        unsigned long storage[32];
} el3_dormant_data[1];

static void plat_save_el3_dormant_data()
{
	struct _el3_dormant_data *p = &el3_dormant_data[0];

	p->mp0_l2actlr_el1 = read_l2actlr();
	p->mp0_l2ectlr_el1 = read_l2ectlr();

	//backup L2RSTDISABLE and set as "not disable L2 reset"
	p->mp0_l2rstdisable = mmio_read_32(MP0_CA7L_CACHE_CONFIG);
	mmio_write_32(MP0_CA7L_CACHE_CONFIG,
		      mmio_read_32(MP0_CA7L_CACHE_CONFIG) & ~L2RSTDISABLE);
}

static void plat_restore_el3_dormant_data()
{
	struct _el3_dormant_data *p = &el3_dormant_data[0];

	if (p->mp0_l2actlr_el1 == 0 && p->mp0_l2ectlr_el1==0)
		panic();
	write_l2actlr(p->mp0_l2actlr_el1);
	write_l2ectlr(p->mp0_l2ectlr_el1);

	//restore L2RSTDIRSABLE
	mmio_write_32(MP0_CA7L_CACHE_CONFIG,
		      (mmio_read_32(MP0_CA7L_CACHE_CONFIG) & ~L2RSTDISABLE)
		      | (p->mp0_l2rstdisable & L2RSTDISABLE));
}

#if ERRATA_A53_826319
int workaround_826319(unsigned long mpidr)
{
        unsigned long l2actlr;

        /** only apply on 1st CPU of each cluster **/
        if (mpidr & MPIDR_CPU_MASK)
                return 0;

        /** CONFIG_ARM_ERRATA_826319=y (for 6595/6752)
         * Prog CatB Rare,
         * System might deadlock if a write cannot complete until read data is accepted
         * worksround: (L2ACTLR[14]=0, L2ACTLR[3]=1).
         * L2ACTLR must be written before MMU on and any ACE, CHI or ACP traffic.
         **/
        l2actlr = read_l2actlr();
        l2actlr = (l2actlr & ~(1<<14)) | (1<<3);
        write_l2actlr(l2actlr);

        return 0;
}
#else //#if ERRATA_A53_826319
#define workaround_826319() do {} while(0)
#endif //#if ERRATA_A53_826319

#if ERRATA_A53_836870
int workaround_836870(unsigned long mpidr)
{
        unsigned long cpuactlr;

        /** CONFIG_ARM_ERRATA_836870=y (for 6595/6752/6735, prior to r0p4)
         * Prog CatC,
         * Non-allocating reads might prevent a store exclusive from passing
         * worksround: set the CPUACTLR.DTAH bit.
         * The CPU Auxiliary Control Register can be written only when the system
         * is idle. ARM recommends that you write to this register after a powerup
         * reset, before the MMU is enabled, and before any ACE or ACP traffic
         * begins.
         **/
        cpuactlr = read_cpuactlr();
        cpuactlr = cpuactlr | (1<<24);
        write_cpuactlr(cpuactlr);

        return 0;
}
#else //#if ERRATA_A53_836870
#define workaround_836870() do {} while(0)
#endif //#if ERRATA_A53_836870

int clear_cntvoff(unsigned long mpidr)
{
    unsigned int scr_val, val;

    /**
     * Clear CNTVOFF in ATF for ARMv8 platform
     **/
    val = 0;

    /* set NS_BIT */
    scr_val = read_scr();
    write_scr(scr_val | SCR_NS_BIT);

    write_cntvoff_el2(val);

    /* write back the original value */
    write_scr(scr_val);

//    printf("[0x%X] cntvoff_el2=0x%x\n",mpidr, read_cntvoff_el2());
    return val;
}

/*******************************************************************************
 * Private FVP function to program the mailbox for a cpu before it is released
 * from reset.
 ******************************************************************************/
static void plat_program_mailbox(uint64_t mpidr, uint64_t address)
{
	uint64_t linear_id;
	mailbox_t *plat_mboxes;

	linear_id = plat_core_pos_by_mpidr(mpidr);
	plat_mboxes = (mailbox_t *)MBOX_BASE;
	plat_mboxes[linear_id].value = address;
	flush_dcache_range((unsigned long) &plat_mboxes[linear_id],
			   sizeof(unsigned long));
}

#if	ENABLE_PLAT_COMPAT
/*******************************************************************************
 * Private FVP function which is used to determine if any platform actions
 * should be performed for the specified affinity instance given its
 * state. Nothing needs to be done if the 'state' is not off or if this is not
 * the highest affinity level which will enter the 'state'.
 ******************************************************************************/
static int32_t plat_do_plat_actions(unsigned int afflvl, unsigned int state)
{
	unsigned int max_phys_off_afflvl;

	assert(afflvl <= MPIDR_AFFLVL2);

	if (state != PSCI_STATE_OFF)
		return -EAGAIN;

	/*
	 * Find the highest affinity level which will be suspended and postpone
	 * all the platform specific actions until that level is hit.
	 */
	max_phys_off_afflvl = psci_get_max_phys_off_afflvl();
	assert(max_phys_off_afflvl != PSCI_INVALID_DATA);
	if (afflvl != max_phys_off_afflvl)
		return -EAGAIN;

	return 0;
}

/*******************************************************************************
 * FVP handler called when an affinity instance is about to enter standby.
 ******************************************************************************/
void plat_affinst_standby(unsigned int power_state)
{
	/*
	 * Enter standby state
	 * dsb is good practice before using wfi to enter low power states
	 */
	INFO("in %s\n", __FUNCTION__);
	dsb();
	wfi();
}

/*******************************************************************************
 * FVP handler called when an affinity instance is about to be turned on. The
 * level and mpidr determine the affinity instance.
 ******************************************************************************/
int32_t plat_affinst_on(uint64_t mpidr,
		   uint64_t sec_entrypoint,
		   uint32_t afflvl,
		   uint32_t state)
{
	int rc = PSCI_E_SUCCESS;
	unsigned long linear_id;

	/*
	 * It's possible to turn on only affinity level 0 i.e. a cpu
	 * on the FVP. Ignore any other affinity level.
	 */
	if (afflvl != MPIDR_AFFLVL0)
		return rc;

	plat_program_mailbox(mpidr, sec_entrypoint);

	linear_id = plat_core_pos_by_mpidr(mpidr);
	extern void bl31_warm_entrypoint(void);

	if (linear_id >= 8) {
#if SPMC_SW_MODE
		rc  = big_spmc_sw_pwr_on(1<<(linear_id-8));
#else
		rc = power_on_big(linear_id);
#endif
		INFO("Yes, rc = %d\n", rc);
		return rc;
	} else if (linear_id >= 4) {
		if (!(little_on & 0xF0)){
			INFO("Enable CPUSYS1\n");
			power_on_little_cl(1);
		}
		mmio_write_32(MP1_MISC_CONFIG3, mmio_read_32(MP1_MISC_CONFIG3) | 0x0000F000);
		// mmio_write_32(MP1_MISC_CONFIG_BOOT_ADDR(linear_id-4), (unsigned long)bl31_on_entrypoint);
		mmio_write_32(MP1_MISC_CONFIG_BOOT_ADDR(linear_id-4), (unsigned long) bl31_warm_entrypoint);
		INFO("mt_on_1, entry %x\n", mmio_read_32(MP1_MISC_CONFIG_BOOT_ADDR(linear_id-4)));
	} else {
		if(!(little_on & 0x0F)) {
			INFO("Enable CPUSYS0\n");
			power_on_little_cl(0);
		}
		mmio_write_32(MP0_MISC_CONFIG3, mmio_read_32(MP0_MISC_CONFIG3) | 0x0000F000);
		// mmio_write_32(MP0_MISC_CONFIG_BOOT_ADDR(linear_id), (unsigned long)bl31_on_entrypoint);
		mmio_write_32(MP0_MISC_CONFIG_BOOT_ADDR(linear_id), (unsigned long) bl31_warm_entrypoint);
		INFO("mt_on_0, entry %x\n", mmio_read_32(MP0_MISC_CONFIG_BOOT_ADDR(linear_id)));
	}

	/* control CPU power */
	printf("... power on CPU%ld ...\n", linear_id);
	power_on_little(linear_id);

	return rc;
}

extern void gic_dist_save(void);
extern void gic_dist_restore(void);

/*******************************************************************************
 * FVP handler called when an affinity instance is about to be turned off. The
 * level and mpidr determine the affinity instance. The 'state' arg. allows the
 * platform to decide whether the cluster is being turned off and take apt
 * actions.
 *
 * CAUTION: There is no guarantee that caches will remain turned on across calls
 * to this function as each affinity level is dealt with. So do not write & read
 * global variables across calls. It will be wise to do flush a write to the
 * global to prevent unpredictable results.
 ******************************************************************************/
void plat_affinst_off(uint32_t afflvl, uint32_t state)
{
	/* Determine if any platform actions need to be executed */
	if (plat_do_plat_actions(afflvl, state) == -EAGAIN)
		return;

        unsigned long mpidr = read_mpidr_el1();

	/*
	 * If execution reaches this stage then this affinity level will be
	 * suspended. Perform at least the cpu specific actions followed the
	 * cluster specific operations if applicable.
	 */
	// plat_cpu_pwrdwn_common();

	/*
	 * Prevent interrupts from spuriously waking up
	 * this cpu
	 */
	gic_rdist_save();
	gic_cpuif_deactivate(BASE_GICC_BASE);

	/*
	 *
	 */
	unsigned int linear_id = plat_core_pos_by_mpidr(mpidr);
	pend_off = linear_id;

#if SPMC_SPARK2
	INFO("%s core:%d(callee) disable SPARK-core-side\n",__FUNCTION__, linear_id);
	// turn off spark2 cpu-side by callee
	set_cpu_retention_control(0);
#endif

	/*
	 * Perform cluster power down
	 */
	if (afflvl != MPIDR_AFFLVL0) {
		// plat_cluster_pwrdwn_common();

		/*
		 * Disable coherency if this cluster is to be
		 * turned off
		 */
		if (linear_id < 8) {
			/* move to power_off_big() if (linear_id >= 8) */
			plat_cci_disable();
			INFO("%s: cci_disable_cluster_coherency(%d)\n", __FUNCTION__, linear_id);
			disable_scu(mpidr);
			INFO("%s: disable_scu(%d)\n", __FUNCTION__, linear_id);
		}
	}
}

/*******************************************************************************
 * FVP handler called when an affinity instance is about to be suspended. The
 * level and mpidr determine the affinity instance. The 'state' arg. allows the
 * platform to decide whether the cluster is being turned off and take apt
 * actions.
 *
 * CAUTION: There is no guarantee that caches will remain turned on across calls
 * to this function as each affinity level is dealt with. So do not write & read
 * global variables across calls. It will be wise to do flush a write to the
 * global to prevent unpredictable results.
 ******************************************************************************/
void plat_affinst_suspend(unsigned long sec_entrypoint,
			unsigned int afflvl,
			unsigned int state)
{
	/* Determine if any platform actions need to be executed. */
	if (plat_do_plat_actions(afflvl, state) == -EAGAIN)
		return;

	//set cpu0 as aa64 for cpu reset
	mmio_write_32(MP0_MISC_CONFIG3, mmio_read_32(MP0_MISC_CONFIG3) | (1<<12));

        unsigned long mpidr = read_mpidr_el1();
#if SPMC_SPARK2
	uint64_t linear_id;

	linear_id = plat_core_pos_by_mpidr(mpidr);

	set_cpu_retention_control(0);

	little_spark2_core(linear_id, 0);
	while(mmio_read_32(SPM_CPU_RET_STATUS) & (1 << linear_id));
#endif

	/* Program the jump address for the target cpu */
	plat_program_mailbox(read_mpidr_el1(), sec_entrypoint);

	/* Perform the common cluster specific operations */
	if (afflvl != MPIDR_AFFLVL0) {
		/* Perform the common cpu specific operations */
		// plat_cpu_pwrdwn_common();
		// plat_cluster_pwrdwn_common();

		gic_cpuif_deactivate(BASE_GICC_BASE);
		plat_cci_disable();

		disable_scu(mpidr);
		plat_save_el3_dormant_data();
		generic_timer_backup();
		gic_dist_save();
		gic_rdist_save();
	}

	return;
}

/*******************************************************************************
 * FVP handler called when an affinity instance has just been powered on after
 * being turned off earlier. The level and mpidr determine the affinity
 * instance. The 'state' arg. allows the platform to decide whether the cluster
 * was turned off prior to wakeup and do what's necessary to setup it up
 * correctly.
 ******************************************************************************/
void plat_affinst_on_finish(unsigned int afflvl, unsigned int state)
{
	unsigned long mpidr = read_mpidr_el1();

	/* Determine if any platform actions need to be executed. */
	if (plat_do_plat_actions(afflvl, state) == -EAGAIN)
		return;

	/* Perform the common cluster specific operations */
	if (afflvl != MPIDR_AFFLVL0) {
		enable_scu(mpidr);
		INFO("%s: enable_scu()\n", __FUNCTION__);

		/* Enable coherency if this cluster was off */
		plat_cci_enable();
		INFO("%s: plat_cci_enable()\n", __FUNCTION__);
	}

#if SPMC_SPARK2
	/* ---------------------------------------------
	 * CPU retention control.
	 * Set the CPUECTLR[2:0] = 0x01
	 * ---------------------------------------------
	 */
	set_cpu_retention_control(0x1);
#endif

	/*
	 * Ignore the state passed for a cpu. It could only have
	 * been off if we are here.
	 */
#if ERRATA_A53_836870
	workaround_836870(mpidr);
#endif

	/*
	 * clear CNTVOFF, for slave cores
	 */
	clear_cntvoff(mpidr);

	/* Zero the jump address in the mailbox for this cpu */
	plat_program_mailbox(read_mpidr_el1(), 0);

	gic_cpuif_init();
	gic_rdist_restore();

	/* TODO: This setup is needed only after a cold boot */
	// arm_gic_pcpu_distif_setup();

	enable_ns_access_to_cpuectlr();
}

/*******************************************************************************
 * FVP handler called when an affinity instance has just been powered on after
 * having been suspended earlier. The level and mpidr determine the affinity
 * instance.
 * TODO: At the moment we reuse the on finisher and reinitialize the secure
 * context. Need to implement a separate suspend finisher.
 ******************************************************************************/
void plat_affinst_suspend_finish(unsigned int afflvl, unsigned int state)
{
	unsigned long mpidr = read_mpidr_el1();

	if (plat_do_plat_actions(afflvl, state) == -EAGAIN)
		return;

	if (afflvl != MPIDR_AFFLVL0) {
		plat_restore_el3_dormant_data();
		dfd_setup();
		gic_setup();
		gic_dist_restore();
		gic_rdist_restore();

		enable_scu(mpidr);

		/* Enable coherency if this cluster was off */
		plat_cci_enable();

		/* Enable the gic cpu interface */
		// arm_gic_cpuif_setup();
		// gic_cpuif_setup(get_plat_config()->gicc_base);
	}

#if SPMC_SPARK2
	uint64_t linear_id;

	linear_id = plat_core_pos_by_mpidr(mpidr);
	little_spark2_setldo(linear_id);
	little_spark2_core(linear_id, 1);

	/* ---------------------------------------------
	 * CPU retention control.
	 * Set the CPUECTLR[2:0] = 0x01
	 * ---------------------------------------------
	 */
	set_cpu_retention_control(0x1);
#endif
	/*
	 * Ignore the state passed for a cpu. It could only have
	 * been off if we are here.
	 */
#if ERRATA_A53_836870
	workaround_836870(mpidr);
#endif

	/*
	 * clear CNTVOFF, for slave cores
	 */
	clear_cntvoff(mpidr);

	/* Zero the jump address in the mailbox for this cpu */
	plat_program_mailbox(read_mpidr_el1(), 0);

	enable_ns_access_to_cpuectlr();

	return;
}

/*******************************************************************************
 * FVP handlers to shutdown/reboot the system
 ******************************************************************************/
static void __dead2 plat_system_off(void)
{
	/* Write the System Configuration Control Register */
	mmio_write_32(VE_SYSREGS_BASE + V2M_SYS_CFGCTRL,
		CFGCTRL_START | CFGCTRL_RW | CFGCTRL_FUNC(FUNC_SHUTDOWN));
	wfi();
	ERROR("MT6797 System Off: operation not handled.\n");
	panic();
}

static void __dead2 plat_system_reset(void)
{

	/* Write the System Configuration Control Register */
	INFO("MT6797 System Reset\n");
	mmio_clrsetbits_32(MTK_WDT_BASE,
		(MTK_WDT_MODE_DUAL_MODE | MTK_WDT_MODE_IRQ),
		MTK_WDT_MODE_KEY);

	mmio_setbits_32(MTK_WDT_BASE, (MTK_WDT_MODE_KEY | MTK_WDT_MODE_EXTEN));
	mmio_setbits_32(MTK_WDT_SWRST, MTK_WDT_SWRST_KEY);

	wfi();
	ERROR("MT6797 System Reset: operation not handled.\n");
	panic();
}

/*******************************************************************************
 * Export the platform handlers to enable psci to invoke them
 ******************************************************************************/
static const plat_pm_ops_t plat_plat_pm_ops = {
	.affinst_standby = plat_affinst_standby,
	.affinst_on = plat_affinst_on,
	.affinst_off = plat_affinst_off,
	.affinst_suspend = plat_affinst_suspend,
	.affinst_on_finish = plat_affinst_on_finish,
	.affinst_suspend_finish = plat_affinst_suspend_finish,
	.system_off = plat_system_off,
	.system_reset = plat_system_reset
};

/*******************************************************************************
 * Export the platform specific power ops & initialize the plat power controller
 ******************************************************************************/
int platform_setup_pm(const plat_pm_ops_t **plat_ops)
{
	*plat_ops = &plat_plat_pm_ops;
	return 0;
}
#else

#define	MTK_PWR_LVL0	0
#define	MTK_PWR_LVL1	1
#define	MTK_PWR_LVL2	2

/* Macros to read the MTK power domain state */
#define	MTK_CORE_PWR_STATE(state)	(state)->pwr_domain_state[MTK_PWR_LVL0]
#define	MTK_CLUSTER_PWR_STATE(state)	(state)->pwr_domain_state[MTK_PWR_LVL1]
#define	MTK_SYSTEM_PWR_STATE(state)	((PLAT_MAX_PWR_LVL > MTK_PWR_LVL1) ? \
				 (state)->pwr_domain_state[MTK_PWR_LVL2] : 0)

extern void gic_dist_save(void);
extern void gic_dist_restore(void);

void platform_cpu_standby(plat_local_state_t cpu_state)
{
#if 0
	unsigned int scr;

	scr = read_scr_el3();
	write_scr_el3(scr | SCR_IRQ_BIT);
	isb();
	dsb();
	wfi();
	write_scr_el3(scr);
#endif
}

extern void bl31_warm_entrypoint(void);

static uintptr_t secure_entrypoint;

int platform_pwr_domain_on(u_register_t mpidr)
{
	int rc = PSCI_E_SUCCESS;
	unsigned long linear_id;

	plat_program_mailbox(mpidr, secure_entrypoint);

	linear_id = plat_core_pos_by_mpidr(mpidr);

	if (linear_id >= 8) {
#if SPMC_SW_MODE
		rc  = big_spmc_sw_pwr_on(1<<(linear_id-8));
#else
		rc = power_on_big(linear_id);
#endif
		INFO("Yes, rc = %d\n", rc);
		return rc;
	} else if (linear_id >= 4) {
		if (!(little_on & 0xF0)) {
			INFO("Enable CPUSYS1\n");
			power_on_little_cl(1);
		}
		mmio_write_32(MP1_MISC_CONFIG3, mmio_read_32(MP1_MISC_CONFIG3) | 0x0000F000);
		mmio_write_32(MP1_MISC_CONFIG_BOOT_ADDR(linear_id-4), (unsigned long) bl31_warm_entrypoint);
		INFO("mt_on_1, entry %x\n", mmio_read_32(MP1_MISC_CONFIG_BOOT_ADDR(linear_id-4)));
	} else {
		if (!(little_on & 0x0F)) {
			INFO("Enable CPUSYS0\n");
			power_on_little_cl(0);
		}
		mmio_write_32(MP0_MISC_CONFIG3, mmio_read_32(MP0_MISC_CONFIG3) | 0x0000F000);
		mmio_write_32(MP0_MISC_CONFIG_BOOT_ADDR(linear_id), (unsigned long) bl31_warm_entrypoint);
		INFO("mt_on_0, entry %x\n", mmio_read_32(MP0_MISC_CONFIG_BOOT_ADDR(linear_id)));
	}

	/* control CPU power */
	INFO("... power on CPU%ld ...\n", linear_id);
	power_on_little(linear_id);

	return rc;
}

void platform_pwr_domain_off(const psci_power_state_t *state)
{
}

void platform_pwr_domain_suspend(const psci_power_state_t *state)
{

	mmio_write_32(MP0_MISC_CONFIG3, mmio_read_32(MP0_MISC_CONFIG3) | (1<<12));

        unsigned long mpidr = read_mpidr_el1();
#if SPMC_SPARK2
	uint64_t linear_id;

	linear_id = plat_core_pos_by_mpidr(mpidr);

	set_cpu_retention_control(0);

	little_spark2_core(linear_id, 0);
	while(mmio_read_32(SPM_CPU_RET_STATUS) & (1 << linear_id));
#endif

	/* Program the jump address for the target cpu */
	plat_program_mailbox(read_mpidr_el1(), secure_entrypoint);

	/* Perform the common cluster specific operations */
	// if (afflvl != MPIDR_AFFLVL0) {
	if ((MTK_CLUSTER_PWR_STATE(state) == MTK_LOCAL_STATE_OFF) ||
		(MTK_SYSTEM_PWR_STATE(state) == MTK_LOCAL_STATE_OFF)) {

		gic_cpuif_deactivate(BASE_GICC_BASE);
		plat_cci_disable();

		disable_scu(mpidr);
		plat_save_el3_dormant_data();
		generic_timer_backup();
		gic_dist_save();
		gic_rdist_save();
	}

	return;
}

void platform_pwr_domain_on_finish(const psci_power_state_t *state)
{
	unsigned long mpidr = read_mpidr_el1();

	/* Perform the common cluster specific operations */
	if ((MTK_CLUSTER_PWR_STATE(state) == MTK_LOCAL_STATE_OFF) ||
		(MTK_SYSTEM_PWR_STATE(state) == MTK_LOCAL_STATE_OFF)) {
		enable_scu(mpidr);
		INFO("%s: enable_scu()\n", __FUNCTION__);

		/* Enable coherency if this cluster was off */
		plat_cci_enable();
		INFO("%s: plat_cci_enable()\n", __FUNCTION__);
	}

#if SPMC_SPARK2
	/* ---------------------------------------------
	 * CPU retention control.
	 * Set the CPUECTLR[2:0] = 0x01
	 * ---------------------------------------------
	 */
	set_cpu_retention_control(0x1);
#endif

	/*
	 * Ignore the state passed for a cpu. It could only have
	 * been off if we are here.
	 */
#if ERRATA_A53_836870
	workaround_836870(mpidr);
#endif

	/*
	 * clear CNTVOFF, for slave cores
	 */
	clear_cntvoff(mpidr);

	/* Zero the jump address in the mailbox for this cpu */
	plat_program_mailbox(read_mpidr_el1(), 0);

	gic_cpuif_init();
	gic_rdist_restore();

	// enable_ns_access_to_cpuectlr();
}

void platform_pwr_domain_suspend_finish(const psci_power_state_t *state)
{
	unsigned long mpidr = read_mpidr_el1();

	if ((MTK_CLUSTER_PWR_STATE(state) == MTK_LOCAL_STATE_OFF) ||
		(MTK_SYSTEM_PWR_STATE(state) == MTK_LOCAL_STATE_OFF)) {
		plat_restore_el3_dormant_data();
		dfd_setup();
		gic_setup();
		gic_dist_restore();
		gic_rdist_restore();

		enable_scu(mpidr);

		/* Enable coherency if this cluster was off */
		plat_cci_enable();

		/* Enable the gic cpu interface */
		// arm_gic_cpuif_setup();
		// gic_cpuif_setup(get_plat_config()->gicc_base);
	}

#if SPMC_SPARK2
	uint64_t linear_id;

	linear_id = plat_core_pos_by_mpidr(mpidr);
	little_spark2_setldo(linear_id);
	little_spark2_core(linear_id, 1);

	/* ---------------------------------------------
	 * CPU retention control.
	 * Set the CPUECTLR[2:0] = 0x01
	 * ---------------------------------------------
	 */
	set_cpu_retention_control(0x1);
#endif
	/*
	 * Ignore the state passed for a cpu. It could only have
	 * been off if we are here.
	 */
#if ERRATA_A53_836870
	workaround_836870(mpidr);
#endif

	/*
	 * clear CNTVOFF, for slave cores
	 */
	clear_cntvoff(mpidr);

	/* Zero the jump address in the mailbox for this cpu */
	plat_program_mailbox(read_mpidr_el1(), 0);

	enable_ns_access_to_cpuectlr();

	return;
}

__dead2 void platform_system_off(void)
{
	mmio_write_32(VE_SYSREGS_BASE + V2M_SYS_CFGCTRL,
		CFGCTRL_START | CFGCTRL_RW | CFGCTRL_FUNC(FUNC_SHUTDOWN));
	wfi();
	ERROR("MT6797 System Off: operation not handled.\n");
	panic();
}

__dead2 void platform_system_reset(void)
{
	INFO("MT6797 System Reset\n");
	mmio_clrsetbits_32(MTK_WDT_BASE,
		(MTK_WDT_MODE_DUAL_MODE | MTK_WDT_MODE_IRQ),
		MTK_WDT_MODE_KEY);

	mmio_setbits_32(MTK_WDT_BASE, (MTK_WDT_MODE_KEY | MTK_WDT_MODE_EXTEN));
	mmio_setbits_32(MTK_WDT_SWRST, MTK_WDT_SWRST_KEY);

	wfi();
	ERROR("MT6797 System Reset: operation not handled.\n");
	panic();
}

int32_t platform_validate_power_state(unsigned int power_state,
                                   psci_power_state_t *req_state)
{
	int pstate = psci_get_pstate_type(power_state);
	int pwr_lvl = psci_get_pstate_pwrlvl(power_state);
	int i;

	assert(req_state);

	if (pwr_lvl > PLAT_MAX_PWR_LVL)
		return PSCI_E_INVALID_PARAMS;

	/* Sanity check the requested state */
	if (pstate == PSTATE_TYPE_STANDBY) {
		/*
		 * It's possible to enter standby only on power level 0
		 * Ignore any other power level.
		 */
		if (pwr_lvl != 0)
			return PSCI_E_INVALID_PARAMS;

		req_state->pwr_domain_state[MTK_PWR_LVL0] =
					MTK_LOCAL_STATE_RET;
	} else {
		for (i = 0; i <= pwr_lvl; i++)
			req_state->pwr_domain_state[i] =
					MTK_LOCAL_STATE_OFF;
	}

	/*
	 * We expect the 'state id' to be zero.
	 */
	if (psci_get_pstate_id(power_state))
		return PSCI_E_INVALID_PARAMS;

	return PSCI_E_SUCCESS;
}

int platform_validate_ns_entrypoint(uintptr_t entrypoint)
{
	return PSCI_E_SUCCESS;
}

void platform_get_sys_suspend_power_state(psci_power_state_t *req_state)
{
}

/*******************************************************************************
 * Export the platform handlers to enable psci to invoke them
 ******************************************************************************/
static const plat_psci_ops_t mt_plat_psci_ops = {
        .cpu_standby                    = platform_cpu_standby,
        .pwr_domain_on                  = platform_pwr_domain_on,
        .pwr_domain_off                 = platform_pwr_domain_off,
        .pwr_domain_suspend             = platform_pwr_domain_suspend,
        .pwr_domain_on_finish           = platform_pwr_domain_on_finish,
        .pwr_domain_suspend_finish      = platform_pwr_domain_suspend_finish,
        .system_off                     = platform_system_off,
        .system_reset                   = platform_system_reset,
        .validate_power_state           = platform_validate_power_state,
        .validate_ns_entrypoint         = platform_validate_ns_entrypoint,
        .get_sys_suspend_power_state    = platform_get_sys_suspend_power_state,
};

/*******************************************************************************
 * Export the platform specific power ops and initialize Power Controller
 ******************************************************************************/
int plat_setup_psci_ops(uintptr_t sec_entrypoint,
                        const plat_psci_ops_t **psci_ops)
{
	/*
	 * Initialize PSCI ops struct
	 */
        *psci_ops = &mt_plat_psci_ops;
        secure_entrypoint = sec_entrypoint;

        return 0;
}

#endif
