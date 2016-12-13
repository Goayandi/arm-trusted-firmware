#include <bakery_lock.h>
#include <mmio.h>

#include <mt_cpuxgpt.h>
#include <spm.h>

DEFINE_BAKERY_LOCK(spm_lock);
static int spm_hotplug_ready __section("tzfw_coherent_mem");
static int spm_mcdi_ready __section("tzfw_coherent_mem");
static int spm_suspend_ready __section("tzfw_coherent_mem");

void spm_lock_init(void)
{
	bakery_lock_init(&spm_lock);
}

void spm_lock_get(void)
{
	bakery_lock_get(&spm_lock);
}

void spm_lock_release(void)
{
	bakery_lock_release(&spm_lock);
}

int is_mcdi_ready(void)
{
	return spm_mcdi_ready;
}

int is_hotplug_ready(void)
{
	return spm_hotplug_ready;
}

int is_suspend_ready(void)
{
	return spm_suspend_ready;
}

void set_mcdi_ready(void)
{
	spm_mcdi_ready = 1;
	spm_hotplug_ready = 0;
	spm_suspend_ready = 0;
}

void set_hotplug_ready(void)
{
	spm_mcdi_ready = 0;
	spm_hotplug_ready = 1;
	spm_suspend_ready = 0;
}

void set_suspend_ready(void)
{
	spm_mcdi_ready = 0;
	spm_hotplug_ready = 0;
	spm_suspend_ready = 1;
}

void clear_all_ready(void)
{
	spm_mcdi_ready = 0;
	spm_hotplug_ready = 0;
	spm_suspend_ready = 0;
}

void spm_reset_and_init_pcm()
{
	uint32_t con1;
	int retry = 0, timeout = 5000;
	// uint32_t save_r1, save_r15, save_pcm_sta, save_irq_sta, dvfs_en;

	/* [Vcorefs] backup r0 to POWER_ON_VAL0 for MEM Ctrl should work during PCM reset */
	if (mmio_read_32(SPM_PCM_REG1_DATA) == 0x1) {
#if 0
		if (!vcorefs_sodi_rekick_lock()) {
			dvfs_en = !is_vcorefs_dvfs_enable();
			save_r1  = mmio_read_32(PCM_REG1_DATA);
			save_r15 = mmio_read_32(PCM_REG15_DATA);
			save_pcm_sta = mmio_read_32(PCM_FSM_STA);
			save_irq_sta = mmio_read_32(SPM_IRQ_STA);
			con1 = mmio_read_32(SPM_WAKEUP_EVENT_MASK);
			mmio_write_32(SPM_WAKEUP_EVENT_MASK, (con1 & ~(0x1)));

			/* don't change vcore and dram when dvfs enable reture false */
			mmio_write_32(SPM_SW_RSV_1, (spm_read(SPM_SW_RSV_1) & (~0xF)) | SPM_OFFLOAD);
			mmio_write_32(SPM_CPU_WAKEUP_EVENT, 1);
			if (is_vcorefs_feature_enable()) {
				__spm_backup_vcore_dvfs_dram_shuffle();
				while ((spm_read(SPM_IRQ_STA) & PCM_IRQ_ROOT_MASK_LSB) == 0) {
					if (retry > timeout) {
						pr_err("[VcoreFS] init state: r15=0x%x r1=0x%x pcmsta=0x%x irqsta=0x%x\n",
							save_r15, save_r1, save_pcm_sta, save_irq_sta);
						pr_err("[VcoreFS] CPU waiting F/W ack fail, PCM_FSM_STA: 0x%x, timeout: %d\n",
									spm_read(PCM_FSM_STA), timeout);
						pr_err("[VcoreFS] curr state: r15=0x%x r6=0x%x pcmsta=0x%x irqsta=0x%x\n",
							spm_read(PCM_REG15_DATA), spm_read(PCM_REG6_DATA),
							spm_read(PCM_FSM_STA), spm_read(SPM_IRQ_STA));

						spm_vcorefs_dump_dvfs_regs(NULL);
						BUG();
					}
					udelay(1);
					retry++;
				}
			}

#if SPM_AEE_RR_REC
			aee_rr_rec_spm_common_scenario_val(0);
#endif
			mmio_write_32(SPM_CPU_WAKEUP_EVENT, 0);
			mmio_write_32(SPM_WAKEUP_EVENT_MASK, con1);
			mmio_write_32(SPM_SW_RSV_1, (spm_read(SPM_SW_RSV_1) & (~0xF)) | SPM_CLEAN_WAKE_EVENT_DONE);
		}

		/* backup mem control from r0 to POWER_ON_VAL0 */
		if (mmio_read_32(SPM_POWER_ON_VAL0) != mmio_read_32(SPM_PCM_REG0_DATA)) {
			// spm_crit("VAL0 from 0x%x to 0x%x\n", spm_read(SPM_POWER_ON_VAL0), spm_read(PCM_REG0_DATA));
			mmio_write_32(SPM_POWER_ON_VAL0, mmio_read_32(SPM_PCM_REG0_DATA));
		}

		/* disable r0 and r7 to control power */
		mmio_write_32(SPM_PCM_PWR_IO_EN, 0);

		/* [Vcorefs] disable pcm timer after leaving FW */
		mmio_write_32(SPM_PCM_CON1, SPM_REGWR_CFG_KEY | (mmio_read_32(SPM_PCM_CON1) & ~PCM_TIMER_EN_LSB));

		/* backup vcore state from REG6[24:23] to RSV_5[1:0] */
		mmio_write_32(SPM_SW_RSV_5, (mmio_read_32(SPM_SW_RSV_5) & ~(0x3)) |
					((mmio_read_32(SPM_PCM_REG6_DATA) & SPM_VCORE_STA_REG) >> 23));
#endif
	}

	/* reset PCM */
	mmio_write_32(SPM_PCM_CON0, SPM_REGWR_CFG_KEY | PCM_CK_EN_LSB | PCM_SW_RESET_LSB);
	retry = 0;
	while ((mmio_read_32(SPM_PCM_FSM_STA) & 0x7fffff) != PCM_FSM_STA_DEF) {
		if (retry > timeout) {
			// spm_crit2("reset pcm PCM_FSM_STA=0x%x\n", spm_read(PCM_FSM_STA));
			// BUG(); /* PCM reset failed */
		}
		udelay(1);
		retry++;
	}
	mmio_write_32(SPM_PCM_CON0, SPM_REGWR_CFG_KEY | PCM_CK_EN_LSB);

	/* init PCM_CON0 (disable event vector) */
	mmio_write_32(SPM_PCM_CON0, SPM_REGWR_CFG_KEY | PCM_CK_EN_LSB | EN_IM_SLEEP_DVS_LSB);

	/* init PCM_CON1 (disable PCM timer but keep PCM WDT setting) */
	con1 = mmio_read_32(SPM_PCM_CON1) & (PCM_WDT_WAKE_MODE_LSB | PCM_WDT_EN_LSB);
	mmio_write_32(SPM_PCM_CON1, con1 | SPM_REGWR_CFG_KEY | EVENT_LOCK_EN_LSB |
		  SPM_SRAM_ISOINT_B_LSB | SPM_SRAM_SLEEP_B_LSB |
#if 0
		  (pcmdesc->replace ? 0 : IM_NONRP_EN_LSB) |
#endif
		  MIF_APBEN_LSB | SCP_APB_INTERNAL_EN_LSB);
}

void spm_kick_im_to_fetch(const struct pcm_desc *pcmdesc)
{
}

void spm_set_power_control(const struct pwr_ctrl *pwrctrl)
{
}

void spm_set_wakeup_event(const struct pwr_ctrl *pwrctrl)
{
}

void spm_kick_pcm_to_run(struct pwr_ctrl *pwrctrl)
{
}
