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
	INFO("%s: spm_mcdi_ready = %x\n", __func__, spm_mcdi_ready);
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
	INFO("%s: spm_mcdi_ready = %x\n", __func__, spm_mcdi_ready);
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

void spm_reset_and_init_pcm(void)
{
	uint32_t con1;
	int retry = 0, timeout = 5000;
	// uint32_t save_r1, save_r15, save_pcm_sta, save_irq_sta, dvfs_en;

	/* [Vcorefs] backup r0 to POWER_ON_VAL0 for MEM Ctrl should work
	 * during PCM reset
	 */
	if (mmio_read_32(SPM_PCM_REG1_DATA) == 0x1)
		// whatever needs to be done

	/* reset PCM */
	mmio_write_32(SPM_PCM_CON0,
		SPM_REGWR_CFG_KEY | PCM_CK_EN_LSB | PCM_SW_RESET_LSB);
	retry = 0;
	while ((mmio_read_32(SPM_PCM_FSM_STA) & 0x7fffff) != PCM_FSM_STA_DEF) {
		if (retry > timeout) {
			// spm_crit2("reset pcm PCM_FSM_STA=0x%x\n",
			//	spm_read(PCM_FSM_STA));
		}
		udelay(1);
		retry++;
	}
	mmio_write_32(SPM_PCM_CON0, SPM_REGWR_CFG_KEY | PCM_CK_EN_LSB);

	/* init PCM_CON0 (disable event vector) */
	mmio_write_32(SPM_PCM_CON0,
		SPM_REGWR_CFG_KEY | PCM_CK_EN_LSB | EN_IM_SLEEP_DVS_LSB);

	/* init PCM_CON1 (disable PCM timer but keep PCM WDT setting) */
	con1 = mmio_read_32(SPM_PCM_CON1) &
				(PCM_WDT_WAKE_MODE_LSB | PCM_WDT_EN_LSB);
	mmio_write_32(SPM_PCM_CON1,
			con1 | SPM_REGWR_CFG_KEY | EVENT_LOCK_EN_LSB |
			SPM_SRAM_ISOINT_B_LSB | SPM_SRAM_SLEEP_B_LSB |
#if 0
		  (pcmdesc->replace ? 0 : IM_NONRP_EN_LSB) |
#endif
			MIF_APBEN_LSB | SCP_APB_INTERNAL_EN_LSB);
}

void spm_kick_im_to_fetch(const struct pcm_desc *pcmdesc)
{
	uint32_t ptr, len, con0;

	/* tell IM where is PCM code (use slave mode if code existed) */
	ptr = (unsigned int)(unsigned long) pcmdesc->base;
	INFO("%s: pcm->base = %lx\n", __func__, (unsigned long) pcmdesc->base);

	len = pcmdesc->size - 1;
	if (mmio_read_32(SPM_PCM_IM_PTR) != ptr ||
		mmio_read_32(SPM_PCM_IM_LEN) != len || pcmdesc->sess > 2) {
		mmio_write_32(SPM_PCM_IM_PTR, ptr);
		mmio_write_32(SPM_PCM_IM_LEN, len);
	} else {
		mmio_write_32(SPM_PCM_CON1, mmio_read_32(SPM_PCM_CON1) |
						SPM_REGWR_CFG_KEY |
						IM_SLAVE_LSB);
	}

	/* kick IM to fetch (only toggle IM_KICK) */
	con0 = mmio_read_32(SPM_PCM_CON0) & ~(CON0_IM_KICK | CON0_PCM_KICK);

	mmio_write_32(SPM_PCM_CON0, con0 | SPM_REGWR_CFG_KEY | PCM_CK_EN_LSB |
			CON0_PCM_KICK);
	mmio_write_32(SPM_PCM_CON0, con0 | SPM_REGWR_CFG_KEY | PCM_CK_EN_LSB);
}

void spm_set_power_control(const struct pwr_ctrl *pwrctrl)
{
	/* set other SYS request mask */
	mmio_write_32(SPM_AP_STANDBY_CON, (!!pwrctrl->conn_apsrc_sel << 27) |
			(!!pwrctrl->conn_mask_b << 26) |
			(!!pwrctrl->md_apsrc0_sel << 25) |
			(!!pwrctrl->md_apsrc1_sel << 24) |
			(mmio_read_32(SPM_AP_STANDBY_CON) &
			SRCCLKENI_MASK_B_LSB) | /* bit23 */
			(!!pwrctrl->lte_mask_b << 22) |
			(!!pwrctrl->scp_req_mask_b << 21) |
			(!!pwrctrl->md2_req_mask_b << 20) |
			(!!pwrctrl->md1_req_mask_b << 19) |
			(!!pwrctrl->md_ddr_dbc_en << 18) |
			(!!pwrctrl->mcusys_idle_mask << 6) |
			(!!pwrctrl->mptop_idle_mask << 5) |
			(!!pwrctrl->mp3top_idle_mask << 4) |
			(!!pwrctrl->mp2top_idle_mask << 3) |
			(!!pwrctrl->mp1top_idle_mask << 2) |
			(!!pwrctrl->mp0top_idle_mask << 1) |
			(!!pwrctrl->wfi_op << 0));

	mmio_write_32(SPM_SRC_REQ, (!!pwrctrl->cpu_md_dvfs_sop_force_on << 16) |
			(!!pwrctrl->spm_flag_run_common_scenario << 10) |
			(!!pwrctrl->spm_flag_dis_vproc_vsram_dvs << 9) |
			(!!pwrctrl->spm_flag_keep_csyspwrupack_high << 8) |
			(!!pwrctrl->spm_ddren_req << 7) |
			(!!pwrctrl->spm_dvfs_force_down << 6) |
			(!!pwrctrl->spm_dvfs_req << 5) |
			(!!pwrctrl->spm_vrf18_req << 4) |
			(!!pwrctrl->spm_infra_req << 3) |
			(!!pwrctrl->spm_lte_req << 2) |
			(!!pwrctrl->spm_f26m_req << 1) |
			(!!pwrctrl->spm_apsrc_req << 0));

	mmio_write_32(SPM_SRC_MASK,
			(!!pwrctrl->conn_srcclkena_dvfs_req_mask_b << 31) |
			(!!pwrctrl->md_srcclkena_1_dvfs_req_mask_b << 30) |
			(!!pwrctrl->md_srcclkena_0_dvfs_req_mask_b << 29) |
			(!!pwrctrl->emi_bw_dvfs_req_mask << 28) |
			(!!pwrctrl->cpu_dvfs_req_mask << 27) |
			(!!pwrctrl->md1_dvfs_req_mask << 25) |
			(!!pwrctrl->md_vrf18_req_1_mask_b << 24) |
			(!!pwrctrl->md_vrf18_req_0_mask_b << 23) |
			(!!pwrctrl->md_ddr_en_1_mask_b << 22) |
			(!!pwrctrl->md_ddr_en_0_mask_b << 21) |
			(!!pwrctrl->md32_apsrcreq_infra_mask_b << 20) |
			(!!pwrctrl->conn_apsrcreq_infra_mask_b << 19) |
			(!!pwrctrl->md_apsrcreq_1_infra_mask_b << 18) |
			(!!pwrctrl->md_apsrcreq_0_infra_mask_b << 17) |
			(!!pwrctrl->srcclkeni_infra_mask_b << 16) |
			(!!pwrctrl->md32_srcclkena_infra_mask_b << 15) |
			(!!pwrctrl->conn_srcclkena_infra_mask_b << 14) |
			(!!pwrctrl->md_srcclkena_1_infra_mask_b << 13) |
			(!!pwrctrl->md_srcclkena_0_infra_mask_b << 12) |
			((pwrctrl->vsync_mask_b & 0x1f) << 7) |
			(!!pwrctrl->ccifmd_md2_event_mask_b << 6) |
			(!!pwrctrl->ccifmd_md1_event_mask_b << 5) |
			(!!pwrctrl->ccif1_to_ap_mask_b << 4) |
			(!!pwrctrl->ccif1_to_md_mask_b << 3) |
			(!!pwrctrl->ccif0_to_ap_mask_b << 2) |
			(!!pwrctrl->ccif0_to_md_mask_b << 1));

	mmio_write_32(SPM_SRC2_MASK,
			(!!pwrctrl->disp_od_req_mask_b << 27) |
			(!!pwrctrl->cpu_md_emi_dvfs_req_prot_dis << 26) |
			(!!pwrctrl->emi_boost_dvfs_req_mask_b << 25) |
			(!!pwrctrl->sdio_on_dvfs_req_mask_b << 24) |
			(!!pwrctrl->l1_c2k_rccif_wake_mask_b << 23) |
			(!!pwrctrl->ps_c2k_rccif_wake_mask_b << 22) |
			(!!pwrctrl->c2k_l1_rccif_wake_mask_b << 21) |
			(!!pwrctrl->c2k_ps_rccif_wake_mask_b << 20) |
			(!!pwrctrl->mfg_req_mask_b << 19) |
			(!!pwrctrl->disp1_req_mask_b << 18) |
			(!!pwrctrl->disp_req_mask_b << 17) |
			(!!pwrctrl->conn_ddr_en_mask_b << 16) |
			((pwrctrl->vsync_dvfs_halt_mask_b & 0x1f) << 11) |
			(!!pwrctrl->md2_ddr_en_dvfs_halt_mask_b << 10) |
			(!!pwrctrl->md1_ddr_en_dvfs_halt_mask_b << 9) |
			(!!pwrctrl->cpu_md_dvfs_erq_merge_mask_b << 8) |
			(!!pwrctrl->gce_req_mask_b << 7) |
			(!!pwrctrl->vdec_req_mask_b << 6) |
			((pwrctrl->dvfs_halt_mask_b & 0x1f) << 0));

	mmio_write_32(SPM_CLK_CON,
			(mmio_read_32(SPM_CLK_CON) & ~CC_SRCLKENA_MASK_0) |
			(pwrctrl->srclkenai_mask ? CC_SRCLKENA_MASK_0 : 0));

	/* set CPU WFI mask */
	mmio_write_32(SPM_MP2_CPU0_WFI_EN, !!pwrctrl->mp2_cpu0_wfi_en);
	mmio_write_32(SPM_MP2_CPU1_WFI_EN, !!pwrctrl->mp2_cpu1_wfi_en);
	mmio_write_32(SPM_MP1_CPU0_WFI_EN, !!pwrctrl->mp1_cpu0_wfi_en);
	mmio_write_32(SPM_MP1_CPU1_WFI_EN, !!pwrctrl->mp1_cpu1_wfi_en);
	mmio_write_32(SPM_MP1_CPU2_WFI_EN, !!pwrctrl->mp1_cpu2_wfi_en);
	mmio_write_32(SPM_MP1_CPU3_WFI_EN, !!pwrctrl->mp1_cpu3_wfi_en);
	mmio_write_32(SPM_MP0_CPU0_WFI_EN, !!pwrctrl->mp0_cpu0_wfi_en);
	mmio_write_32(SPM_MP0_CPU1_WFI_EN, !!pwrctrl->mp0_cpu1_wfi_en);
	mmio_write_32(SPM_MP0_CPU2_WFI_EN, !!pwrctrl->mp0_cpu2_wfi_en);
	mmio_write_32(SPM_MP0_CPU3_WFI_EN, !!pwrctrl->mp0_cpu3_wfi_en);
}

static uint32_t pcm_timer_ramp_max = 1;
static uint32_t pcm_timer_ramp_max_sec_loop = 1;

void spm_set_wakeup_event(const struct pwr_ctrl *pwrctrl)
{
	uint32_t val, mask, isr;

	INFO("%s: begin\n", __func__);
	/* set PCM timer (set to max when disable) */
	if (pwrctrl->timer_val_ramp_en != 0) {
		val = pcm_timer_ramp_max;
		pcm_timer_ramp_max++;
		if (pcm_timer_ramp_max >= 300)
			pcm_timer_ramp_max = 1;
	} else if (pwrctrl->timer_val_ramp_en_sec != 0) {
		val = pcm_timer_ramp_max * 1600;	/* 50ms */

		pcm_timer_ramp_max += 1;
		if (pcm_timer_ramp_max >= 300)	/* max 15 sec */
			pcm_timer_ramp_max = 1;

		pcm_timer_ramp_max_sec_loop++;
		if (pcm_timer_ramp_max_sec_loop >= 50) {
			pcm_timer_ramp_max_sec_loop = 0;
			/* range 6min to 10min */
			val = (pcm_timer_ramp_max + 300) * 32000;
		}
	} else {
		if (pwrctrl->timer_val_cust == 0)
			val = pwrctrl->timer_val ? pwrctrl->timer_val :
					PCM_TIMER_MAX;
		else
			val = pwrctrl->timer_val_cust;
	}

	mmio_write_32(SPM_PCM_TIMER_VAL, val);
	mmio_write_32(SPM_PCM_CON1,
		mmio_read_32(SPM_PCM_CON1) | SPM_REGWR_CFG_KEY |
		PCM_TIMER_EN_LSB);

	/* unmask AP wakeup source */
	if (pwrctrl->wake_src_cust == 0)
		mask = pwrctrl->wake_src;
	else
		mask = pwrctrl->wake_src_cust;

	if (pwrctrl->syspwreq_mask)
		mask &= ~WAKE_SRC_R12_CSYSPWREQ_B;
	mmio_write_32(SPM_WAKEUP_EVENT_MASK, ~mask);

#if 0
	/* unmask MD32 wakeup source */
	spm_write(SPM_SLEEP_MD32_WAKEUP_EVENT_MASK, ~pwrctrl->wake_src_md32);
#endif

	/* unmask SPM ISR (keep TWAM setting) */
	isr = mmio_read_32(SPM_IRQ_MASK) & SPM_TWAM_IRQ_MASK_LSB;
	mmio_write_32(SPM_IRQ_MASK, isr | ISRM_RET_IRQ_AUX);
	INFO("%s: end\n", __func__);
}

void spm_kick_pcm_to_run(struct pwr_ctrl *pwrctrl)
{
	uint32_t con0;

	INFO("%s: begin\n", __func__);
	/* init register to match PCM expectation */
	mmio_write_32(SPM_MAS_PAUSE_MASK_B, 0xffffffff);
	mmio_write_32(SPM_MAS_PAUSE2_MASK_B, 0xffffffff);
	mmio_write_32(SPM_PCM_REG_DATA_INI, 0);

	/* set PCM flags and data */
	mmio_write_32(SPM_SW_FLAG, pwrctrl->pcm_flags);
	mmio_write_32(SPM_SW_RSV_0, pwrctrl->pcm_reserve);

	/* lock Infra DCM when PCM runs */
	mmio_write_32(SPM_CLK_CON,
			(mmio_read_32(SPM_CLK_CON) & ~SPM_LOCK_INFRA_DCM_LSB) |
			(pwrctrl->infra_dcm_lock ? SPM_LOCK_INFRA_DCM_LSB : 0));

	/* enable r0 and r7 to control power */
	mmio_write_32(SPM_PCM_PWR_IO_EN,
			(pwrctrl->r0_ctrl_en ? PCM_PWRIO_EN_R0 : 0) |
			(pwrctrl->r7_ctrl_en ? PCM_PWRIO_EN_R7 : 0));

	/* kick PCM to run (only toggle PCM_KICK) */
	con0 = mmio_read_32(SPM_PCM_CON0) & ~(CON0_IM_KICK | CON0_PCM_KICK);
	mmio_write_32(SPM_PCM_CON0, con0 | SPM_REGWR_CFG_KEY | PCM_CK_EN_LSB |
			CON0_PCM_KICK);
	mmio_write_32(SPM_PCM_CON0, con0 | SPM_REGWR_CFG_KEY | PCM_CK_EN_LSB);
	INFO("%s: end\n", __func__);
}
