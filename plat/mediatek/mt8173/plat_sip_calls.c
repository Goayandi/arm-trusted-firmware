/*
 * Copyright (c) 2015, ARM Limited and Contributors. All rights reserved.
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
#include <assert.h>
#include <bl_common.h>
#include <context_mgmt.h>
#include <crypt.h>
#include <debug.h>
#include <mmio.h>
#include <mt8173_def.h>
#include <mtcmos.h>
#include <mtk_sip_error.h>
#include <mtk_sip_svc.h>
#include <mtk_sip_private.h>
#include <platform.h>
#include <plat_private.h>
#include <plat_sip_calls.h>
#include <runtime_svc.h>

extern entry_point_info_t *bl31_plat_get_next_kernel_ep_info(uint32_t type);

#define MP0_MISC_CONFIG0	(MCUCFG_BASE + 0x0030)
#define MP0_MISC_CONFIG9	(MCUCFG_BASE + 0x0054)

/*******************************************************************************
 * SMC Call for LK Jump
 ******************************************************************************/
struct kernel_info {
	uint64_t pc;
	uint64_t r0;
	uint64_t r1;
};

static struct kernel_info k_info;

static void save_kernel_info(uint64_t pc, uint64_t r0, uint64_t r1)
{
	k_info.pc=pc;
	k_info.r0=r0;
	k_info.r1=r1;
}

uint64_t get_kernel_info_pc(void)
{
	return k_info.pc;
}

uint64_t get_kernel_info_r0(void)
{
	return k_info.r0;
}

uint64_t get_kernel_info_r1(void)
{
	return k_info.r1;
}

void bl31_prepare_k64_entry(void)
{
	entry_point_info_t *next_image_info;
	uint32_t image_type;

	/* Determine which image to execute next */
	image_type = NON_SECURE;

	/* Program EL3 registers to enable entry into the next EL */
	next_image_info = bl31_plat_get_next_kernel_ep_info(image_type);
	assert(next_image_info);
	assert(image_type == GET_SECURITY_STATE(next_image_info->h.attr));

	INFO("BL3-1: Preparing for EL3 exit to %s world, Kernel\n",
	(image_type == SECURE) ? "secure" : "normal");
	INFO("BL3-1: Next image address = 0x%llx\n",
		(unsigned long long)next_image_info->pc);
	INFO("BL3-1: Next image spsr = 0x%x\n", next_image_info->spsr);
	cm_init_context(read_mpidr_el1(), next_image_info);
	cm_prepare_el3_exit(image_type);
}

/* Authorized secure register list */
enum {
	SREG_HDMI_COLOR_EN = 0x14000904
};

static const uint32_t authorized_sreg[] = {
	SREG_HDMI_COLOR_EN
};

#define authorized_sreg_cnt	\
	(sizeof(authorized_sreg) / sizeof(authorized_sreg[0]))

uint64_t mt_sip_set_authorized_sreg(uint32_t sreg, uint32_t val)
{
	uint64_t i;
	for (i = 0; i < authorized_sreg_cnt; i++) {
		if (authorized_sreg[i] == sreg) {
			mmio_write_32(sreg, val);
			return SIP_SVC_E_SUCCESS;
		}
	}

	return SIP_SVC_E_INVALID_PARAMS;
}

static uint64_t mt_sip_pwr_on_mtcmos(uint32_t val)
{
	uint32_t ret;

	ret = mtcmos_non_cpu_ctrl(1, val);
	if (ret)
		return SIP_SVC_E_INVALID_PARAMS;
	else
		return SIP_SVC_E_SUCCESS;
}

static uint64_t mt_sip_pwr_off_mtcmos(uint32_t val)
{
	uint32_t ret;

	ret = mtcmos_non_cpu_ctrl(0, val);
	if (ret)
		return SIP_SVC_E_INVALID_PARAMS;
	else
		return SIP_SVC_E_SUCCESS;
}

static uint64_t mt_sip_pwr_mtcmos_support(void)
{
	return SIP_SVC_E_SUCCESS;
}


/*******************************************************************************
 * SMC Call for Kernel MCUSYS register write
 ******************************************************************************/

static uint64_t mcusys_write_count = 0;
static uint64_t sip_mcusys_write(unsigned int reg_addr, unsigned int reg_value)
{
	if((reg_addr & 0xFFFF0000) != (MCUCFG_BASE & 0xFFFF0000))
		return SIP_SVC_E_INVALID_Range;

    /* Perform range check */
/*
    if(( MP0_MISC_CONFIG0 <= reg_addr && reg_addr <= MP0_MISC_CONFIG9 ) ||
       ( MP1_MISC_CONFIG0 <= reg_addr && reg_addr <= MP1_MISC_CONFIG9 )) {
        return SIP_SVC_E_PERMISSION_DENY;
    }
*/
	if(MP0_MISC_CONFIG0 <= reg_addr && reg_addr <= MP0_MISC_CONFIG9)
		return SIP_SVC_E_PERMISSION_DENY;

	mmio_write_32(reg_addr, reg_value);
	dsb();

	mcusys_write_count++;

	return SIP_SVC_E_SUCCESS;
}

/*******************************************************************************
 * SIP top level handler for servicing SMCs.
 ******************************************************************************/
uint64_t sip_smc_handler(uint32_t smc_fid,
			 uint64_t x1,
			 uint64_t x2,
			 uint64_t x3,
			 uint64_t x4,
			 void *cookie,
			 void *handle,
			 uint64_t flags)
{
	uint64_t rc;
	uint32_t ns;
	atf_arg_t_ptr teearg;

	/* Determine which security state this SMC originated from */
	ns = is_caller_non_secure(flags);

	//WARN("sip_smc_handler\n");
	//WARN("id=0x%llx\n", smc_fid);
	//WARN("x1=0x%llx, x2=0x%llx, x3=0x%llx, x4=0x%llx\n", x1, x2, x3, x4);

	switch (smc_fid) {
	case MTK_SIP_TBASE_HWUID_AARCH32:
		teearg = (atf_arg_t_ptr)(uintptr_t)TEE_BOOT_INFO_ADDR;
		if (ns)
			SMC_RET1(handle, SMC_UNK);
		SMC_RET4(handle, teearg->hwuid[0], teearg->hwuid[1],
			 teearg->hwuid[2], teearg->hwuid[3]);
		break;
	case MTK_SIP_KERNEL_MCUSYS_WRITE_AARCH32:
	case MTK_SIP_KERNEL_MCUSYS_WRITE_AARCH64:
		rc = sip_mcusys_write(x1, x2);
		break;
	case MTK_SIP_KERNEL_MCUSYS_ACCESS_COUNT_AARCH32:
	case MTK_SIP_KERNEL_MCUSYS_ACCESS_COUNT_AARCH64:
		rc = mcusys_write_count;
		break;
	case MTK_SIP_KERNEL_TMP_AARCH32:
		INFO("save kernel info\n");
		save_kernel_info(x1, x2, x3);
		INFO("end bl31_prepare_k64_entry...\n");
		bl31_prepare_k64_entry();
		INFO("el3_exit\n");
		SMC_RET0(handle);
		break;
	case MTK_SIP_SET_AUTHORIZED_SECURE_REG:
		rc = mt_sip_set_authorized_sreg((uint32_t)x1, (uint32_t)x2);
		SMC_RET1(handle, rc);
		break;
	case MTK_SIP_PWR_ON_MTCMOS:
		rc = mt_sip_pwr_on_mtcmos((uint32_t)x1);
		SMC_RET1(handle, rc);
		break;
	case MTK_SIP_PWR_OFF_MTCMOS:
		rc = mt_sip_pwr_off_mtcmos((uint32_t)x1);
		SMC_RET1(handle, rc);
		break;
	case MTK_SIP_PWR_MTCMOS_SUPPORT:
		rc = mt_sip_pwr_mtcmos_support();
		SMC_RET1(handle, rc);
		break;
	case MTK_SIP_SET_HDCP_KEY_NUM:
		rc = crypt_set_hdcp_key_num((uint32_t)x1);
		SMC_RET1(handle, rc);
		break;
	case MTK_SIP_CLR_HDCP_KEY:
		rc = crypt_clear_hdcp_key();
		SMC_RET1(handle, rc);
		break;
	case MTK_SIP_SET_HDCP_KEY_EX:
		rc = crypt_set_hdcp_key_ex(x1, x2, x3);
		SMC_RET1(handle, rc);
		break;
	default:
		rc = SMC_UNK;
		WARN("Unimplemented SIP Call: 0x%x \n", smc_fid);
		break;
	}
	SMC_RET1(handle, rc);
}
