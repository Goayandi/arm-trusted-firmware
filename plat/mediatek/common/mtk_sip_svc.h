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
#ifndef __PLAT_SIP_SVC_H__
#define __PLAT_SIP_SVC_H__

/*******************************************************************************
 * Defines for runtime services func ids
 ******************************************************************************/
/* SMC32 ID range from 0x82000000 to 0x82000FFF */
/* SMC64 ID range from 0xC2000000 to 0xC2000FFF */

/* For MTK SMC from Trustonic TEE */
/* 0x82000000 - 0x820000FF & 0xC2000000 - 0xC20000FF */
#define MTK_SIP_TBASE_HWUID_AARCH32       0x82000000

/* For MTK SMC from Boot-loader */
/* 0x82000100 - 0x820001FF & 0xC2000100 - 0xC20001FF */
#define MTK_SIP_BL_INIT_AARCH32           0x82000100
#define MTK_SIP_BL_INIT_AARCH64           0xC2000100

/* For MTK SMC from Kernel */
/* 0x82000200 - 0x820002FF & 0xC2000200 - 0xC20002FF */
#define MTK_SIP_KERNEL_TMP_AARCH32        0x82000200
#define MTK_SIP_KERNEL_TMP_AARCH64        0xC2000200
#define MTK_SIP_KERNEL_MCUSYS_WRITE_AARCH32 0x82000201
#define MTK_SIP_KERNEL_MCUSYS_WRITE_AARCH64 0xC2000201
#define MTK_SIP_KERNEL_MCUSYS_ACCESS_COUNT_AARCH32 0x82000202
#define MTK_SIP_KERNEL_MCUSYS_ACCESS_COUNT_AARCH64 0xC2000202
#define MTK_SIP_KERNEL_L2_SHARING_AARCH32  0x82000203
#define MTK_SIP_KERNEL_L2_SHARING_AARCH64  0xC2000203
#define MTK_SIP_KERNEL_WDT_AARCH32 0x82000204
#define MTK_SIP_KERNEL_WDT_AARCH64 0xC2000204
#define MTK_SIP_KERNEL_GIC_DUMP_AARCH32 0x82000205
#define MTK_SIP_KERNEL_GIC_DUMP_AARCH64 0xC2000205
#define MTK_SIP_KERNEL_DAPC_INIT_AARCH32 0x82000206
#define MTK_SIP_KERNEL_DAPC_INIT_AARCH64 0xC2000206

#define MTK_SIP_SET_AUTHORIZED_SECURE_REG	0x82000001
#define MTK_SIP_PWR_ON_MTCMOS	0x82000402
#define MTK_SIP_PWR_OFF_MTCMOS	0x82000403
#define MTK_SIP_PWR_MTCMOS_SUPPORT             0x82000404

/* For MTK SMC Reserved */
/* 0x82000300 - 0x82000FFF & 0xC2000300 - 0xC2000FFF */

/*
 * Number of SIP calls (above) implemented.
 */
#define MTK_SIP_SVC_NUM_CALLS             2

/*******************************************************************************
 * Defines for SIP Service queries
 ******************************************************************************/
/* 0x82000000 - 0x8200FEFF is SIP service calls */
#define MTK_SIP_SVC_CALL_COUNT      0x8200ff00
#define MTK_SIP_SVC_UID             0x8200ff01
/* 0x8200ff02 is reserved */
#define MTK_SIP_SVC_VERSION         0x8200ff03
/* 0x8200ff04 - 0x8200FFFF is reserved for future expansion */

/* MTK SIP Service Calls version numbers */
#define MTK_SIP_VERSION_MAJOR		0x0
#define MTK_SIP_VERSION_MINOR		0x1

/* The macros below are used to identify SIP calls from the SMC function ID */
/* SMC32 ID range from 0x82000000 to 0x82000FFF */
/* SMC64 ID range from 0xC2000000 to 0xC2000FFF */
#define SIP_FID_MASK                0xf000u
#define SIP_FID_VALUE               0u
#define is_sip_fid(_fid) \
	(((_fid) & SIP_FID_MASK) == SIP_FID_VALUE)
#endif /* __PLAT_SIP_SVC_H__ */
