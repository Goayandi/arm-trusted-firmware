#
# Copyright (c) 2013-2014, ARM Limited and Contributors. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# Neither the name of ARM nor the names of its contributors may be used
# to endorse or promote products derived from this software without specific
# prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

MTK_PLAT                :=	plat/mediatek
MTK_PLAT_SOC            :=	${MTK_PLAT}/${PLAT}

PLAT_INCLUDES		:=	-I${MTK_PLAT_SOC}/include			\
				-I${MTK_PLAT_SOC}				\
				-I${MTK_PLAT_SOC}/debug/			\
				-I${MTK_PLAT_SOC}/drivers/emi			\
				-I${MTK_PLAT_SOC}/drivers/idvfs			\
				-I${MTK_PLAT_SOC}/drivers/l2c			\
				-I${MTK_PLAT_SOC}/drivers/ocp			\
				-I${MTK_PLAT_SOC}/drivers/pwrc			\
				-I${MTK_PLAT_SOC}/drivers/scp			\
				-I${MTK_PLAT_SOC}/drivers/timer			\
				-I${MTK_PLAT}/common/				\
				-I${MTK_PLAT}/common/drivers/uart		\
				-Iplat/common					\
				-Iinclude/plat/arm/common

PLAT_BL_COMMON_SOURCES	:=	drivers/io/io_fip.c				\
				drivers/io/io_memmap.c				\
				drivers/io/io_semihosting.c			\
				drivers/io/io_storage.c				\
				lib/aarch64/xlat_tables.c			\
				lib/semihosting/semihosting.c			\
				lib/semihosting/aarch64/semihosting_call.S	\
				plat/common/aarch64/plat_common.c		\
				${MTK_PLAT_SOC}/plat_io_storage.c		\
				plat/common/plat_gicv2.c			\

BL31_SOURCES		+=	drivers/arm/cci/cci.c				\
				drivers/console/aarch64/console.S		\
				drivers/arm/gic/common/gic_common.c		\
				drivers/arm/gic/v2/gicv2_main.c			\
				drivers/arm/gic/v2/gicv2_helpers.c		\
				drivers/arm/tzc400/tzc400.c			\
				lib/cpus/aarch64/aem_generic.S			\
				lib/cpus/aarch64/cortex_a53.S			\
				lib/cpus/aarch64/cortex_a57.S			\
				lib/cpus/aarch64/cortex_a72.S			\
				plat/common/aarch64/platform_mp_stack.S		\
				${MTK_PLAT}/common/drivers/uart/8250_console.S	\
				${MTK_PLAT}/common/mtk_plat_common.c		\
				${MTK_PLAT_SOC}/bl31_plat_setup.c		\
				${MTK_PLAT_SOC}/plat_gic.c			\
				${MTK_PLAT_SOC}/plat_dfd.c			\
				${MTK_PLAT_SOC}/plat_pm.c			\
				${MTK_PLAT_SOC}/power.c				\
				${MTK_PLAT_SOC}/plat_topology.c			\
				${MTK_PLAT_SOC}/scu.c				\
				${MTK_PLAT_SOC}/mailbox.c			\
				${MTK_PLAT_SOC}/aarch64/plat_helpers.S		\
				${MTK_PLAT_SOC}/aarch64/platform_common.c	\
				${MTK_PLAT_SOC}/debug/fiq_smp_call.c		\
				${MTK_PLAT_SOC}/debug/cache/plat_cache.c	\
				${MTK_PLAT_SOC}/debug/cache/plat_cache_ops.S	\
				${MTK_PLAT_SOC}/drivers/timer/mt_cpuxgpt.c	\
				${MTK_PLAT_SOC}/drivers/pwrc/plat_pwrc.c	\
				${MTK_PLAT_SOC}/drivers/md/md.c			\
				${MTK_PLAT_SOC}/drivers/l2c/l2c.c		\
				${MTK_PLAT_SOC}/drivers/emi/emi_mpu.c		\
				${MTK_PLAT_SOC}/drivers/idvfs/mt_idvfs_api.c	\
				${MTK_PLAT_SOC}/drivers/ocp/mt_ocp_api.c	\
				${MTK_PLAT_SOC}/drivers/scp/scp.c

BL31_SOURCES		+=	${MTK_PLAT_SOC}/sip_svc/sip_svc_common.c	\
				${MTK_PLAT_SOC}/sip_svc/sip_svc_setup.c		\

ifeq (${SPD}, tbase)
BL31_SOURCES		+=	${MTK_PLAT_SOC}/plat_tbase.c
endif

ifeq (${SPD}, teeid)
BL31_SOURCES           +=      ${MTK_PLAT_SOC}/plat_teei.c
endif

# Flag used by the MTK_platform port to determine the version of ARM GIC architecture
# to use for interrupt management in EL3.
MT_GIC_ARCH		:=	2
$(eval $(call add_define,MT_GIC_ARCH))

# Enable workarounds for selected Cortex-A53 erratas.
ERRATA_A53_826319       :=      0
ERRATA_A53_836870       :=      0

DEBUG			:=	1
RESET_TO_BL31		:=	1

# MSCI-A has move slave port than CCI-400
ARM_CCI_PRODUCT_ID	:=	500

BUILD_PLAT		:=	${BUILD_BASE}/${BUILD_TYPE}

MTK_SIP_KERNEL_BOOT_ENABLE	:=	1

$(eval $(call add_define,MTK_SIP_KERNEL_BOOT_ENABLE))
