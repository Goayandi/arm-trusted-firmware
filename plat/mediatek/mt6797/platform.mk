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

MTK_PLAT		:=	mediatek/${PLAT}
MTK_PLAT_SOC		:=	${MTK_PLAT}/${PLAT}

PLAT_INCLUDES		:=	-Iplat/${MTK_PLAT}/include/        \
				-Iplat/${MTK_PLAT}/                \
				-Iplat/${MTK_PLAT}/drivers/log     \
				-Iplat/${MTK_PLAT}/drivers/timer/  \
				-Iplat/${MTK_PLAT}/drivers/l2c/ \
				-Iplat/${MTK_PLAT}/drivers/emi/ \
				-Iplat/${MTK_PLAT}/drivers/idvfs \
				-Iplat/${MTK_PLAT}/drivers/ocp \
				-Iplat/${MTK_PLAT}/drivers/scp/ \
				-Iplat/common

PLAT_BL_COMMON_SOURCES	:=	drivers/io/io_fip.c				\
				drivers/io/io_memmap.c				\
				drivers/io/io_semihosting.c			\
				drivers/io/io_storage.c				\
				lib/aarch64/xlat_tables.c			\
				lib/semihosting/semihosting.c			\
				lib/semihosting/aarch64/semihosting_call.S	\
				plat/common/aarch64/plat_common.c		\
				plat/${MTK_PLAT}/plat_io_storage.c			\
				plat/${MTK_PLAT}/fiq_smp_call.c                      \
				plat/${MTK_PLAT}/plat_cache.c			\
				plat/${MTK_PLAT}/plat_cache_ops.S

BL31_SOURCES		+=	drivers/arm/gic/arm_gic.c			\
				drivers/arm/gic/gic_v2.c			\
				drivers/arm/gic/gic_v3.c			\
				drivers/arm/tzc400/tzc400.c			\
				lib/cpus/aarch64/aem_generic.S			\
				lib/cpus/aarch64/cortex_a53.S			\
				lib/cpus/aarch64/cortex_a57.S			\
				lib/cpus/aarch64/cortex_a72.S			\
				plat/common/aarch64/platform_mp_stack.S		\
				plat/${MTK_PLAT}/bl31_plat_setup.c			\
				plat/${MTK_PLAT}/plat_gic.c				\
				plat/${MTK_PLAT}/plat_dfd.c				\
				plat/${MTK_PLAT}/plat_pm.c				\
				plat/${MTK_PLAT}/power.c				\
				plat/${MTK_PLAT}/plat_security.c				\
				plat/${MTK_PLAT}/plat_topology.c				\
				plat/${MTK_PLAT}/scu.c				\
				plat/${MTK_PLAT}/mailbox.c				\
				plat/${MTK_PLAT}/aarch64/plat_helpers.S			\
				plat/${MTK_PLAT}/aarch64/platform_common.c			\
				plat/${MTK_PLAT}/drivers/uart/uart.c			\
				plat/${MTK_PLAT}/drivers/timer/mt_cpuxgpt.c              \
				plat/${MTK_PLAT}/drivers/pwrc/plat_pwrc.c \
				plat/${MTK_PLAT}/drivers/md/md.c		\
				plat/${MTK_PLAT}/drivers/l2c/l2c.c                  \
				plat/${MTK_PLAT}/drivers/emi/emi_mpu.c\
				plat/${MTK_PLAT}/drivers/idvfs/mt_idvfs_api.c \
				plat/${MTK_PLAT}/drivers/ocp/mt_ocp_api.c \
				plat/${MTK_PLAT}/drivers/mcsi/mcsi_a.c \
				plat/${MTK_PLAT}/drivers/scp/scp.c

BL31_SOURCES		+=	plat/${MTK_PLAT}/sip_svc/sip_svc_common.c		\
				plat/${MTK_PLAT}/sip_svc/sip_svc_setup.c		\
				plat/${MTK_PLAT}/drivers/log/log.c

ifeq (${SPD}, tbase)
BL31_SOURCES		+=	plat/${MTK_PLAT}/plat_tbase.c
endif

ifeq (${SPD}, teeid)
BL31_SOURCES           +=      plat/${MTK_PLAT}/plat_teei.c
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