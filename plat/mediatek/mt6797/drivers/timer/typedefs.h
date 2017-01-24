/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein is
 * confidential and proprietary to MediaTek Inc. and/or its licensors. Without
 * the prior written permission of MediaTek inc. and/or its licensors, any
 * reproduction, modification, use or disclosure of MediaTek Software, and
 * information contained herein, in whole or in part, shall be strictly
 * prohibited.
 *
 * MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER
 * ON AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL
 * WARRANTIES, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR
 * NONINFRINGEMENT. NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH
 * RESPECT TO THE SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY,
 * INCORPORATED IN, OR SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES
 * TO LOOK ONLY TO SUCH THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO.
 * RECEIVER EXPRESSLY ACKNOWLEDGES THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO
 * OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES CONTAINED IN MEDIATEK
 * SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE
 * RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S
 * ENTIRE AND CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE
 * RELEASED HEREUNDER WILL BE, AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE
 * MEDIATEK SOFTWARE AT ISSUE, OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE
 * CHARGE PAID BY RECEIVER TO MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek
 * Software") have been modified by MediaTek Inc. All revisions are subject to
 * any receiver's applicable license agreements with MediaTek Inc.
 */

#ifndef _TYPEDEFS_H_
#define _TYPEDEFS_H_

#include <stdint.h>

enum {
	RX,
	TX,
	NONE
};

#define READ_REGISTER_UINT32(reg) \
	(*(uint32_t * const)(uintptr_t)(reg))

#define WRITE_REGISTER_UINT32(reg, val) \
	((*(uint32_t * const)(uintptr_t)(reg)) = (val))

#define READ_REGISTER_UINT16(reg) \
	(*(uint16_t * const)(uintptr_t)(reg))

#define WRITE_REGISTER_UINT16(reg, val) \
	((*(uint16_t * const)(uintptr_t)(reg)) = (val))

#define READ_REGISTER_UINT8(reg) \
	(*(uint8_t * const)(uintptr_t)(reg))

#define WRITE_REGISTER_UINT8(reg, val) \
	((*(uint8_t * const)(uintptr_t)(reg)) = (val))

#define INREG8(x)	READ_REGISTER_UINT8((uint8_t *)(x))
#define OUTREG8(x, y)	(WRITE_REGISTER_UINT8((uint8_t *)(x), (uint8_t)(y)))
#define SETREG8(x, y)	OUTREG8(x, INREG8(x)|(y))
#define CLRREG8(x, y)	OUTREG8(x, INREG8(x)&~(y))
#define MASKREG8(x, y, z)	OUTREG8(x, (INREG8(x)&~(y))|(z))

#define INREG16(x)	READ_REGISTER_UINT16((x))
#define OUTREG16(x, y)	WRITE_REGISTER_UINT16((x), (int16_t)(y))
#define SETREG16(x, y)	OUTREG16(x, INREG16(x)|(y))
#define CLRREG16(x, y)	OUTREG16(x, INREG16(x)&~(y))
#define MASKREG16(x, y, z)	OUTREG16(x, (INREG16(x)&~(y))|(z))

#define INREG32(x)	READ_REGISTER_UINT32((x))
#define OUTREG32(x, y)	WRITE_REGISTER_UINT32((x), (uint32_t)(y))
#define SETREG32(x, y)	OUTREG32(x, INREG32(x)|(y))
#define CLRREG32(x, y)	OUTREG32(x, INREG32(x)&~(y))
#define MASKREG32(x, y, z)	OUTREG32(x, (INREG32(x)&~(y))|(z))


#define DRV_Reg8(addr)              INREG8(addr)
#define DRV_WriteReg8(addr, data)   OUTREG8(addr, data)
#define DRV_SetReg8(addr, data)     SETREG8(addr, data)
#define DRV_ClrReg8(addr, data)     CLRREG8(addr, data)

#define DRV_Reg16(addr)             INREG16(addr)
#define DRV_WriteReg16(addr, data)  OUTREG16(addr, data)
#define DRV_SetReg16(addr, data)    SETREG16(addr, data)
#define DRV_ClrReg16(addr, data)    CLRREG16(addr, data)

#define DRV_Reg32(addr)             INREG32(addr)
#define DRV_WriteReg32(addr, data)  OUTREG32(addr, data)
#define DRV_SetReg32(addr, data)    SETREG32(addr, data)
#define DRV_ClrReg32(addr, data)    CLRREG32(addr, data)

// !!! DEPRECATED, WILL BE REMOVED LATER !!!
#define DRV_Reg(addr)               DRV_Reg16(addr)
#define DRV_WriteReg(addr, data)    DRV_WriteReg16(addr, data)
#define DRV_SetReg(addr, data)      DRV_SetReg16(addr, data)
#define DRV_ClrReg(addr, data)      DRV_ClrReg16(addr, data)

#define __raw_readb(REG)            DRV_Reg8(REG)
#define __raw_readw(REG)            DRV_Reg16(REG)
#define __raw_readl(REG)            DRV_Reg32(REG)
#define __raw_writeb(VAL, REG)      DRV_WriteReg8(REG, VAL)
#define __raw_writew(VAL, REG)      DRV_WriteReg16(REG, VAL)
#define __raw_writel(VAL, REG)      DRV_WriteReg32(REG, VAL)

extern void platform_assert(char *file, int line, char *expr);

#define ASSERT(expr) do { \
			if (!(expr)) { \
				platform_assert(__FILE__, __LINE__, #expr); \
			} \
		} while (0)

// compile time assert
#define COMPILE_ASSERT(condition) ((void)sizeof(char[1 - 2*!!!(condition)]))

#include <stdarg.h>

#define READ_REG(REG)           __raw_readl(REG)
#define WRITE_REG(VAL, REG)     __raw_writel(VAL, REG)

#endif
