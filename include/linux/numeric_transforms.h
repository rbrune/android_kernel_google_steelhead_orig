/*
 * Copyright (C) 2011 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __LINUX_NUMERIC_TRANSFORMS_H
#define __LINUX_NUMERIC_TRANSFORMS_H

#include <linux/types.h>

extern void reduce_s32_u32_ratio(s32 *N, u32 *D);

struct fixed_u64_32 {
	u64 whole;
	u32 frac;
};

struct fixed_s64_32 {
	s64 whole;
	u32 frac;
};

extern void linear_transform_s64_to_s64_32(
		s64 val,
		s64 basis_64,
		s32 N,
		u32 D,
		struct fixed_s64_32* basis_64_32,
		struct fixed_s64_32* result);

extern s64 linear_transform_s64_to_s64(
		s64 val,
		s64 basis1,
		s32 N,
		u32 D,
		s64 basis2);

#endif  /* __LINUX_NUMERIC_TRANSFORMS_H */
