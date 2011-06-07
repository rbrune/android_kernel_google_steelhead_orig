/* lib/numeric_transforms.c
 *
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

#ifndef HOST_SIDE_UNIT_TEST
#include <linux/bug.h>
#include <linux/module.h>
#include <linux/numeric_transforms.h>
#include <asm/div64.h>
#define UT_DEBUG(a, b...)
#else
#include "unit_test_harness.h"
#endif

#define MAX64 ((u64)(-1))
#define MAX32 ((u32)(-1))

#define MINS64 ((s64)(0x1ull << 63))
#define MAXS64 ((s64)((0x1ull << 63) - 1))

#define ABS(x) (((x) < 0) ? -(x) : (x))

void reduce_s32_u32_ratio(s32 *N, u32 *D)
{
	u32 a, b;
	if (!N || !D || !(*D)) {
		BUG();
		return;
	}

	a = ABS(*N);
	b = *D;

	if (a == 0) {
		*D = 1;
		return;
	}

	/* This implements Euclid's method to find GCD. */
	if (a < b) {
		u32 tmp = a;
		a = b;
		b = tmp;
	}

	while (1) {
		/* a is now the greater of the two. */
		const u32 remainder = a % b;
		if (remainder == 0) {
			*N /= b;
			*D /= b;
			return;
		}
		/* by swapping remainder and b, we are guaranteeing that a is
		 * still the greater of the two upon entrance to the loop.
		 */
		a = b;
		b = remainder;
	}
}

static int scale_u64_to_u64_32(u64 val, u32 N, u32 D,
			       struct fixed_u64_32 *res,
			       int round_up_not_down)
{
	u64 tmp1, tmp2, frac;
	u32 r;
	BUG_ON(!res);
	BUG_ON(!D);

	/* Let U32(X) denote a u32 containing the upper 32 bits of a
	 * 64 bit integer X.
	 * Let L32(X) denote a u32 containing the lower 32 bits of a
	 * 64 bit integer X.
	 * Let X[A, B] with A <= B denote bits A through B of the integer X.
	 * Let (A | B) denote the concatination of two 32 bit ints, A and B.
	 * IOW X = (A | B) => U32(X) == A && L32(X) == B
	 *
	 * compute M = val * N (a 96 bit int)
	 * ---------------------------------
	 * tmp2 = U32(val) * N (a 64 bit int)
	 * tmp1 = L32(val) * N (a 64 bit int)
	 * which means
	 * M = val * N = (tmp2 << 32) + tmp1
	 */
	tmp2 = (val >> 32) * N;
	tmp1 = (val & MAX32) * N;

	UT_DEBUG("val * N = %lx%08x\n", tmp2 + (tmp1 >> 32), (u32)tmp1);

	/* compute M[32, 95]
	 * tmp2 = tmp2 + U32(tmp1)
	 *      = (U32(val) * N) + U32(L32(val) * N)
	 *      = M[32, 95]
	 */
	tmp2 += tmp1 >> 32;

	/* if M[64, 95] >= D, then M/D has bits > 63 set and we have
	 * an overflow.
	 */
	if ((tmp2 >> 32) >= D) {
		res->whole = MAX64;
		res->frac  = MAX32;
		return 0;
	}

	/* Divide.  Going in we know
	 * tmp2 = M[32, 95]
	 * U32(tmp2) < D
	 */
	r = do_div(tmp2, D);

	/* At this point
	 * tmp1      = L32(val) * N
	 * tmp2      = M[32, 95] / D
	 *           = (M / D)[32, 95]
	 * r         = M[32, 95] % D
	 * U32(tmp2) = 0
	 *
	 * compute tmp1 = (r | M[0, 31])
	 */
	tmp1 = (tmp1 & MAX32) | ((u64)r << 32);

	/* Divide again.  Stuff the remainder into the upper 32 bits of frac. */
	frac = ((u64)do_div(tmp1, D)) << 32;

	UT_DEBUG("whole M/D = %lx%08lx :: doing %16lx / %x\n",
		 tmp2, tmp1, frac, D);
	/* At this point
	 * tmp2      = (M / D)[32, 95]
	 * tmp1      = (M / D)[ 0, 31]
	 * frac      = (M % D | 0)
	 * U32(tmp1) = 0
	 * U32(tmp2) = 0
	 * U32(frac) < D
	 * Divide one final time to compute the fractional component.
	 */
	r = do_div(frac, D);
	if (r && round_up_not_down)
		++frac;
	UT_DEBUG("frac %16lx\n", frac);

	/* Finally...
	 * tmp2      = (M / D)[ 32, 95]
	 * tmp1      = (M / D)[  0, 31]
	 * frac      = (M / D)[-32, -1]
	 * U32(tmp1) = 0
	 * U32(tmp2) = 0
	 * U32(frac) = 0 or 1 (remote possibility of 1 if we forced a round up)
	 *
	 * Pack the results into res and we are finished.
	 */
	res->frac  = (u32)frac;
	res->whole = (((tmp2 & MAX32) << 32) | tmp1) + (frac >> 32);

	return 1;
}

void linear_transform_s64_to_s64_32(s64 val, s64 basis_64, s32 N, u32 D,
				    struct fixed_s64_32 *basis_64_32,
				    struct fixed_s64_32 *result)
{
	int is_neg;
	struct fixed_u64_32 scaled;
	u64 abs_val;
	BUG_ON(!result);
	BUG_ON(!basis_64_32);

	/* Compute abs(val - basis_64). Keep track of whether or not this delta
	 * will be negative after the scale opertaion.
	 */
	if (val < basis_64) {
		is_neg = (N > 0) ? 1 : 0;
		abs_val = basis_64 - val;
	} else {
		is_neg = (N > 0) ? 0 : 1;
		abs_val = val - basis_64;
	}
	UT_DEBUG("CODE : %c%016lx\n", is_neg ? '-' : ' ', abs_val);

	/* Scale.  If the scale operation overflows, we cannot proceed.  Just
	 * return an overflow (or underflow) value depending on whether or not
	 * val was positive or negative.
	 */
	if (!scale_u64_to_u64_32(abs_val, (u32)ABS(N), D, &scaled, is_neg)) {
		if (is_neg) {
			result->whole = MINS64;
			result->frac  = 0;
		} else {
			result->whole = MAXS64;
			result->frac  = MAX32;
		}
		return;
	}

	UT_DEBUG("CODE : %c%016lx.%08x\n", is_neg ? '-' : ' ',
		 scaled.whole, scaled.frac);

	/* If the scaled val should be negative, then invert scaled. */
	if (is_neg) {
		scaled.whole = ~scaled.whole;
		scaled.frac  = (~scaled.frac) + 1;
		if (!scaled.frac)
			++scaled.whole;
	}

	/* result = scaled + basis_64_32 */
	result->frac  = basis_64_32->frac  + scaled.frac;
	result->whole = basis_64_32->whole + scaled.whole;
	if ((result->frac < basis_64_32->frac) || (result->frac < scaled.frac))
		++result->whole;

	UT_DEBUG("CODE : %016lx.%08x\n", scaled.whole, scaled.frac);
	UT_DEBUG("CODE : %016lx.%08x\n", basis_64_32->whole, basis_64_32->frac);
	UT_DEBUG("CODE : %016lx.%08x\n", result->whole, result->frac);

	if (is_neg) {
		if (((scaled.whole ^ result->whole) &
		     (basis_64_32->whole ^ result->whole)) & MINS64) {
			if (result->whole < 0) {
				result->whole = MAXS64;
				result->frac  = MAX32;
			} else {
				result->whole = MINS64;
				result->frac  = 0;
			}
		}
	} else {
		u8 o1, o2, o3;
		/* Check for overflow.  If we overflowed, peg the value of the
		 * result and return.  Otherwise we are finished, just return.
		 * Keep in mind; we just did
		 *
		 * signed = unsigned + signed.
		 *
		 * Underflow should not be possible, but overflow is possible.
		 */
		o1 = (u64)scaled.whole >> 63;
		o2 = ((u64)basis_64_32->whole >> 63) ^ 0x1;
		o3 = (u64)result->whole >> 63;
		if ((o3 & (o2 | o1)) | (o2 & o1)) {
			result->whole = MAXS64;
			result->frac  = MAX32;
		}

	}
}

static int scale_u64_to_u64(u64 val, u32 N, u32 D, u64 *res,
			    int round_up_not_down)
{
	u64 tmp1, tmp2;
	u32 r;
	BUG_ON(!res);
	BUG_ON(!D);

	/* Let U32(X) denote a u32 containing the upper 32 bits of
	 * a 64 bit integer X.
	 * Let L32(X) denote a u32 containing the lower 32 bits of
	 * a 64 bit integer X.
	 * Let X[A, B] with A <= B denote bits A through B of the integer X.
	 * Let (A | B) denote the concatination of two 32 bit ints, A and B.
	 * IOW X = (A | B) => U32(X) == A && L32(X) == B
	 *
	 * compute M = val * N (a 96 bit int)
	 * ---------------------------------
	 * tmp2 = U32(val) * N (a 64 bit int)
	 * tmp1 = L32(val) * N (a 64 bit int)
	 * which means
	 * M = val * N = (tmp2 << 32) + tmp1
	 */
	tmp2 = (val >> 32) * N;
	tmp1 = (val & MAX32) * N;

	UT_DEBUG("val * N = %lx%08x\n", tmp2 + (tmp1 >> 32), (u32)tmp1);

	/* compute M[32, 95]
	 * tmp2 = tmp2 + U32(tmp1)
	 *      = (U32(val) * N) + U32(L32(val) * N)
	 *      = M[32, 95]
	 */
	tmp2 += tmp1 >> 32;

	/* if M[64, 95] >= D, then M/D has bits > 63 set and we have
	 * an overflow.
	 */
	if ((tmp2 >> 32) >= D) {
		*res = MAX64;
		return 0;
	}

	/* Divide.  Going in we know
	 * tmp2 = M[32, 95]
	 * U32(tmp2) < D
	 */
	r = do_div(tmp2, D);

	/* At this point
	 * tmp1      = L32(val) * N
	 * tmp2      = M[32, 95] / D
	 *           = (M / D)[32, 95]
	 * r         = M[32, 95] % D
	 * U32(tmp2) = 0
	 *
	 * compute tmp1 = (r | M[0, 31])
	 */
	tmp1 = (tmp1 & MAX32) | ((u64)r << 32);

	/* Divide again.  Keep the remainder around in order to round
	 * properly.
	 */
	r = do_div(tmp1, D);

	UT_DEBUG("whole M/D = %lx%08lx\n", tmp2, tmp1);
	/* At this point
	 * tmp2      = (M / D)[32, 95]
	 * tmp1      = (M / D)[ 0, 31]
	 * r         =  M % D
	 * U32(tmp1) = 0
	 * U32(tmp2) = 0
	 */

	/* Pack the result and deal with the round-up case (As well as the
	 * remote possiblility over overflow in such a case).
	 */
	*res = (tmp2 << 32) | tmp1;
	if (r && round_up_not_down) {
		++(*res);
		if (!(*res)) {
			*res = MAX64;
			return 0;
		}
	}

	return 1;
}

s64 linear_transform_s64_to_s64(
		s64 val,
		s64 basis1,
		s32 N,
		u32 D,
		s64 basis2) {
	u64 scaled, res;
	u64 abs_val;
	int is_neg;

	UT_DEBUG("(((%016lx - %016lx) * %08x) / %08x) + %016lx == ?\n",
			val, basis1, N, D, basis2);

	/* Compute abs(val - basis_64). Keep track of whether or not this delta
	 * will be negative after the scale opertaion.
	 */
	if (val < basis1) {
		is_neg = 1;
		abs_val = basis1 - val;
	} else {
		is_neg = 0;
		abs_val = val - basis1;
	}

	if (N < 0)
		is_neg = !is_neg;

	UT_DEBUG("abs %016lx is_neg %d\n", abs_val, is_neg);

	if (!scale_u64_to_u64(
				abs_val,
				ABS(N),
				D,
				&scaled,
				is_neg))
		return is_neg ? MINS64 : MAXS64; /* overflow/undeflow */

	UT_DEBUG("scaled %016lx\n", scaled);

	/* if scaled is >= 0x8000<etc>, then we are going to overflow or
	 * underflow unless ABS(basis2) is large enough to pull us back
	 * into the non-overflow/underflow region.
	 */
	if (scaled & MINS64) {
		UT_DEBUG("pullin check; is_neg %d scaled %016lx basis2 %016lx"
				" (sl %016lx abs_basis %016lx)\n",
				is_neg, scaled, basis2,
				 (scaled & MAXS64),
				 ABS(basis2));

		if (is_neg && (basis2 < 0)) {
			UT_DEBUG("certain underflow (scaled %016lx, is_neg %d,"
				 " basis2 %016lx\n", scaled, is_neg, basis2);
			return MINS64; /* certain underflow */
		}

		if (!is_neg && (basis2 >= 0)) {
			UT_DEBUG("certain overflow (scaled %016lx, is_neg %d,"
				 " basis2 %016lx\n", scaled, is_neg, basis2);
			return MAXS64; /* certain overflow */
		}

		if (ABS(basis2) <= (scaled & MAXS64)) {
			UT_DEBUG("not enough (scaled %016lx, is_neg %d,"
				 " basis2 %016lx (%016lx <= %016lx)\n",
				 scaled, is_neg, basis2,
				 ABS(basis2), (scaled & MAXS64));
			return is_neg ? MINS64 : MAXS64; /* not enough */
		}

		/* Looks like we are OK */
		return (is_neg ? (-scaled) : scaled) + basis2;
	}

	/* Scaled fits within signed bounds, so we just need to check for
	 * over/underflow for two signed integers.  Basically, if both scaled
	 * and basis2 have the same sign bit, and the result has a different
	 * sign bit, then we have under/overflow.  An easy way to compute this
	 * is
	 *    ((scaled_signbit XNOR basis_signbit) &&
	 *     (scaled_signbit XOR res_signbit)) ==
	 *    ((scaled_signbit XOR basis_signbit XOR 1) &&
	 *     (scaled_signbit XOR res_signbit))
	 */
	if (is_neg)
		scaled = -scaled;
	res = scaled + basis2;

	UT_DEBUG("signed ovfl check s = %016lx b = %016lx r = %016lx\n",
			scaled, basis2, res);

	if ((scaled ^ basis2 ^ MINS64) & (scaled ^ res) & MINS64)
		return (basis2 < 0) ? MINS64 : MAXS64; /* overflow */

	return res;
}
