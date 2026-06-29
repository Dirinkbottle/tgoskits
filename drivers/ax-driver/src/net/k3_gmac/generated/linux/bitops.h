/* SPDX-License-Identifier: GPL-2.0-only */
#pragma once

#ifndef BIT
#define BIT(nr) (1U << (nr))
#endif

#ifndef GENMASK
#define GENMASK(h, l) \
	(((~0U) - ((1U << (l)) - 1U)) & (~0U >> (31U - (h))))
#endif
