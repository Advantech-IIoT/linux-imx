
/* SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause */
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53LX and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */

/**
 * @file  vl53lx_types.h
 * @brief VL53LX types definition
 */

#ifndef _VL53LX_TYPES_H_
#define _VL53LX_TYPES_H_

#include <linux/types.h>
/** use where fractional values are expected
 *
 * Given a floating point value f it's .16 bit point is (int)(f*(1<<16))
 */
typedef uint32_t FixPoint1616_t;

#endif /* VL53LX_TYPES_H_ */
