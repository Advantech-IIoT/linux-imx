
/* SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause */
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53LX and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */


#ifndef _VL53LX_PLATFORM_USER_DATA_H_
#define _VL53LX_PLATFORM_USER_DATA_H_

#include "vl53lx_ll_def.h"

#include <linux/string.h>
#include "vl53lx_def.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include <linux/string.h>

#define VL53LX_Dev_t VL53LX_DevData_t
#define VL53LX_DEV VL53LX_DevData_t *

#define VL53LXDevDataGet(Dev, field) (Dev->field)
#define VL53LXDevDataSet(Dev, field, data) ((Dev->field) = (data))

#define VL53LXDevStructGetLLDriverHandle(Dev) (&VL53LXDevDataGet(Dev, LLData))
#define VL53LXDevStructGetLLResultsHandle(Dev) (&VL53LXDevDataGet(Dev,\
		llresults))

#ifdef __cplusplus
}
#endif

#endif

