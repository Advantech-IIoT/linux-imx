
// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53LX and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */

/**
 * @file /stmvl53lx_log.c  vl53lx_module  ST VL53LX linux kernel module
 *
 * This is implementation of low level driver trace support
 */
#include <linux/module.h>
#include <linux/printk.h>

#include "stmvl53lx.h"

#ifdef VL53LX_LOG_ENABLE

/* trace levels defaults can be forced here */
static bool trace_function = VL53LX_TRACE_FUNCTION_NONE;
/* modules selection can mix multiple modules with | operator */
static int trace_module = VL53LX_TRACE_MODULE_NONE;
static int trace_level = VL53LX_TRACE_LEVEL_NONE;

module_param(trace_function, bool, 0644);
MODULE_PARM_DESC(trace_function,
	"allow tracing of low level function entry and exit");

module_param(trace_module, int, 0644);
MODULE_PARM_DESC(trace_module,
	"control tracing of low level per module");

module_param(trace_level, int, 0644);
MODULE_PARM_DESC(trace_level,
	"control tracing of low level per level");

void log_trace_print(uint32_t module, uint32_t level, uint32_t function,
	const char *format, ...)
{
	va_list args;

	if ( ((level <=trace_level) && ((module & trace_module) > 0))
		|| ((function & trace_function) > 0) ) {

		va_start(args, format);
		vprintk(format, args);
		va_end(args);
	}
}

#endif
