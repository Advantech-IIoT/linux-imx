
// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53LX and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */

/**
 * @file /stmvl53lx_module.c  vl53lx_module  ST VL53LX linux kernel module
 *
 * main file
 */
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/time.h>
#include <linux/platform_device.h>
#include <linux/kobject.h>
#include <linux/kthread.h>
#include <linux/jhash.h>
#include <linux/ctype.h>

/*
 * API includes
 */

#include "stmvl53lx.h"
#include "stmvl53lx-i2c.h"

#include "stmvl53lx_if.h" /* our device interface to user space */
#include "stmvl53lx_internal_if.h"

/*
 * include default tuning file
 */
#include "stmvl53lx_tunings.h"

/* 
 * Abnormal stops error code for HAL Layer
 */
#define ABNORMAL_STOP_1 1
#define ABNORMAL_STOP_2 2
#define ABNORMAL_STOP_3 3

#define TIMEOUT_FIRST_FRAME_USECS 100000 /* 100 ms in micro seconds */

/** @ingroup vl53lx_config
 * @{
 */
/**
 * default polling period delay in millisecond
 *
 * It can be set at run time via @ref vl53lx_ioctl or @ref sysfs_attrib
 *
 * @note apply only for device operating in polling mode only
 */
#define STMVL53LX_CFG_POLL_DELAY_MS	5

/**
 * default timing budget in microsecond
 *
 * Can be change at run time via @ref vl53lx_ioctl or @ref sysfs_attrib
 */
#define STMVL53LX_CFG_TIMING_BUDGET_US	30000

/** default distance mode set to medium fix ticket #823926*/
#define STMVL53LX_CFG_DEFAULT_DISTANCE_MODE	VL53LX_DISTANCEMODE_MEDIUM

/** default crosstalk enable */
#define STMVL53LX_CFG_DEFAULT_CROSSTALK_ENABLE	0

#define STMVL53LX_CFG_DEFAULT_OFFSET_CORRECTION_MODE \
	VL53LX_OFFSETCORRECTIONMODE_STANDARD

/** default Dmax mode */
#define STMVL53LX_CFG_DEFAULT_DMAX_MODE		VL53LX_DMAXMODE_CUSTCAL_DATA

/** default smudge correction enable value set to NONE fix ticket #823926 */
#define STMVL53LX_CFG_DEFAULT_SMUDGE_CORRECTION_MODE \
	VL53LX_SMUDGE_CORRECTION_NONE

/** @} */ /* ingroup vl53lx_config */

void st_gettimeofday(struct st_timeval *tv)
{
	struct timespec64 now;

	ktime_get_ts64(&now);
	tv->tv_sec = now.tv_sec;
	tv->tv_usec = now.tv_nsec/1000;
}

/* #define DEBUG_TIME_LOG */


#ifdef DEBUG_TIME_LOG
struct st_timeval start_tv, stop_tv;
#endif

/* Set default value to 1 to allow to see module insertion debug messages */
int stmvl53lx_enable_debug = 1;

#define VL53LX_INPUT_DEVICE_NAME	"STM VL53LX proximity sensor"

static long stmvl53lx_ioctl(struct file *file,
		unsigned int cmd, unsigned long arg);
static int stmvl53lx_open(struct inode *inode, struct file *file);
static int stmvl53lx_release(struct inode *inode, struct file *file);
static int ctrl_start(struct stmvl53lx_data *data);
static int ctrl_stop(struct stmvl53lx_data *data);

static bool force_device_on_en_default = true;

module_param(force_device_on_en_default, bool, 0444);
MODULE_PARM_DESC(force_device_on_en_default,
	"select whether force_device_on_en is true or false by default");

/* boilerplate for integer parameter */
#define IMPLEMENT_PARAMETER_INTEGER(sysfs_name, info_name)\
static ssize_t stmvl53lx_show_##sysfs_name(struct device *dev, \
				struct device_attribute *attr, char *buf) \
{ \
	struct stmvl53lx_data *data = dev_get_drvdata(dev); \
	int param; \
\
	mutex_lock(&data->work_mutex); \
	param = data->sysfs_name; \
	mutex_unlock(&data->work_mutex);; \
\
	return scnprintf(buf, PAGE_SIZE, "%d\n", param); \
} \
\
static ssize_t stmvl53lx_store_##sysfs_name(struct device *dev, \
					struct device_attribute *attr, \
					const char *buf, size_t count) \
{ \
	struct stmvl53lx_data *data = dev_get_drvdata(dev); \
	int rc = 0; \
	int param; \
\
	mutex_lock(&data->work_mutex); \
\
	if (kstrtoint(buf, 0, &param)) { \
		vl53lx_errmsg("invalid syntax in %s", buf); \
		rc = -EINVAL; \
	} else \
		rc = stmvl53lx_set_##sysfs_name(data, param); \
\
	mutex_unlock(&data->work_mutex); \
\
	return rc ? rc : count; \
} \
\
static int ctrl_param_##sysfs_name(struct stmvl53lx_data *data, \
		struct stmvl53lx_parameter *param) \
{ \
	int rc = 0; \
\
	if (param->is_read) { \
		param->value = data->sysfs_name; \
		param->status = 0; \
		vl53lx_dbgmsg("get " info_name " %d", param->value); \
		rc = 0; \
	} else { \
		rc = stmvl53lx_set_##sysfs_name(data, param->value); \
		vl53lx_dbgmsg("rc %d req %d now %d", rc, \
				param->value, data->sysfs_name); \
	} \
\
	return rc; \
}

/**
 *  module interface struct
 *  interface to platform speficic device handling , concern power/reset ...
 */
struct stmvl53lx_module_fn_t {
	int (*init)(void);	/*!< init */
	/**
	 * clean up job
	 * @param data module specific data ptr
	 */
	void (*deinit)(void *data);
	/**
	 *  give device power
	 * @param data  specific module storage ptr
	 * @return 0 on sucess
	 */
	int (*power_up)(void *data);
	/**
	 *  power down TOFO also stop intr
	 */
	int (*power_down)(void *data);
	/*
	 * release reset so device start.
	 */
	int (*reset_release)(void *data);
	/*
	 * put device under reset.
	 */
	int (*reset_hold)(void *data);

	 /**
	  * enable interrupt
	  *
	  * @param object : interface speficic ptr
	  * @note "module specfic ptr is data->client_object
	  * @return 0 on success else error then drievr wan't start ranging!
	  * if no interrupt or it can't be hooked but still to operated in poll
	  * mode then return 0  and force data->poll_mode
	  * might have to clear poll_mode exlplcilty if to operate in real intr
	  * mode as pool mode
	  * is the default
	  */
	int (*start_intr)(void *object, int *poll_mode);

	void (*clean_up)(void); /*!< optional can be void */

	/* increment reference counter */
	void *(*get)(void *object);

	/* decrement reference counter and deallocate memory when zero */
	void (*put)(void *object);
};

/** i2c module interface*/
static struct stmvl53lx_module_fn_t stmvl53lx_module_func_tbl = {
	.init = stmvl53lx_init_i2c,
	.deinit = stmvl53lx_exit_i2c,
	.power_up = stmvl53lx_power_up_i2c,
	.power_down = stmvl53lx_power_down_i2c,
	.reset_release = stmvl53lx_reset_release_i2c,
	.reset_hold = stmvl53lx_reset_hold_i2c,
	.clean_up = stmvl53lx_clean_up_i2c,
	.start_intr = stmvl53lx_start_intr,
	.get = stmvl53lx_get,
	.put = stmvl53lx_put,
};


#ifndef MIN
#	define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

/*
 * INPUT Subsys interface
 */
static int sleep_for_data(struct stmvl53lx_data *data, pid_t pid,
				struct list_head *head);

static void stmvl53lx_input_push_data(struct stmvl53lx_data *data);

/*
 * Mutex to handle device id add/removal
 */
static DEFINE_MUTEX(dev_table_mutex);

/**
 * in-used device LUT
 * we need this as the message reception from netlink message can't
 * associate directly to a device instance that is as we look up id
 * to device data structure
 */
struct stmvl53lx_data *stmvl53lx_dev_table[STMVL53LX_CFG_MAX_DEV];

/**
 * Misc device device operations
 */
static const struct file_operations stmvl53lx_ranging_fops = {
	.owner =		THIS_MODULE,
	.unlocked_ioctl =	stmvl53lx_ioctl,
	.open =			stmvl53lx_open,
	.release =		stmvl53lx_release,
};

static int store_last_error(struct stmvl53lx_data *data, int rc)
{
	data->last_error = rc;

	return -EIO;
}

static int allocate_dev_id(void)
{
	int i;

	mutex_lock(&dev_table_mutex);

	for (i = 0; i < STMVL53LX_CFG_MAX_DEV; i++)
		if (!stmvl53lx_dev_table[i])
			break;
	i = i < STMVL53LX_CFG_MAX_DEV ? i : -1;

	mutex_unlock(&dev_table_mutex);

	return i;
}

static void deallocate_dev_id(int id)
{
	mutex_lock(&dev_table_mutex);

	stmvl53lx_dev_table[id] = NULL;

	mutex_unlock(&dev_table_mutex);
}

/* helpers to manage reader list for blockint ioctl */
/* call them with lock */
static void empty_and_free_list(struct list_head *head)
{
	struct stmvl53lx_waiters *waiter;
	struct stmvl53lx_waiters *tmp;

	list_for_each_entry_safe(waiter, tmp, head, list) {
		list_del(&waiter->list);
		kfree(waiter);
	}
}

static int add_reader(pid_t pid, struct list_head *head)
{
	struct stmvl53lx_waiters *new_waiter;

	new_waiter = kmalloc(sizeof(struct stmvl53lx_waiters), GFP_KERNEL);
	if (!new_waiter)
		return -ENOMEM;
	new_waiter->pid = pid;
	list_add(&new_waiter->list, head);

	return 0;
}

static bool is_pid_in_list(pid_t pid, struct list_head *head)
{
	struct stmvl53lx_waiters *waiter;

	list_for_each_entry(waiter, head, list)
		if (waiter->pid == pid)
			return true;

	return false;
}

static void wake_up_data_waiters(struct stmvl53lx_data *data)
{
	empty_and_free_list(&data->simple_data_reader_list);
	empty_and_free_list(&data->mz_data_reader_list);
	wake_up(&data->waiter_for_data);
}

static void stmvl53lx_insert_flush_events_lock(struct stmvl53lx_data *data)
{
	while (data->flush_todo_counter) {
		data->flushCount++;
		input_report_abs(data->input_dev_ps, ABS_GAS, data->flushCount);
		input_sync(data->input_dev_ps);
		vl53lx_dbgmsg("Sensor HAL Flush Count = %u\n",
			data->flushCount);
		data->flush_todo_counter--;
	}
}

static int reset_release(struct stmvl53lx_data *data)
{
	int rc = 0;

	if (!data->reset_state)
		return 0;

	rc = stmvl53lx_module_func_tbl.reset_release(data->client_object);
	if (rc)
		vl53lx_errmsg("reset release fail rc=%d\n", rc);
	else
		data->reset_state = 0;

	return rc;
}

static int reset_hold(struct stmvl53lx_data *data)
{
	int rc = 0;

	if (data->reset_state)
		return 0;

	if (data->force_device_on_en)
		return 0;

	rc = stmvl53lx_module_func_tbl.reset_hold(data->client_object);
	if (!rc)
		data->reset_state = 1;

	return rc;
}

/**
 *
 * @param pstart_tv time val  starting point
 * @param pstop_tv time val  end point
 * @return time dif in usec
 */
long stmvl53lx_tv_dif(struct st_timeval *pstart_tv, struct st_timeval *pstop_tv)
{
	long total_sec, total_usec;

	total_sec = pstop_tv->tv_sec - pstart_tv->tv_sec;
	total_usec = (pstop_tv->tv_usec - pstart_tv->tv_usec);

	return total_sec*1000000+total_usec;
}

static int setup_tunings(struct stmvl53lx_data *data)
{
	int rc = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(tunings); i++) {
		rc = VL53LX_SetTuningParameter(&data->stdev, tunings[i][0],
			tunings[i][1]);
		if (rc) {
			rc = store_last_error(data, rc);
			break;
		}
	}

	return rc;
}

static void kill_mz_data(VL53LX_MultiRangingData_t *pdata)
{
	int i;

	memset(pdata, 0, sizeof(*pdata));
	for (i = 0; i < VL53LX_MAX_RANGE_RESULTS; i++)
		pdata->RangeData[i].RangeStatus = VL53LX_RANGESTATUS_NONE;
}

/**
 * send params to sensor
 *
 * @warning must be used if only stopped
 * @param data device data
 * @return 0 on sucess
 */
static int stmvl53lx_sendparams(struct stmvl53lx_data *data)
{
	int rc = 0;

	rc = VL53LX_SetXTalkCompensationEnable(&data->stdev,
		data->crosstalk_enable);
	if (rc) {
		vl53lx_errmsg("VL53LX_SetXTalkCompensationEnable %d fail %d",
				data->crosstalk_enable, rc);
		rc = store_last_error(data, rc);
		goto done;
	}
	vl53lx_dbgmsg("Xtalk enable @%d\n", data->crosstalk_enable);

	/* apply distance mode only in lite and standard ranging */
	rc = VL53LX_SetDistanceMode(&data->stdev, data->distance_mode);
	if (rc) {
		vl53lx_errmsg("VL53LX_SetDistanceMode %d fail %d",
			data->distance_mode, rc);
		rc = store_last_error(data, rc);
		goto done;
	}
	vl53lx_dbgmsg("distance mode @%d\n", data->distance_mode);

	/* apply timing budget */
	rc = VL53LX_SetMeasurementTimingBudgetMicroSeconds(&data->stdev,
			data->timing_budget);
	if (rc) {
		vl53lx_errmsg("SetTimingBudget %d fail %d",
				data->timing_budget, rc);
		rc = store_last_error(data, rc);
		goto done;
	}
	vl53lx_dbgmsg("timing budget @%d\n", data->timing_budget);

	/* apply offset correction mode */
	rc = VL53LX_SetOffsetCorrectionMode(&data->stdev,
			data->offset_correction_mode);
	if (rc) {
		vl53lx_errmsg("offset correction mode %d fail %d",
				data->offset_correction_mode, rc);
		rc = store_last_error(data, rc);
		goto done;
	}
	vl53lx_dbgmsg("offset correction mode @%d\n",
		data->offset_correction_mode);

	/* apply smudge correction enable */
	rc =  VL53LX_SmudgeCorrectionEnable(&data->stdev,
		data->smudge_correction_mode);
	if (rc) {
		vl53lx_errmsg("smudge correction mode %d fail %d",
			data->smudge_correction_mode, rc);
		rc = store_last_error(data, rc);
		goto done;
	}
	vl53lx_dbgmsg("smudge correction mode @%d\n",
		data->smudge_correction_mode);

	/* apply roi */
	rc = VL53LX_SetUserROI(&data->stdev, &data->roi_cfg);
	if (rc) {
		vl53lx_errmsg("VL53LX_SetUserROI fail %d\n", rc);
		rc = store_last_error(data, rc);
		goto done;
	}
	vl53lx_dbgmsg("ROI set TopLeft(%d %d) BottomRight(%d %d)\n",
		data->roi_cfg.TopLeftX,
		data->roi_cfg.TopLeftY,
		data->roi_cfg.BotRightX,
		data->roi_cfg.BotRightY);
	
done:

	return rc;
}

/**
 * start sensor
 *
 * @warning must be used if only stopped
 * @param data device data
 * @return 0 on sucess
 */
static int stmvl53lx_start(struct stmvl53lx_data *data)
{
	int rc = 0;

	data->is_first_irq = true;
	data->is_data_valid = false;
	data->is_xtalk_value_changed = false;
	data->wait_first_frame = true;

	rc = reset_release(data);
	if (rc)
		goto done;

	rc = stmvl53lx_sendparams(data);
	if (rc)
		goto done;

	/* init the timing  */
	st_gettimeofday(&data->start_tv);
	data->meas.start_tv = data->start_tv;
	/* init the ranging data => kill the previous ranging mz data */
	kill_mz_data(&data->meas.multi_range_data);

	data->allow_hidden_start_stop = false;
	/* kick off ranging */
	rc = VL53LX_StartMeasurement(&data->stdev);
	if (rc) {
		vl53lx_errmsg("VL53LX_StartMeasurement @%d fail %d",
				__LINE__, rc);
		rc = store_last_error(data, rc);
		goto done;
	}

	data->meas.cnt = 0;
	data->meas.err_cnt = 0;
	data->meas.err_tot = 0;
	data->meas.poll_cnt = 0;
	data->meas.intr = 0;
	data->enable_sensor = 1;
	if (data->poll_mode) {
		/* kick off the periodical polling work */
		schedule_delayed_work(&data->dwork,
			msecs_to_jiffies(data->poll_delay_ms));
	}

done:
	data->is_first_start_done = true;

	return rc;
}

/**
 * stop sensor
 *
 * work lock must be held
 * @warning to be used if only started!
 */
static int stmvl53lx_stop(struct stmvl53lx_data *data)
{
	int rc = 0;

	rc = VL53LX_StopMeasurement(&data->stdev);
	if (rc) {
		vl53lx_errmsg("VL53LX_StopMeasurement @%d fail %d",
				__LINE__, rc);
		rc = store_last_error(data, rc);
	}
	/* put device under reset */
	/* do we ask explicit intr stop or just use stop */
	reset_hold(data);

	data->enable_sensor = 0;
	/* fix ticket 1458514 deadlock multiple devices, cancel pending data collection */
	cancel_delayed_work(&data->dwork);

	/* wake up all waiters */
	/* they will receive -ENODEV error */
	wake_up_data_waiters(data);

	return rc;
}

/*
 * SysFS support
 */
static ssize_t stmvl53lx_show_enable_ps_sensor(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct stmvl53lx_data *data = dev_get_drvdata(dev);

	return snprintf(buf, 5, "%d\n", data->enable_sensor);
}

static ssize_t stmvl53lx_store_enable_ps_sensor(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct stmvl53lx_data *data = dev_get_drvdata(dev);
	int rc = 0;
	unsigned long val;

	rc = kstrtoul(buf, 10, &val);
	if (rc) {
		vl53lx_errmsg("enable sensor syntax in %s\n", buf);
		return -EINVAL;
	}
	if (val == 1) {
		rc = ctrl_start(data);
	} else if (val == 0) {
		rc = ctrl_stop(data);
	} else {
		/* TODO: Remove this workaround after investigation */
		/* see Codex - 479397 for details */
		vl53lx_dbgmsg("Unclog Input sub-system\n");
		/* Unclog the input device sub-system */
		input_report_abs(data->input_dev_ps, ABS_HAT0X, -1);
		input_report_abs(data->input_dev_ps, ABS_HAT0Y, -1);
		input_report_abs(data->input_dev_ps, ABS_HAT1X, -1);
		input_report_abs(data->input_dev_ps, ABS_HAT1Y, -1);
		input_report_abs(data->input_dev_ps, ABS_HAT2X, -1);
		input_report_abs(data->input_dev_ps, ABS_HAT2Y, -1);
		input_report_abs(data->input_dev_ps, ABS_HAT3X, -1);
		input_report_abs(data->input_dev_ps, ABS_HAT3Y, -1);
		input_report_abs(data->input_dev_ps, ABS_WHEEL, -1);
		input_report_abs(data->input_dev_ps, ABS_BRAKE, -1);
		input_report_abs(data->input_dev_ps, ABS_GAS, -1);
		input_report_abs(data->input_dev_ps, ABS_TILT_X, -1);
		input_report_abs(data->input_dev_ps, ABS_TILT_Y, -1);
		input_report_abs(data->input_dev_ps, ABS_TOOL_WIDTH, -1);
		input_report_abs(data->input_dev_ps, ABS_DISTANCE, -1);
		input_report_abs(data->input_dev_ps, ABS_THROTTLE, -1);
		input_report_abs(data->input_dev_ps, ABS_RUDDER, -1);
		input_report_abs(data->input_dev_ps, ABS_MISC, -1);
		input_report_abs(data->input_dev_ps, ABS_VOLUME,
									-1);
		input_sync(data->input_dev_ps);
		vl53lx_dbgmsg("Unclog the input sub-system\n");
		rc = 0;
	}

	vl53lx_dbgmsg("End\n");

	return rc ? rc : count;
}

/**
 * sysfs attribute "enable_ps_sensor" [rd/wr]
 *
 * @li read show the current enable state
 * @li write set the new state value "0" put sensor off "1"  put it on
 *
 * @return  0 on success , EINVAL if fail to start
 *
 * @warning their's no check and assume exclusive usage of sysfs and ioctl\n
 * Sensor will be put on/off disregard of any setup done by the ioctl channel.
 *
 * @ingroup sysfs_attrib
 */
static DEVICE_ATTR(enable_ps_sensor, 0664/*S_IWUGO | S_IRUGO*/,
		stmvl53lx_show_enable_ps_sensor,
		stmvl53lx_store_enable_ps_sensor);

static ssize_t stmvl53lx_show_unique_id(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct stmvl53lx_data *data = dev_get_drvdata(dev);
	uint64_t param;

	mutex_lock(&data->work_mutex);
	param = data->unique_id;
	mutex_unlock(&data->work_mutex);

	return scnprintf(buf, PAGE_SIZE, "0X%llx\n", param);
}

/**
 * sysfs attribute " unique_id" [rd]
 *
 * Will get unique_id using sysfs.
 *
 * @ingroup sysfs_attrib
 */
static DEVICE_ATTR(unique_id, 0440/*S_IRUGO*/,
				stmvl53lx_show_unique_id,
				NULL);

static int stmvl53lx_set_poll_delay_ms(struct stmvl53lx_data *data, int delay)
{
	int rc = 0;

	if (delay <= 0)
		rc = -EINVAL;
	else
		data->poll_delay_ms = delay;

	return rc;
}

IMPLEMENT_PARAMETER_INTEGER(poll_delay_ms, "poll delay ms")

/**
 * sysfs attribute "poll_delay_ms" [rd/wr]
 *
 * @li read show the current polling delay in millisecond
 * @li write set the new polling delay in millisecond
 *
 * @note apply only if device is in polling mode\n
 * for best performances (minimal delay and cpu load ) set it to the device
 * period operating period +1 millis

 * @ingroup sysfs_attrib
 */
static DEVICE_ATTR(set_delay_ms, 0660/*S_IWUGO | S_IRUGO*/,
				stmvl53lx_show_poll_delay_ms,
				stmvl53lx_store_poll_delay_ms);

/* Timing Budget */
static int stmvl53lx_set_timing_budget(struct stmvl53lx_data *data, int timing)
{
	int rc = 0;

	if (timing <= 0) {
		vl53lx_errmsg("invalid timing valid %d\n", timing);
		rc = -EINVAL;
	} else if (data->enable_sensor) {
		rc = VL53LX_SetMeasurementTimingBudgetMicroSeconds(&data->stdev,
			timing);
		if (rc) {
			vl53lx_errmsg("SetTimingBudget %d fail %d", timing, rc);
			rc = store_last_error(data, rc);
		} else
			data->timing_budget = timing;
	} else
		data->timing_budget = timing;

	return rc;
}

IMPLEMENT_PARAMETER_INTEGER(timing_budget, "timing budget")

/**
 * sysfs "timing_budget"  [rd/wr]
 *
 *  set or get the ranging timing budget in microsecond
 *
 * @ingroup sysfs_attrib
 */
static DEVICE_ATTR(timing_budget, 0660/*S_IWUGO | S_IRUGO*/,
			stmvl53lx_show_timing_budget,
			stmvl53lx_store_timing_budget);


static ssize_t stmvl53lx_show_roi(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct stmvl53lx_data *data = dev_get_drvdata(dev);
	int n;

	mutex_lock(&data->work_mutex);
	n = scnprintf(buf, PAGE_SIZE, "%d %d %d %d\n",
			data->roi_cfg.TopLeftX,
			data->roi_cfg.TopLeftY,
			data->roi_cfg.BotRightX,
			data->roi_cfg.BotRightY);
	mutex_unlock(&data->work_mutex);
	return n;
}


static ssize_t stmvl53lx_store_roi(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct stmvl53lx_data *data = dev_get_drvdata(dev);
	int rc;
	int n;
	int tlx, tly, brx, bry;

	mutex_lock(&data->work_mutex);
	if (data->enable_sensor) {
		vl53lx_errmsg("ERROR can't set roi while ranging");
		rc = -EBUSY;
	} else {
		n = sscanf(buf, "%d %d %d %d", &tlx, &tly, &brx, &bry);
		if (n == 4) {
			data->roi_cfg.TopLeftX = tlx;
			data->roi_cfg.TopLeftY = tly;
			data->roi_cfg.BotRightX = brx;
			data->roi_cfg.BotRightY = bry;
			rc = count;
			vl53lx_dbgmsg("ROI %2d %2d %2d %2d\n", 
				(int)data->roi_cfg.TopLeftX, (int)data->roi_cfg.TopLeftY,
				(int)data->roi_cfg.BotRightX, (int)data->roi_cfg.BotRightY);
		} else {
			vl53lx_errmsg("wrong roi syntax %s ", buf);
			rc = -EINVAL;
		}
	}
	mutex_unlock(&data->work_mutex);
	vl53lx_dbgmsg("ret %d count %d\n", rc, (int)count);

	return rc;
}

/**
 * sysfs attribute "roi" [rd/wr]
 *
 * @li read show the current user customized roi setting
 * @li write set user custom roi, it can only be done while not ranging.
 *
 * syntax for set input roi
 * @li "tlx tly brx bry\n" will set the roi
  *
 * @warning roi coordinate is not image x,y(down) but euclidian x,y(up)
 *
 * @warning roi validity is only check at next range start
 *
 *@code
 * >#to set roi
 * >echo "0 15 15 0" > /sys/class/input6/roi
 * >echo $?
 * 0
 * >cat /sys/class/input6/roi
 * "0 15 15 0"
 * >echo "1" > /sys/class/input6/enable_ps_sensor
 * #try to set roi while ranging
 * >echo "0 15 15 0" > /sys/class/input6/roi
 * [58451.912109] stmvl53lx_store_roi:  ERROR can't set roi while ranging
 * >echo $?
 * 1
 *@endcode
 * @ingroup sysfs_attrib
 */
static DEVICE_ATTR(roi, 0660/*S_IWUGO | S_IRUGO*/,
				stmvl53lx_show_roi,
				stmvl53lx_store_roi);


static int stmvl53lx_set_distance_mode(struct stmvl53lx_data *data,
	int distance_mode)
{
	int rc = 0;

	if (data->enable_sensor) {
		vl53lx_errmsg("can't change distance mode while ranging\n");
		rc = -EBUSY;
	} else {
		switch (distance_mode) {
		case VL53LX_DISTANCEMODE_SHORT:
		case VL53LX_DISTANCEMODE_MEDIUM:
		case VL53LX_DISTANCEMODE_LONG:
			data->distance_mode = distance_mode;
			break;
		default:
			vl53lx_errmsg("invalid distance mode %d\n",
				distance_mode);
			rc = -EINVAL;
			break;
		}
	}

	return rc;
}

IMPLEMENT_PARAMETER_INTEGER(distance_mode, "distance mode")

/**
 * sysfs attribute " distance mode" [rd/wr]
 *
 * set the distance mode value can only be used while: not ranging
 * @li 1 @a VL53LX_DISTANCEMODE_SHORT
 * @li 2 @a VL53LX_DISTANCEMODE_MEDIUM
 * @li 3 @a VL53LX_DISTANCEMODE_LONG
 *
 * @ingroup sysfs_attrib
 */
static DEVICE_ATTR(distance_mode, 0660/*S_IWUGO | S_IRUGO*/,
				stmvl53lx_show_distance_mode,
				stmvl53lx_store_distance_mode);

static int stmvl53lx_set_crosstalk_enable(struct stmvl53lx_data *data,
	int crosstalk_enable)
{
	int rc = 0;

	if (data->enable_sensor) {
		vl53lx_errmsg("can't change crosstalk enable while ranging\n");
		rc = -EBUSY;
	} else if (crosstalk_enable == 0 || crosstalk_enable == 1) {
		data->crosstalk_enable = crosstalk_enable;
	} else {
		vl53lx_errmsg("invalid crosstalk enable %d\n",
			crosstalk_enable);
		rc = -EINVAL;
	}

	return rc;
}

IMPLEMENT_PARAMETER_INTEGER(crosstalk_enable, "crosstalk enable")

/**
 * sysfs attribute " crosstalk enable" [rd/wr]
 *
 * control if crosstalk compensation is eanble or not
 * @li 0 disable crosstalk compensation
 * @li 1 enable crosstalk compensation
 *
 * @ingroup sysfs_attrib
 */
static DEVICE_ATTR(crosstalk_enable, 0660/*S_IWUGO | S_IRUGO*/,
				stmvl53lx_show_crosstalk_enable,
				stmvl53lx_store_crosstalk_enable);


static int stmvl53lx_set_force_device_on_en(struct stmvl53lx_data *data,
	int force_device_on_en)
{
	int rc = 0;

	if (force_device_on_en != 0 && force_device_on_en != 1) {
		vl53lx_errmsg("invalid force_device_on_en mode %d\n",
			force_device_on_en);
		return -EINVAL;
	}

	data->force_device_on_en = force_device_on_en;

	/* don't update reset if sensor is enable */
	if (data->enable_sensor)
		return 0;

	/* ok update reset according force_device_on_en value */
	if (force_device_on_en)
		rc = reset_release(data);
	else
		rc = reset_hold(data);

	return rc;
}

IMPLEMENT_PARAMETER_INTEGER(force_device_on_en, "force device on enable")

/**
 * sysfs attribute " force_device_on_enable" [rd/wr]
 *
 * Control if device is put under reset when stopped.
 * @li 0 feature is disable. Device is put under reset when stopped.
 * @li 1 feature is enable. Device is not put under reset when stopped.
 *
 * @ingroup sysfs_attrib
 */
static DEVICE_ATTR(force_device_on_enable, 0660/*S_IWUGO | S_IRUGO*/,
				stmvl53lx_show_force_device_on_en,
				stmvl53lx_store_force_device_on_en);

static int stmvl53lx_set_offset_correction_mode(struct stmvl53lx_data *data,
	int offset_correction_mode)
{
	int rc = 0;

	if (data->enable_sensor) {
		vl53lx_errmsg(
			"can't change offset correction mode while ranging\n");
		rc = -EBUSY;
	} else {
		switch (offset_correction_mode) {
		case VL53LX_OFFSETCORRECTIONMODE_STANDARD:
		case VL53LX_OFFSETCORRECTIONMODE_PERVCSEL:
			data->offset_correction_mode = offset_correction_mode;
			break;
		default:
			vl53lx_errmsg("invalid offset correction mode %d\n",
				offset_correction_mode);
			rc = -EINVAL;
			break;
		}
	}

	return rc;
}

IMPLEMENT_PARAMETER_INTEGER(offset_correction_mode, "offset correction mode")

/**
 * sysfs attribute " offset_correction_mode" [rd/wr]
 *
 * Control which offset correction is apply on result. can only be used
 * while: not ranging
 * @li 1 @a VL53LX_OFFSETCORRECTIONMODE_STANDARD
 * @li 3 @a VL53LX_OFFSETCORRECTIONMODE_PERVCSEL
 *
 * @ingroup sysfs_attrib
 */
static DEVICE_ATTR(offset_correction_mode, 0660/*S_IWUGO | S_IRUGO*/,
				stmvl53lx_show_offset_correction_mode,
				stmvl53lx_store_offset_correction_mode);

static ssize_t stmvl53lx_do_flush(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct stmvl53lx_data *data = dev_get_drvdata(dev);

	mutex_lock(&data->work_mutex);

	data->flush_todo_counter++;
	if (data->enable_sensor == 0)
		stmvl53lx_insert_flush_events_lock(data);

	mutex_unlock(&data->work_mutex);

	return count;
}

static DEVICE_ATTR(do_flush, 0660/*S_IWUGO | S_IRUGO*/,
			NULL,
			stmvl53lx_do_flush);

static ssize_t stmvl53lx_show_enable_debug(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", stmvl53lx_enable_debug);
}

static ssize_t stmvl53lx_store_enable_debug(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int enable_debug;
	int rc = 0;

	if (kstrtoint(buf, 0, &enable_debug)) {
		vl53lx_errmsg("invalid syntax in %s", buf);
		rc = -EINVAL;
	} else
		stmvl53lx_enable_debug = enable_debug;

	return rc ? rc : count;
}

/**
 * sysfs attribute " debug enable" [rd/wr]
 *
 * dynamic control of vl53lx_dbgmsg messages. Note that in any case your code
 * must be enable with DEBUG in stmvl53lx.h at compile time.
 * @li 0 disable vl53lx_dbgmsg messages
 * @li 1 enable vl53lx_dbgmsg messages
 *
 * @ingroup sysfs_attrib
 */
static DEVICE_ATTR(enable_debug, 0660/*S_IWUGO | S_IRUGO*/,
				stmvl53lx_show_enable_debug,
				stmvl53lx_store_enable_debug);

static ssize_t stmvl53lx_show_last_error_config(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct stmvl53lx_data *data = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", data->last_error);
}

/**
 * sysfs attribute " last_error" [rd]
 *
 * Will get last internal error using sysfs.
 *
 * @ingroup sysfs_attrib
 */
static DEVICE_ATTR(last_error, 0440/*S_IRUGO*/,
				stmvl53lx_show_last_error_config,
				NULL);

static ssize_t display_FixPoint1616(char *buf, size_t size, FixPoint1616_t fix)
{
	uint32_t msb = fix >> 16;
	uint32_t lsb = fix & 0xffff;

	lsb = (lsb * 1000000ULL + 32768) / 65536;

	return scnprintf(buf, size, "%d.%06d", msb, (uint32_t) lsb);
}

static ssize_t stmvl53lx_show_optical_center_config(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct stmvl53lx_data *data = dev_get_drvdata(dev);
	ssize_t res = 0;

	res += display_FixPoint1616(&buf[res], PAGE_SIZE - res,
		data->optical_offset_x);
	res += scnprintf(&buf[res], PAGE_SIZE - res, " ");
	res += display_FixPoint1616(&buf[res], PAGE_SIZE - res,
		data->optical_offset_y);

	res += scnprintf(&buf[res], PAGE_SIZE - res, "\n");

	return res;
}

/**
 * sysfs attribute " optical_center" [rd]
 *
 * Will get optical_center using sysfs.
 *
 * @ingroup sysfs_attrib
 */
static DEVICE_ATTR(optical_center, 0440/*S_IRUGO*/,
				stmvl53lx_show_optical_center_config,
				NULL);

static int stmvl53lx_set_tuning(struct stmvl53lx_data *data, int key,
	int value)
{
	int rc = 0;

	if (data->enable_sensor) {
		vl53lx_errmsg("can't change tuning params while ranging\n");
		return -EBUSY;
	}

	if (data->is_calibrating) {
		vl53lx_errmsg("can't change tuning params while calibrating\n");
		return -EBUSY;
	}

	if (key & ~0xffff)
		return -EINVAL;

	vl53lx_dbgmsg("trying to set %d with key %d", value, key);

	rc = VL53LX_SetTuningParameter(&data->stdev, key, value);
	if (rc)
		rc = store_last_error(data, rc);

	return rc;
}

static ssize_t stmvl53lx_store_tuning(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct stmvl53lx_data *data = dev_get_drvdata(dev);
	int key;
	int value;
	int n;
	int rc = 0;

	mutex_lock(&data->work_mutex);

	n = sscanf(buf, "%d %d", &key, &value);
	if (n != 2) {
		rc = -EINVAL;
		goto error;
	}
	rc = stmvl53lx_set_tuning(data, key, value);
	if (rc)
		goto error;

	mutex_unlock(&data->work_mutex);

	return count;

error:
	mutex_unlock(&data->work_mutex);

	return rc;
}

/**
 * sysfs attribute "tuning" [wr]
 *
 * write a tuning parameter. Two integer parameters are given. First one
 * is a key that specify which tuning parameter is update. Other one is the
 * value which is write.
 *
 * writing a tuning parameter is only allowed before the first start.
 *
 * @ingroup sysfs_attrib
 */
static DEVICE_ATTR(tuning, 0220/*S_IWUGO */,
				NULL,
				stmvl53lx_store_tuning);

static int stmvl53lx_display_tuning_key(struct stmvl53lx_data *data, char *buf,
	int *pos, int key)
{
	int rc = 0;
	int value = 0;
	int sz;

	rc = VL53LX_GetTuningParameter(&data->stdev, key, &value);
	if (rc)
		return 0;

	sz = snprintf(&buf[*pos], PAGE_SIZE - *pos, "%d %d\n", key, value);
	if (sz >= PAGE_SIZE - *pos)
		return -ENOSPC; /* FIXME : another better error ? */

	*pos += sz;

	return 0;
}

static ssize_t stmvl53lx_show_tuning_status(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	const int max_tuning_key = 65535;
	struct stmvl53lx_data *data = dev_get_drvdata(dev);
	int rc = 0;
	int i;
	int pos = 0;

	mutex_lock(&data->work_mutex);

	for (i = 0; i < max_tuning_key; ++i) {
		rc = stmvl53lx_display_tuning_key(data, buf, &pos, i);
		if (rc)
			break;
	}

	mutex_unlock(&data->work_mutex);

	return rc ? rc : pos;
}

/**
 * sysfs attribute "tuning_status" [rd]
 *
 * write a tuning parameter. Two integer parameters are given. First one
 * is a key that specify which tuning parameter is update. Other one is the
 * value which is write.
 *
 * writing a tuning parameter is only allowed before the first start.
 *
 * @ingroup sysfs_attrib
 */
static DEVICE_ATTR(tuning_status, 0440/*S_IRUGO */,
				stmvl53lx_show_tuning_status,
				NULL);

static int stmvl53lx_set_smudge_correction_mode(struct stmvl53lx_data *data,
	int smudge_correction_mode)
{
	int rc = 0;

	if (data->enable_sensor) {
		vl53lx_errmsg("can't change smudge corr mode while ranging\n");
		rc = -EBUSY;
	} else {
		switch (smudge_correction_mode) {
		case VL53LX_SMUDGE_CORRECTION_NONE:
		case VL53LX_SMUDGE_CORRECTION_CONTINUOUS:
		case VL53LX_SMUDGE_CORRECTION_SINGLE:
		case VL53LX_SMUDGE_CORRECTION_DEBUG:
			data->smudge_correction_mode = smudge_correction_mode;
			break;
		default:
			vl53lx_errmsg("invalid smudge correction mode %d\n",
				smudge_correction_mode);
			rc = -EINVAL;
			break;
		}
	}

	return rc;
}

IMPLEMENT_PARAMETER_INTEGER(smudge_correction_mode, "smudge correction mode")

/**
 * sysfs attribute " smudge_correction_mode" [rd/wr]
 *
 * This parameter will control if smudge correction is enable and how crosstalk
 * values are updated.
 * @li 0 @a VL53LX_SMUDGE_CORRECTION_NONE
 * @li 1 @a VL53LX_SMUDGE_CORRECTION_CONTINUOUS
 * @li 2 @a VL53LX_SMUDGE_CORRECTION_SINGLE
 * @li 3 @a VL53LX_SMUDGE_CORRECTION_DEBUG
 *
 * @ingroup sysfs_attrib
 */
static DEVICE_ATTR(smudge_correction_mode, 0660/*S_IWUGO | S_IRUGO*/,
				stmvl53lx_show_smudge_correction_mode,
				stmvl53lx_store_smudge_correction_mode);

static ssize_t stmvl53lx_show_is_xtalk_value_changed_config(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct stmvl53lx_data *data = dev_get_drvdata(dev);
	int param;

	mutex_lock(&data->work_mutex);
	param = data->is_xtalk_value_changed;
	mutex_unlock(&data->work_mutex);

	return scnprintf(buf, PAGE_SIZE, "%d\n", param);
}

/**
 * sysfs attribute " is_xtalk_value_changed" [rd]
 *
 * Will get is_xtalk_value_changed using sysfs.
 *
 * @ingroup sysfs_attrib
 */
static DEVICE_ATTR(is_xtalk_value_changed, 0440/*S_IRUGO*/,
				stmvl53lx_show_is_xtalk_value_changed_config,
				NULL);

static struct attribute *stmvl53lx_attributes[] = {
	&dev_attr_enable_ps_sensor.attr,
	&dev_attr_set_delay_ms.attr,
	&dev_attr_timing_budget.attr,
	&dev_attr_roi.attr,
	&dev_attr_do_flush.attr,
	&dev_attr_distance_mode.attr,
	&dev_attr_crosstalk_enable.attr,
	&dev_attr_enable_debug.attr,
	&dev_attr_force_device_on_enable.attr,
	&dev_attr_last_error.attr,
	&dev_attr_offset_correction_mode.attr,
	&dev_attr_optical_center.attr,
	&dev_attr_tuning.attr,
	&dev_attr_tuning_status.attr,
	&dev_attr_smudge_correction_mode.attr,
	&dev_attr_is_xtalk_value_changed.attr,
	&dev_attr_unique_id.attr,
	NULL
};

static const struct attribute_group stmvl53lx_attr_group = {
	.attrs = stmvl53lx_attributes,
};

static ssize_t stmvl53lx_calib_data_read(struct file *filp,
	struct kobject *kobj, struct bin_attribute *attr,
	char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct stmvl53lx_data *data = dev_get_drvdata(dev);
	VL53LX_CalibrationData_t calib;
	int rc = 0;
	void *src = (void *) &calib;

	mutex_lock(&data->work_mutex);

	vl53lx_dbgmsg("off = %lld / count = %d", off, count);

	/* sanity check */
	if (off < 0 || off > sizeof(VL53LX_CalibrationData_t))
		goto invalid;

	/* got current calibration data */
	memset(&calib, 0, sizeof(calib));
	rc = VL53LX_GetCalibrationData(&data->stdev, &calib);
	if (rc) {
		vl53lx_errmsg("VL53LX_GetCalibrationData fail %d", rc);
		rc = store_last_error(data, rc);
		goto error;
	}

	/* copy to buffer */
	if (off + count > sizeof(VL53LX_CalibrationData_t))
		count = sizeof(VL53LX_CalibrationData_t) - off;
	memcpy(buf, src + off, count);

	mutex_unlock(&data->work_mutex);

	return count;

invalid:
	vl53lx_errmsg("invalid syntax");
	rc = -EINVAL;
	goto error;

error:
	mutex_unlock(&data->work_mutex);

	return rc;
}

static ssize_t stmvl53lx_calib_data_write(struct file *filp,
	struct kobject *kobj, struct bin_attribute *attr,
	char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct stmvl53lx_data *data = dev_get_drvdata(dev);
	int rc = 0;

	mutex_lock(&data->work_mutex);

	vl53lx_dbgmsg("off = %lld / count = %d", off, count);

	if (data->enable_sensor) {
		rc = -EBUSY;
		vl53lx_errmsg("can't set calib data while ranging\n");
		goto error;
	}

	/* we only support one time write */
	if (off != 0 || count != sizeof(VL53LX_CalibrationData_t))
		goto invalid;

	rc = VL53LX_SetCalibrationData(&data->stdev,
		(VL53LX_CalibrationData_t *) buf);
	if (rc) {
		vl53lx_errmsg("VL53LX_SetCalibrationData fail %d", rc);
		rc = store_last_error(data, rc);
		goto error;
	}

	mutex_unlock(&data->work_mutex);

	return count;

invalid:
	vl53lx_errmsg("invalid syntax");
	rc = -EINVAL;
	goto error;

error:
	mutex_unlock(&data->work_mutex);

	return rc;
}

static struct bin_attribute stmvl53lx_calib_data_attr = {
	.attr = {
		.name = "calibration_data",
		.mode = 0660/*S_IWUGO | S_IRUGO*/,
	},
	.size = sizeof(VL53LX_CalibrationData_t),
	.read = stmvl53lx_calib_data_read,
	.write = stmvl53lx_calib_data_write,
};


static int ctrl_reg_access(struct stmvl53lx_data *data, void *p)
{
	struct stmvl53lx_register reg;
	size_t total_byte;
	int rc = 0;

	if (data->is_device_remove)
		return -ENODEV;

	/* fix ticket #1466615 Race Condition (TOCTOU/Double Fetch) */
	total_byte = sizeof(struct stmvl53lx_register);
	if (copy_from_user(&reg, p, total_byte)) {
		vl53lx_errmsg("%d, fail\n", __LINE__);
		return -EFAULT;
	}

	if (reg.cnt > STMVL53LX_MAX_CCI_XFER_SZ) {
		vl53lx_errmsg("reg len %d > size limit\n", reg.cnt);
		return -EINVAL;
	}

	/* put back to user only needed amount of data */
	if (!reg.is_read) {
		rc = VL53LX_WriteMulti(&data->stdev, (uint16_t)reg.index,
				reg.data.bytes, reg.cnt);
		reg.status = rc;
		/* for write only write back status no data */
		total_byte = offsetof(struct stmvl53lx_register, data.b);
		vl53lx_dbgmsg("wr %x %d bytes statu %d\n",
				reg.index, reg.cnt, rc);
		if (rc)
			rc = store_last_error(data, rc);
	} else {
		rc = VL53LX_ReadMulti(&data->stdev, (uint16_t)reg.index,
				reg.data.bytes, reg.cnt);
		reg.status = rc;
		vl53lx_dbgmsg("rd %x %d bytes status %d\n",
				reg.index, reg.cnt, rc);
		/*  if fail do not copy back data only status */
		if (rc) {
			total_byte = offsetof(struct stmvl53lx_register,
					data.b);
			rc = store_last_error(data, rc);
		}
		else
		{ /* reduce the copy paylod to actual needed data(reg.cnt) */
			total_byte = offsetof(struct stmvl53lx_register,
				data.bytes[reg.cnt]);
		}
	}

	if (copy_to_user(p, &reg, total_byte)) {
		vl53lx_errmsg("%d, fail\n", __LINE__);
		return -EFAULT;
	}
	return rc;
}


/*
 *
 */
static int ctrl_start(struct stmvl53lx_data *data)
{
	int rc = 0;

	mutex_lock(&data->work_mutex);

	if (data->is_device_remove) {
		rc = -ENODEV;
		goto done;
	}

	vl53lx_dbgmsg(" state = %d\n", data->enable_sensor);

	/* turn on tof sensor only if it's not already started */
	if (data->enable_sensor == 0 && !data->is_calibrating) {
		/* to start */
		rc = stmvl53lx_start(data);
	} else{
		rc = -EBUSY;
	}
	vl53lx_dbgmsg(" final state = %d\n", data->enable_sensor);

done:
	mutex_unlock(&data->work_mutex);

	return rc;
}

/**
 * no lock version of ctrl_stop (mutex shall be held)
 *
 * @warning exist only for use in device exit to ensure "locked and started"
 * may also beuse in some erro handling when mutex is already locked
 * @return 0 on success and was running >0 if already off <0 on error
 */
static int _ctrl_stop(struct stmvl53lx_data *data)
{
	int rc = 0;

	vl53lx_dbgmsg("enter state = %d\n", data->enable_sensor);
	/* be sure waiters are woken */
	data->is_data_valid = true;
	/* turn on tof sensor only if it's not enabled by other	client */
	if (data->enable_sensor == 1) {
		/* to stop */
		rc = stmvl53lx_stop(data);
	} else {
		vl53lx_dbgmsg("already off did nothing\n");
		rc = 0;
	}
	stmvl53lx_insert_flush_events_lock(data);
	vl53lx_dbgmsg("	final state = %d\n", data->enable_sensor);

	return rc;
}

/**
 * get work lock and stop sensor
 *
 * see @ref _ctrl_stop
 *
 * @param data device
 * @return 0 on success EBUSY if arleady off
 */
static int ctrl_stop(struct stmvl53lx_data *data)
{
	int rc = 0;

	mutex_lock(&data->work_mutex);

	if (data->is_device_remove) {
		rc = -ENODEV;
		goto done;
	}
	if (data->enable_sensor)
		rc = _ctrl_stop(data);
	else
		rc = -EBUSY;

done:
	mutex_unlock(&data->work_mutex);

	return rc;
}

static bool is_new_data_for_me(struct stmvl53lx_data *data, pid_t pid,
	struct list_head *head)
{
	return data->is_data_valid && !is_pid_in_list(pid, head);
}

static bool sleep_for_data_condition(struct stmvl53lx_data *data, pid_t pid,
	struct list_head *head)
{
	bool res;

	mutex_lock(&data->work_mutex);
	res = is_new_data_for_me(data, pid, head);
	mutex_unlock(&data->work_mutex);

	return res;
}

static int sleep_for_data(struct stmvl53lx_data *data, pid_t pid,
				struct list_head *head)
{
	int rc = 0;
	int32_t timeout;

	timeout = (2 * data->timing_budget);
	if ((data->wait_first_frame) && (timeout < TIMEOUT_FIRST_FRAME_USECS))
		timeout = TIMEOUT_FIRST_FRAME_USECS;
	mutex_unlock(&data->work_mutex);
	rc = wait_event_interruptible_timeout(data->waiter_for_data,
			sleep_for_data_condition(data, pid, head),
			usecs_to_jiffies(timeout));
	if (rc == 0) /* condition evaluated to false after timeout elapsed */
		rc = -EAGAIN;
	else 
		rc = 0;
	mutex_lock(&data->work_mutex);

	return data->enable_sensor ? rc : -ENODEV;
}

static int ctrl_mz_data_common(struct stmvl53lx_data *data, void __user *p,
	bool is_additional)
{
	struct stmvl53lx_data_with_additional __user *d = p;
	int rc = 0;

	mutex_lock(&data->work_mutex);
	if (data->is_device_remove) {
		rc = -ENODEV;
		goto done;
	}
	rc = copy_to_user(&d->data, &data->meas.multi_range_data,
		sizeof(VL53LX_MultiRangingData_t));
	if (rc) {
		vl53lx_dbgmsg("copy to user fail %d", rc);
		rc = -EFAULT;
		goto done;
	}
	if (is_additional) {
		rc = copy_to_user(&d->additional_data,
			&data->meas.additional_data,
			sizeof(VL53LX_AdditionalData_t));
		if (rc) {
			vl53lx_dbgmsg("copy to user fail %d", rc);
			rc = -EFAULT;
			goto done;
		}
	}
	if (!data->enable_sensor)
		rc = -ENODEV;

done:
	mutex_unlock(&data->work_mutex);

	return rc;
}

static int ctrl_mz_data_blocking_common(struct stmvl53lx_data *data,
	void __user *p, bool is_additional)
{
	int rc = 0;
	int rc0;
	struct stmvl53lx_data_with_additional __user *d = p;
	pid_t pid = current->pid;

	mutex_lock(&data->work_mutex);
	if (data->is_device_remove) {
		rc = -ENODEV;
		goto done;
	}
	/* If device not ranging then exit on error */
	if (!data->enable_sensor) {
		rc = -ENODEV;
		goto done;
	}
	/* sleep if data already read */
	if (!is_new_data_for_me(data, pid, &data->mz_data_reader_list))
		rc = sleep_for_data(data, pid, &data->mz_data_reader_list);
	if (rc) {
		kill_mz_data(&data->meas.multi_range_data);
		rc0 = copy_to_user(&d->data, &data->meas.multi_range_data,
			sizeof(VL53LX_MultiRangingData_t));
		goto done;
	}

	if (data->meas.multi_range_data.StreamCount == 0)
		data->wait_first_frame = false;
	/* unless we got interrupted we return data to user and note read */
	rc = copy_to_user(&d->data, &data->meas.multi_range_data,
		sizeof(VL53LX_MultiRangingData_t));
	if (rc)
		goto done;
	if (is_additional) {
		rc = copy_to_user(&d->additional_data,
			&data->meas.additional_data,
			sizeof(VL53LX_AdditionalData_t));
		if (rc)
			goto done;
	}
	rc = add_reader(pid, &data->mz_data_reader_list);

done:
	mutex_unlock(&data->work_mutex);

	return rc;
}

/**
 * Get multi zone data
 * @param data
 * @param p [out] user ptr to @ref VL53LX_MultiRangingData_t structure\n
 *  is always set but on EFAULT error
 *
 * @return
 * @li 0 on success
 * @li ENODEV if not ranging
 * @li ENOEXEC not in multi zone mode
 * @li EFAULT  copy to user error
 */
static int ctrl_mz_data(struct stmvl53lx_data *data, void __user *p)
{
	return ctrl_mz_data_common(data, p, false);
}

static int ctrl_mz_data_blocking(struct stmvl53lx_data *data, void __user *p)
{
	return ctrl_mz_data_blocking_common(data, p, false);
}

/**
 * Get multi zone data with histogram debug data
 * @param data
 * @param p [out] user ptr to @ref struct stmvl53lx_data_with_additional
 * structure is always set but on EFAULT error
 *
 * @return
 * @li 0 on success
 * @li ENODEV if not ranging
 * @li ENOEXEC not in multi zone mode
 * @li EFAULT  copy to user error
 */
static int ctrl_mz_data_additional(struct stmvl53lx_data *data, void __user *p)
{
	return ctrl_mz_data_common(data, p, true);
}

static int ctrl_mz_data_blocking_additional(struct stmvl53lx_data *data,
	void __user *p)
{
	return ctrl_mz_data_blocking_common(data, p, true);
}

static int ctrl_param_last_error(struct stmvl53lx_data *data,
		struct stmvl53lx_parameter *param)
{
	int rc = 0;

	if (param->is_read) {
		param->value = data->last_error;
		param->status = 0;
		vl53lx_dbgmsg("get last error %d", param->value);
		rc = 0;
	} else {
		rc = -EINVAL;
	}

	return rc;
}

static int ctrl_param_optical_center(struct stmvl53lx_data *data,
		struct stmvl53lx_parameter *param)
{
	if (!param->is_read)
		return -EINVAL;

	param->value = data->optical_offset_x;
	param->value2 = data->optical_offset_y;

	return 0;
}

static int ctrl_param_tuning(struct stmvl53lx_data *data,
		struct stmvl53lx_parameter *param)
{
	if (param->is_read)
		return -EINVAL;

	return stmvl53lx_set_tuning(data, param->value, param->value2);
}

static int ctrl_param_is_xtalk_value_changed(struct stmvl53lx_data *data,
		struct stmvl53lx_parameter *param)
{
	if (!param->is_read)
		return -EINVAL;

	param->value = data->is_xtalk_value_changed;

	return 0;
}

/**
 * handle ioctl set param mode
 *
 * @param data
 * @param p
 * @return 0 on success
 */
static int ctrl_params(struct stmvl53lx_data *data, void __user *p)
{
	int rc, rc2;
	struct stmvl53lx_parameter param;

	mutex_lock(&data->work_mutex);

	if (data->is_device_remove) {
		rc = -ENODEV;
		goto done;
	}
	rc = copy_from_user(&param, p, sizeof(param));
	param.status = 0;
	if (rc) {
		rc = -EFAULT;
		goto done; /* no need for status in user struct */
	}
	switch (param.name) {
	case VL53LX_POLLDELAY_PAR:
		rc = ctrl_param_poll_delay_ms(data, &param);
		break;
	case VL53LX_TIMINGBUDGET_PAR:
		rc = ctrl_param_timing_budget(data, &param);
		break;
	case VL53LX_DISTANCEMODE_PAR:
		rc = ctrl_param_distance_mode(data, &param);
	break;
	case VL53LX_XTALKENABLE_PAR:
		rc = ctrl_param_crosstalk_enable(data, &param);
	break;
	case VL53LX_FORCEDEVICEONEN_PAR:
		rc = ctrl_param_force_device_on_en(data, &param);
	break;
	case VL53LX_LASTERROR_PAR:
		rc = ctrl_param_last_error(data, &param);
	break;
	case VL53LX_OFFSETCORRECTIONMODE_PAR:
		rc = ctrl_param_offset_correction_mode(data, &param);
	break;
	case VL53LX_OPTICALCENTER_PAR:
		rc = ctrl_param_optical_center(data, &param);
	break;
	case VL53LX_TUNING_PAR:
		rc = ctrl_param_tuning(data, &param);
		break;
	case VL53LX_SMUDGECORRECTIONMODE_PAR:
		rc = ctrl_param_smudge_correction_mode(data, &param);
	break;
	case VL53LX_ISXTALKVALUECHANGED_PAR:
		rc = ctrl_param_is_xtalk_value_changed(data, &param);
	break;
	default:
		vl53lx_errmsg("unknown or unsupported %d\n", param.name);
		rc = -EINVAL;
	}
	/* copy back (status at least ) to user */
	if (param.is_read  && rc == 0) {
		rc2 = copy_to_user(p, &param, sizeof(param));
		if (rc2) {
			rc = -EFAULT; /* kill prev status if that fail */
			vl53lx_errmsg("copy to user fail %d\n", rc);
		}
	}
done:
	mutex_unlock(&data->work_mutex);
	return rc;
}

/**
 * implement set/get roi ioctl
 * @param data device
 * @param p user space ioctl arg ptr
 * @return 0 on success <0 errno code
 *	@li -EBUSY when trying to set roi while ranging
 *	@li -EFAULT if cpy to/fm user fail for requested number of roi
 */
static int ctrl_roi(struct stmvl53lx_data *data, void __user *p)
{
	int rc;
	struct stmvl53lx_ioctl_roi_t roi;

	mutex_lock(&data->work_mutex);
	
	if (data->is_device_remove) {
		rc = -ENODEV;
		goto done;
	}
	/* first copy all data */
	rc = copy_from_user(&roi, p, sizeof(roi));
	if (rc) {
		rc = -EFAULT;
		goto done;
	}

	if (roi.is_read) {
		memcpy(&roi.Roi, &data->roi_cfg, sizeof(roi.Roi));
		rc = copy_to_user(p, &roi, sizeof(roi));
		if (rc) {
			vl53lx_errmsg("fail to copy Roi to user %d", rc);
			rc = -EFAULT;
			goto done;
		}
	} else {
		if (data->enable_sensor) {
			rc = -EBUSY;
			vl53lx_errmsg("can't set roi while ranging\n");
			goto done;
		}
		memcpy(&data->roi_cfg, &roi.Roi, sizeof(data->roi_cfg));
		vl53lx_dbgmsg("ROI modified TopLeft(%d %d) BottomRight(%d %d)\n",
			data->roi_cfg.TopLeftX,
			data->roi_cfg.TopLeftY,
			data->roi_cfg.BotRightX,
			data->roi_cfg.BotRightY);
	}

done:
	mutex_unlock(&data->work_mutex);
	return rc;
}

static int ctrl_calibration_data(struct stmvl53lx_data *data, void __user *p)
{
	int rc = 0;
	struct stmvl53lx_ioctl_calibration_data_t calib;
	int data_offset = offsetof(struct stmvl53lx_ioctl_calibration_data_t,
					data);

	mutex_lock(&data->work_mutex);

	if (data->is_device_remove) {
		rc = -ENODEV;
		goto done;
	}
	rc = copy_from_user(&calib, p, data_offset);
	if (rc) {
		vl53lx_errmsg("fail to detect read or write %d", rc);
		rc = -EFAULT;
		goto done;
	}

	if (calib.is_read) {
		memset(&calib.data, 0, sizeof(calib.data));
		rc = VL53LX_GetCalibrationData(&data->stdev, &calib.data);
		if (rc) {
			vl53lx_errmsg("VL53LX_GetCalibrationData fail %d", rc);
			rc = store_last_error(data, rc);
			goto done;
		}
		rc = copy_to_user(p + data_offset, &calib.data,
			sizeof(calib.data));
	} else {
		if (data->enable_sensor) {
			rc = -EBUSY;
			vl53lx_errmsg("can't set calib data while ranging\n");
			goto done;
		}
		rc = copy_from_user(&calib.data, p + data_offset,
			sizeof(calib.data));
		if (rc) {
			vl53lx_errmsg("fail to copy calib data");
			rc = -EFAULT;
			goto done;
		}
		rc = VL53LX_SetCalibrationData(&data->stdev, &calib.data);
		if (rc) {
			vl53lx_errmsg("VL53LX_SetCalibrationData fail %d", rc);
			rc = store_last_error(data, rc);
		}
	}

done:
	mutex_unlock(&data->work_mutex);

	return rc;
}

static int ctrl_perform_calibration_ref_spad_lock(struct stmvl53lx_data *data,
	struct stmvl53lx_ioctl_perform_calibration_t *calib)
{
	int rc = 0;
	
	data->is_delay_allowed = 1;
	rc = VL53LX_PerformRefSpadManagement(&data->stdev);
	data->is_delay_allowed = 0;

	if (rc) {
		vl53lx_errmsg("VL53LX_PerformRefSpadManagement fail => %d", rc);
		rc = store_last_error(data, rc);
	}

	return rc;
}

static int ctrl_perform_calibration_crosstalk_lock(struct stmvl53lx_data *data,
	struct stmvl53lx_ioctl_perform_calibration_t *calib)
{
	int rc = 0;

	rc = stmvl53lx_sendparams(data);
	if (rc)
		goto done;

	rc = VL53LX_PerformXTalkCalibration(&data->stdev);
	if (rc) {
		vl53lx_errmsg("VL53LX_PerformXTalkCalibration fail => %d", rc);
		rc = store_last_error(data, rc);
	}
	/* fix ticket #823926 enable Xtalk correction after a calibration */
	data->crosstalk_enable = 1;
done:
		return rc;
}


static int ctrl_perform_simple_calibration_offset_lock(
	struct stmvl53lx_data *data,
	struct stmvl53lx_ioctl_perform_calibration_t *calib)
{
	int rc = 0;

	/* finally perform calibration */
	/* allow delay add after stop in VL53LX_run_<offset|zone>_calibration */
	data->is_delay_allowed = 1;

	rc = stmvl53lx_sendparams(data);
	if (rc)
		goto done;

	rc = VL53LX_PerformOffsetSimpleCalibration(&data->stdev, calib->param1);
	data->is_delay_allowed = 0;
	if (rc) {
		vl53lx_errmsg(
			"VL53LX_PerformOffsetSimpleCalibration fail => %d", rc);
		rc = store_last_error(data, rc);
	}

done:
	return rc;
}

static int ctrl_perform_per_vcsel_calibration_offset_lock(
	struct stmvl53lx_data *data,
	struct stmvl53lx_ioctl_perform_calibration_t *calib)
{
	int rc = 0;

	/* finally perform calibration */
	/* allow delay add after stop in VL53LX_run_<offset|zone>_calibration */
	data->is_delay_allowed = 1;

	rc = stmvl53lx_sendparams(data);
	if (rc)
		goto done;

	rc = VL53LX_PerformOffsetPerVcselCalibration(&data->stdev,
		calib->param1);
	data->is_delay_allowed = 0;
	if (rc) {
		vl53lx_errmsg(
		"VL53LX_PerformOffsetPerVcselCalibration fail => %d", rc);
		rc = store_last_error(data, rc);
	}

done:
	return rc;
}

static int ctrl_perform_zero_distance_calibration_offset_lock(
	struct stmvl53lx_data *data,
	struct stmvl53lx_ioctl_perform_calibration_t *calib)
{
	int rc = 0;

	/* finally perform calibration */
	/* allow delay add after stop in VL53LX_run_<offset|zone>_calibration */
	data->is_delay_allowed = 1;

	rc = stmvl53lx_sendparams(data);
	if (rc)
		goto done;

	rc = VL53LX_PerformOffsetZeroDistanceCalibration(&data->stdev);
	data->is_delay_allowed = 0;
	if (rc) {
		vl53lx_errmsg(
		"VL53LX_PerformOffsetZeroDistanceCalibration fail => %d", rc);
		rc = store_last_error(data, rc);
	}

done:
	return rc;
}

static int ctrl_perform_calibration(struct stmvl53lx_data *data, void __user *p)
{
	int rc = 0;
	struct stmvl53lx_ioctl_perform_calibration_t calib;

	mutex_lock(&data->work_mutex);

	if (data->is_device_remove) {
		rc = -ENODEV;
		goto done;
	}
	data->is_calibrating = true;
	rc = copy_from_user(&calib, p, sizeof(calib));
	if (rc) {
		rc = -EFAULT;
		goto done;
	}
	if (data->enable_sensor) {
		rc = -EBUSY;
		vl53lx_errmsg("can't perform calibration while ranging\n");
		goto done;
	}

	rc = reset_release(data);
	if (rc)
		goto done;

	switch (calib.calibration_type) {
	case VL53LX_CALIBRATION_REF_SPAD:
		rc = ctrl_perform_calibration_ref_spad_lock(data,
			&calib);
		break;
	case VL53LX_CALIBRATION_CROSSTALK:
		rc = ctrl_perform_calibration_crosstalk_lock(data,
			&calib);
		break;
	case VL53LX_CALIBRATION_OFFSET:
		rc = ctrl_perform_simple_calibration_offset_lock(data,
			&calib);
		break;
	case VL53LX_CALIBRATION_OFFSET_SIMPLE:
		rc = ctrl_perform_simple_calibration_offset_lock(data,
			&calib);
		break;
	case VL53LX_CALIBRATION_OFFSET_PER_VCSEL:
		rc = ctrl_perform_per_vcsel_calibration_offset_lock(data,
			&calib);
		break;
	case VL53LX_CALIBRATION_OFFSET_ZERO_DISTANCE:
		rc = ctrl_perform_zero_distance_calibration_offset_lock(data,
			&calib);
		break;
	default:
		rc = -EINVAL;
		break;
	}

	reset_hold(data);

done:
	data->is_calibrating = false;
	data->is_first_start_done = true;
	mutex_unlock(&data->work_mutex);

	return rc;
}

static int ctrl_unique_id_data(struct stmvl53lx_data *data, void __user *p)
{
	uint64_t UID_read_64;	
	int rc;

	mutex_lock(&data->work_mutex);
	if (data->is_device_remove) {
		rc = -ENODEV;
		goto done;
	}

	UID_read_64 = data->unique_id;

	rc = copy_to_user(p, &UID_read_64, sizeof(UID_read_64));
	if (rc) {
		vl53lx_dbgmsg("copy to user fail %d", rc);
		rc = -EFAULT;
		goto done;
	}

	done:
		mutex_unlock(&data->work_mutex);

	return rc;
}

static int stmvl53lx_ioctl_handler(
		struct stmvl53lx_data *data,
		unsigned int cmd, unsigned long arg,
		void __user *p)
{
	int rc = 0;

	if (!data)
		return -EINVAL;

	switch (cmd) {

	case VL53LX_IOCTL_START:
		vl53lx_dbgmsg("VL53LX_IOCTL_START\n");
		rc = ctrl_start(data);
		break;

	case VL53LX_IOCTL_STOP:
		vl53lx_dbgmsg("VL53LX_IOCTL_STOP\n");
		rc = ctrl_stop(data);
		break;

	/* Register tool */
	case VL53LX_IOCTL_REGISTER:
		vl53lx_dbgmsg("VL53LX_IOCTL_REGISTER\n");
		rc = ctrl_reg_access(data, p);
		break;

	case VL53LX_IOCTL_PARAMETER:
		vl53lx_dbgmsg("VL53LX_IOCTL_PARAMETER\n");
		rc = ctrl_params(data, p);
		break;

	case VL53LX_IOCTL_ROI:
		vl53lx_dbgmsg("VL53LX_IOCTL_ROI\n");
		rc = ctrl_roi(data, p);
		break;

	case VL53LX_IOCTL_MZ_DATA:
		/* vl53lx_dbgmsg("VL53LX_IOCTL_MZ_DATA\n"); */
		rc = ctrl_mz_data(data, p);
		break;
	case VL53LX_IOCTL_MZ_DATA_BLOCKING:
		/* vl53lx_dbgmsg("VL53LX_IOCTL_MZ_DATA_BLOCKING\n"); */
		rc = ctrl_mz_data_blocking(data, p);
		break;
	case VL53LX_IOCTL_CALIBRATION_DATA:
		vl53lx_dbgmsg("VL53LX_IOCTL_CALIBRATION_DATA\n");
		rc = ctrl_calibration_data(data, p);
		break;
	case VL53LX_IOCTL_PERFORM_CALIBRATION:
		vl53lx_dbgmsg("VL53LX_IOCTL_PERFORM_CALIBRATION\n");
		rc = ctrl_perform_calibration(data, p);
		break;
	case VL53LX_IOCTL_MZ_DATA_ADDITIONAL:
		/* vl53lx_dbgmsg("VL53LX_IOCTL_MZ_DATA_ADDITIONAL\n"); */
		rc = ctrl_mz_data_additional(data, p);
		break;
	case VL53LX_IOCTL_MZ_DATA_ADDITIONAL_BLOCKING:
		/* vl53lx_dbgmsg("VL53LX_IOCTL_MZ_DATA_ADDITIONAL_BLOCKING\n");
		 */
		rc = ctrl_mz_data_blocking_additional(data, p);
		break;
	case VL53LX_IOCTL_UID_DATA:
		vl53lx_dbgmsg("VL53L1_IOCTL_UID_DATA\n");
		rc = ctrl_unique_id_data(data, p);	
		break;
	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}



static int stmvl53lx_open(struct inode *inode, struct file *file)
{
	struct stmvl53lx_data *data = container_of(file->private_data,
		struct stmvl53lx_data, miscdev);

	vl53lx_dbgmsg("Start\n");
	stmvl53lx_module_func_tbl.get(data->client_object);
	vl53lx_dbgmsg("End\n");

	return 0;
}

static int stmvl53lx_release(struct inode *inode, struct file *file)
{
	struct stmvl53lx_data *data = container_of(file->private_data,
		struct stmvl53lx_data, miscdev);

	vl53lx_dbgmsg("Start\n");
	stmvl53lx_module_func_tbl.put(data->client_object);
	vl53lx_dbgmsg("End\n");

	return 0;
}


/** max number or error per measure too abort */
#define stvm531_get_max_meas_err(...) 3
/** max number or error per stream too abort */
#define stvm531_get_max_stream_err(...) 6

static void detect_xtalk_value_change(struct stmvl53lx_data *data,
	VL53LX_MultiRangingData_t *meas)
{
	data->is_xtalk_value_changed = meas->HasXtalkValueChanged ? true :
		data->is_xtalk_value_changed;
}

/**
 * handle data retrieval and dispatch
 *
 * work lock must be held
 *
 * called  form work or interrupt thread it must be a blocable context !
 * @param data the device
 */
static void stmvl53lx_on_newdata_event(struct stmvl53lx_data *data)
{
	int rc = 0;
	VL53LX_MultiRangingData_t *pmrange;
	VL53LX_MultiRangingData_t *tmprange;
	VL53LX_TargetRangeData_t RangeData[VL53LX_MAX_RANGE_RESULTS];
	long ts_msec;
	int i;
	struct input_dev *input = data->input_dev_ps;

	st_gettimeofday(&data->meas.comp_tv);
	ts_msec = stmvl53lx_tv_dif(&data->start_tv, &data->meas.comp_tv)/1000;

	pmrange = &data->meas.multi_range_data;
	tmprange = &data->meas.tmp_range_data;

	for (i = 0; i < VL53LX_MAX_RANGE_RESULTS; i++)
		memcpy(&RangeData[i], &pmrange->RangeData[i],
				sizeof(VL53LX_TargetRangeData_t));

	data->meas.intr++;

	rc = VL53LX_GetMultiRangingData(&data->stdev,
		&data->meas.tmp_range_data);

	/* be sure we got VL53LX_RANGESTATUS_NONE for object 0 if we got
	 * no object.
	 */
	if (tmprange->NumberOfObjectsFound == 0)
		tmprange->RangeData[0].RangeStatus =
					VL53LX_RANGESTATUS_NONE;

	memcpy(pmrange, tmprange, sizeof(VL53LX_MultiRangingData_t));

	/* got histogram debug data in case user want it later on */
	if (!rc)
		rc = VL53LX_GetAdditionalData(&data->stdev,
			&data->meas.additional_data);
	detect_xtalk_value_change(data, pmrange);

	/* check if not stopped yet
	 * as we may have been unlocked we must re-check
	 */
	if (data->enable_sensor == 0) {
		vl53lx_dbgmsg("at meas #%d we got stopped\n", data->meas.cnt);
		return;
	}
	if (rc) {
		vl53lx_errmsg("VL53LX_GetRangingMeasurementData @%d %d",
				__LINE__, rc);
		data->meas.err_cnt++;
		data->meas.err_tot++;
		if (data->meas.err_cnt > stvm531_get_max_meas_err(data) ||
			data->meas.err_tot > stvm531_get_max_stream_err(data)) {
			vl53lx_errmsg("on #%d %d err %d tot stop",
				data->meas.cnt, data->meas.err_cnt,
				data->meas.err_tot);
			_ctrl_stop(data);
			input_report_abs(input, ABS_MISC, ABNORMAL_STOP_3);
			input_sync(input);
		}
		return;
	}

	/* FIXME: remove when implemented by ll or bare driver */
	pmrange->TimeStamp = ts_msec;
	for (i = 1; i < pmrange->NumberOfObjectsFound; i++)
		pmrange->TimeStamp = ts_msec;

	data->meas.cnt++;
	vl53lx_dbgmsg("#%3d %2d poll ts %5d status=%d obj cnt=%d\n",
		data->meas.cnt,
		data->meas.poll_cnt,
		pmrange->TimeStamp,
		pmrange->RangeData[0].RangeStatus,
		pmrange->NumberOfObjectsFound);
#if 0
	vl53lx_dbgmsg(
"meas m#%04d i#%04d  p#%04d in %d ms data range status %d range %d\n",
			(int)data->meas.cnt,
			(int)data->meas.intr,
			(int)data->meas.poll_cnt,
			(int)stmvl53lx_tv_dif(&data->meas.start_tv,
					&data->meas.comp_tv)/1000,
			(int)data->meas.range_data.RangeStatus,
			(int)data->meas.range_data.RangeMilliMeter);
#endif
	/* ready that is not always on each new data event */

	/* mark data as valid from now */
	data->is_data_valid = true;

	/* wake up sleeping client */
	wake_up_data_waiters(data);

	/* push data to input subsys and only and make val for ioctl*/
	stmvl53lx_input_push_data(data);
	stmvl53lx_insert_flush_events_lock(data);

	/* roll time now data got used */
	data->meas.start_tv = data->meas.comp_tv;
	data->meas.poll_cnt = 0;
	data->meas.err_cnt = 0;
}


/**
 * * handle interrupt/pusdo irq by polling handling
 *
 * work lock must be held
 *
 * @param data driver
 * @return 0 on success
 */
static int stmvl53lx_intr_process(struct stmvl53lx_data *data)
{
	uint8_t data_rdy;
	int rc = 0;
	struct st_timeval tv_now;
	struct input_dev *input = data->input_dev_ps;

	if (!data->enable_sensor)
		goto done;

	data->meas.poll_cnt++;
	rc = VL53LX_GetMeasurementDataReady(&data->stdev, &data_rdy);
	if (rc) {
		vl53lx_errmsg("GetMeasurementDataReady @%d %d, fail\n",
				__LINE__, rc);
		/* too many successive fail => stop but do not try to do any new
		 * i/o
		 */
		goto stop_io;
	}

	if (!data_rdy) {
		/* FIXME this part to completely skip
		 * if using interrupt and sure we have
		 * no false interrupt to handle or no to do any timing check
		 */
		long poll_us;

		st_gettimeofday(&tv_now);
		poll_us = stmvl53lx_tv_dif(&data->meas.start_tv, &tv_now);
		if (poll_us > data->timing_budget*4) {
			vl53lx_errmsg("we're polling %ld ms too long\n",
					poll_us/1000);
			/*  fixme stop or just warn ? */
			goto stop_io;
		}
		/*  keep trying it could be intr with no processing */
		work_dbg("intr with no data rdy");
		goto done;
	}
	/* we have data to handle */
	/* first irq after reset has no data so we skip it */
	if (data->is_first_irq)
		data->is_first_irq = false;
	else
		stmvl53lx_on_newdata_event(data);
	/* enable_sensor could change on event handling check again */
	if (data->enable_sensor) {
		/* clear interrupt and continue ranging */
		work_dbg("intr clr");
		/* In autonomous mode, bare driver will trigger stop/start
		 * sequence. In that case it wall call platform delay functions.
		 * So allow delay in VL53LX_ClearInterruptAndStartMeasurement()
		 * call.
		 */
		data->is_delay_allowed = data->allow_hidden_start_stop;
		rc = VL53LX_ClearInterruptAndStartMeasurement(&data->stdev);
		data->is_delay_allowed = 0;
		if (rc) {
			/* go to stop but stop any new i/o for dbg */
			vl53lx_errmsg("Cltr intr restart fail %d\n", rc);
			goto stop_io;
		}
	}
done:
	return rc;
stop_io:
	/* too many successive fail take action => stop but do not try to do
	 * any new i/o
	 */
	vl53lx_errmsg("GetDatardy fail stop\n");
	_ctrl_stop(data);
	input_report_abs(input, ABS_MISC, ABNORMAL_STOP_1);
	input_sync(input);
	return rc;

}

static void stmvl53lx_work_handler(struct work_struct *work)
{
	struct stmvl53lx_data *data;

	data = container_of(work, struct stmvl53lx_data, dwork.work);
	work_dbg("enter");
	mutex_lock(&data->work_mutex);
	stmvl53lx_intr_process(data);
	if (data->poll_mode && data->enable_sensor) {
		/* re-sched ourself */
		schedule_delayed_work(&data->dwork,
			msecs_to_jiffies(data->poll_delay_ms));
	}
	mutex_unlock(&data->work_mutex);
}

static void stmvl53lx_input_push_data_multiobject(struct stmvl53lx_data *data)
{
	VL53LX_MultiRangingData_t *mmeas = &data->meas.multi_range_data;
	int i;
	int rc = 0;
	VL53LX_TargetRangeData_t *meas_array[4];
	VL53LX_CalibrationData_t calibration_data;
	struct st_timeval tv;
	struct input_dev *input = data->input_dev_ps;

	st_gettimeofday(&tv);

	for (i = 0; i < 4; i++)
		meas_array[i] = &mmeas->RangeData[i];

	/*************************************************************
	 *  INPUT EVENT CODE	L1/L3 Data
	   ABS_HAT0X	Time in Sec(32)
	   ABS_HAT0Y	Time in uSec(32)
	   ABS_HAT1X	Obj0_Distance(16) :  Obj0_Sigma(16)
	   ABS_HAT1Y	Obj0_MinRange(16) : Obj0_MaxRange(16)
	   ABS_HAT2X	Obj1_Distance(16) :  Obj1_Sigma(16)
	   ABS_HAT2Y	Obj1_ MinRange (16) : Obj1_ MaxRange (16)
	   ABS_HAT3X	Obj0_SignalRate_Spad(32)
	   ABS_HAT3Y	Obj1_SignalRate_Spad(32)
	   ABS_WHEEL	AmbientRate(32)
	   ABS_BRAKE	EffectiveSpadRtnCount(16):RangeStatus_1(8):
					Range_status_0(8)
	   ABS_TILT_X	XtalkChange(8) :StreamCount(8) :
					NumberofObjects(2)
	   ABS_TILT_Y	N/A
	   ABS_TOOL_WIDTH	XtalkValue
	   ABS_DISTANCE
	   ABS_THROTTLE
	   ABS_RUDDER
	   ABS_MISC
	   ABS_VOLUME
	 ************************************************************/

	rc = VL53LX_GetCalibrationData(&data->stdev, &calibration_data);
	if (rc) {
		/* This should not happen */
		vl53lx_errmsg("%d error:%d\n", __LINE__, rc);
		return;
	}

	/* ABS_HAT0X  -	Time in Sec(32) */

	input_report_abs(input, ABS_HAT0X, tv.tv_sec);
	vl53lx_dbgmsg("ABS_HAT0X : %lld, %zu\n", tv.tv_sec, sizeof(tv.tv_sec));
	/* ABS_HAT0Y   - Time in uSec(32) */
	/* REVISIT : The following code may cause loss of data due to */
	/* 8 bytes to 32 bits conversion */
	input_report_abs(input, ABS_HAT0Y, tv.tv_usec);
	vl53lx_dbgmsg("ABS_HAT0Y : %ld\n", tv.tv_usec);

	/* ABS_WHEEL - AmbientRate(32) */
	input_report_abs(input, ABS_WHEEL,
		meas_array[0]->AmbientRateRtnMegaCps);
	vl53lx_dbgmsg("ABS_WHEEL : AmbRate = %d\n",
		meas_array[0]->AmbientRateRtnMegaCps);

	/* ABS_TILT_X	XtalkChange(8) :StreamCount(8) : */
	/* Number of Objects(2) */
	input_report_abs(input, ABS_TILT_X,
			(mmeas->HasXtalkValueChanged << 16)
			|  (mmeas->StreamCount << 8)
			| ((mmeas->NumberOfObjectsFound & 0x3) << 6));
	vl53lx_dbgmsg("ABS_TILT_X :(%d):(%d):(%d)\n\n",
					mmeas->HasXtalkValueChanged,
					mmeas->StreamCount,
					mmeas->NumberOfObjectsFound
					);


	/* ABS_TILT_Y	DMAX */
	input_report_abs(input, ABS_TILT_Y,   0);

	/* ABS_TOOL_WIDTH */
	input_report_abs(input, ABS_TOOL_WIDTH,
calibration_data.customer.algo__crosstalk_compensation_plane_offset_kcps);
		vl53lx_dbgmsg("ABS_TOOL_WIDTH Xtalk = %d\n",
calibration_data.customer.algo__crosstalk_compensation_plane_offset_kcps);

	/* ABS_BRAKE  -	EffectiveSpadRtnCount(16):RangeStatus_3(1): */
	/* Range_status_2(0) */
	input_report_abs(input, ABS_BRAKE,
			 mmeas->EffectiveSpadRtnCount << 16
			| ((meas_array[1]->RangeStatus) << 8)
			| meas_array[0]->RangeStatus);

	vl53lx_dbgmsg("ABS_BRAKE : (%d):(%d):(%d)\n",
			mmeas->EffectiveSpadRtnCount,
			meas_array[1]->RangeStatus,
			meas_array[0]->RangeStatus);

	vl53lx_dbgmsg("ABS_BRAKE : 0x%X\n",
			(mmeas->EffectiveSpadRtnCount & 0xFFFF) << 16
			| ((meas_array[1]->RangeStatus) << 8)
			| meas_array[0]->RangeStatus);

	/* Remaining of data are meaningless in case of no target */
	if (mmeas->NumberOfObjectsFound == 0) {
		input_sync(input);
		return;
	}

	/* ABS_HAT1X   -	 Obj0_Distance(16) :  Obj0_Sigma(16) */
	input_report_abs(input, ABS_HAT1X, meas_array[0]->RangeMilliMeter << 16
			| (meas_array[0]->SigmaMilliMeter/65536));
	vl53lx_dbgmsg("ABS_HAT1X : 0x%X(%d:%d)\n",
			meas_array[0]->RangeMilliMeter << 16
			| (meas_array[0]->SigmaMilliMeter/65536),
			meas_array[0]->RangeMilliMeter,
			(meas_array[0]->SigmaMilliMeter/65536));

	/* ABS_HAT1Y   -	Obj0_MinRange(16) : Obj0_MaxRange(16) */
	input_report_abs(input, ABS_HAT1Y,
			meas_array[0]->RangeMinMilliMeter << 16
			| meas_array[0]->RangeMaxMilliMeter);

	vl53lx_dbgmsg("ABS_HAT1Y : 0x%X(%d:%d)\n",
			meas_array[0]->RangeMinMilliMeter << 16
			| meas_array[0]->RangeMaxMilliMeter,
			meas_array[0]->RangeMinMilliMeter,
			meas_array[0]->RangeMaxMilliMeter);

	if (mmeas->NumberOfObjectsFound > 1) {
		/* ABS_HAT2X   -	Obj1_Distance(16) :  Obj1_Sigma(16) */
		input_report_abs(input, ABS_HAT2X,
				meas_array[1]->RangeMilliMeter << 16
				| (meas_array[1]->SigmaMilliMeter/65536));
		vl53lx_dbgmsg("ABS_HAT2X : 0x%x(%d:%d)\n",
				meas_array[1]->RangeMilliMeter << 16
				| (meas_array[1]->SigmaMilliMeter/65536),
				meas_array[1]->RangeMilliMeter,
				(meas_array[1]->SigmaMilliMeter/65536));

		/* ABS_HAT2Y   -	Obj1_ MinRange (16) : Obj1_ MaxRange (16) */
		input_report_abs(input, ABS_HAT2Y,
				meas_array[1]->RangeMinMilliMeter << 16
				| meas_array[1]->RangeMaxMilliMeter);

		vl53lx_dbgmsg("ABS_HAT2Y : 0x%X(%d:%d)\n",
				meas_array[1]->RangeMinMilliMeter << 16
				| meas_array[1]->RangeMaxMilliMeter,
				meas_array[1]->RangeMinMilliMeter,
				meas_array[1]->RangeMaxMilliMeter);

	}

	/* ABS_HAT3X	  -	Obj0_SignalRate_Spad(32) */
	input_report_abs(input, ABS_HAT3X,
			meas_array[0]->SignalRateRtnMegaCps);
	vl53lx_dbgmsg("ABS_HAT3X : SignalRateRtnMegaCps_0(%d)\n",
			meas_array[0]->SignalRateRtnMegaCps);
	if (mmeas->NumberOfObjectsFound > 1) {
		/* ABS_HAT3Y  -		Obj1_SignalRate_Spad(32) */
		input_report_abs(input, ABS_HAT3Y,
			meas_array[1]->SignalRateRtnMegaCps);
		vl53lx_dbgmsg("ABS_HAT3Y : SignalRateRtnMegaCps_1(%d)\n",
			meas_array[1]->SignalRateRtnMegaCps);
	}

	input_report_abs(input, ABS_MISC, 0);
	input_sync(input);


}

static void stmvl53lx_input_push_data(struct stmvl53lx_data *data)
{
	stmvl53lx_input_push_data_multiobject(data);
}

static int stmvl53lx_input_setup(struct stmvl53lx_data *data)
{
	int rc = 0;
	struct input_dev *idev;
	/* Register to Input Device */
	idev = input_allocate_device();
	if (idev == NULL) {
		rc = -ENOMEM;
		vl53lx_errmsg("%d error:%d\n", __LINE__, rc);
		goto exit_err;
	}
	/*  setup all event */
	set_bit(EV_ABS, idev->evbit);

	input_set_abs_params(idev, ABS_DISTANCE, 0, 0xff, 0, 0);

	input_set_abs_params(idev, ABS_HAT0X, 0, 0xffffffff, 0, 0);
	input_set_abs_params(idev, ABS_HAT0Y, 0, 0xffffffff, 0, 0);
	input_set_abs_params(idev, ABS_HAT1X, 0, 0xffffffff, 0, 0);
	input_set_abs_params(idev, ABS_HAT1Y, 0, 0xffffffff, 0, 0);
	input_set_abs_params(idev, ABS_HAT2X, 0, 0xffffffff, 0, 0);
	input_set_abs_params(idev, ABS_HAT2Y, 0, 0xffffffff, 0, 0);
	input_set_abs_params(idev, ABS_HAT3X, 0, 0xffffffff, 0, 0);
	input_set_abs_params(idev, ABS_HAT3Y, 0, 0xffffffff, 0, 0);
	input_set_abs_params(idev, ABS_WHEEL, 0, 0xffffffff, 0, 0);

	input_set_abs_params(idev, ABS_TILT_Y, 0, 0xffffffff, 0, 0);

	input_set_abs_params(idev, ABS_BRAKE, 0, 0xffffffff, 0, 0);
	input_set_abs_params(idev, ABS_TILT_X, 0, 0xffffffff, 0, 0);
	input_set_abs_params(idev, ABS_TOOL_WIDTH, 0, 0xffffffff, 0, 0);

	input_set_abs_params(idev, ABS_THROTTLE, 0, 0xffffffff, 0, 0);
	input_set_abs_params(idev, ABS_RUDDER, 0, 0xffffffff, 0, 0);
	input_set_abs_params(idev, ABS_MISC, 0, 0xffffffff, 0, 0);

	input_set_abs_params(idev, ABS_VOLUME, 0, 0xffffffff, 0, 0);
	input_set_abs_params(idev, ABS_GAS, 0, 0xffffffff, 0, 0);

	idev->name = "STM VL53LX proximity sensor";
	rc = input_register_device(idev);
	if (rc) {
		rc = -ENOMEM;
		vl53lx_errmsg("%d error:%d\n", __LINE__, rc);
		goto exit_free_dev_ps;
	}
	/* setup drv data */
	input_set_drvdata(idev, data);
	data->input_dev_ps = idev;
	return 0;


exit_free_dev_ps:
	input_free_device(data->input_dev_ps);
exit_err:
	return rc;
}

/**
 * handler to be called by interface module on interrupt
 *
 * managed poll/irq filtering in case poll/irq can be soft forced
 * and the module side still fire interrupt
 *
 * @param data
 * @return 0 if all ok else for error
 */
int stmvl53lx_intr_handler(struct stmvl53lx_data *data)
{
	int rc = 0;

	/* fix ticket 1458514 deadlock multiple devices, defer the data collection */
	if ((data->enable_sensor) && (!data->poll_mode)) {
		rc = schedule_delayed_work(&data->dwork, 0);
	}

	return rc;
}


/**
 * One time device  setup
 *
 * call by bus (i2c/cci) level probe to finalize non bus related device setup
 *
 * @param	data The device data
 * @return	0 on success
 */
int stmvl53lx_setup(struct stmvl53lx_data *data)
{
	int rc = 0;
	VL53LX_DeviceInfo_t dev_info;
	uint64_t dev_uid;

	vl53lx_dbgmsg("Enter\n");

	/* acquire an id */
	data->id = allocate_dev_id();
	if (data->id < 0) {
		vl53lx_errmsg("too many device already created");
		return -1;
	}
	vl53lx_dbgmsg("Dev id %d is @%p\n", data->id, data);
	stmvl53lx_dev_table[data->id] = data;

	/* init mutex */
	/* mutex_init(&data->update_lock); */
	mutex_init(&data->work_mutex);

	/* init work handler */
	INIT_DELAYED_WORK(&data->dwork, stmvl53lx_work_handler);
	data->force_device_on_en = force_device_on_en_default;
	data->reset_state = 1;
	data->is_calibrating = false;
	data->last_error = VL53LX_ERROR_NONE;
	data->is_device_remove = false;

	rc = stmvl53lx_module_func_tbl.power_up(data->client_object);
	if (rc) {
		vl53lx_errmsg("%d,error rc %d\n", __LINE__, rc);
		goto exit_func_end;
	}
	rc = reset_release(data);
	if (rc)
		goto exit_func_end;

	rc = stmvl53lx_input_setup(data);
	if (rc)
		goto exit_func_end;

	/* init blocking ioctl stuff */
	INIT_LIST_HEAD(&data->simple_data_reader_list);
	INIT_LIST_HEAD(&data->mz_data_reader_list);
	init_waitqueue_head(&data->waiter_for_data);
	data->is_data_valid = false;

	/* Register sysfs hooks under input dev */
	rc = sysfs_create_group(&data->input_dev_ps->dev.kobj,
			&stmvl53lx_attr_group);
	if (rc) {
		rc = -ENOMEM;
		vl53lx_errmsg("%d error:%d\n", __LINE__, rc);
		goto exit_unregister_dev_ps;
	}
	rc = sysfs_create_bin_file(&data->input_dev_ps->dev.kobj,
		&stmvl53lx_calib_data_attr);
	if (rc) {
		rc = -ENOMEM;
		vl53lx_errmsg("%d error:%d\n", __LINE__, rc);
		goto exit_unregister_dev_ps;
	}

	data->enable_sensor = 0;

	data->poll_delay_ms = STMVL53LX_CFG_POLL_DELAY_MS;
	data->timing_budget = STMVL53LX_CFG_TIMING_BUDGET_US;
	data->distance_mode = STMVL53LX_CFG_DEFAULT_DISTANCE_MODE;
	data->crosstalk_enable = STMVL53LX_CFG_DEFAULT_CROSSTALK_ENABLE;
	data->offset_correction_mode =
		STMVL53LX_CFG_DEFAULT_OFFSET_CORRECTION_MODE;
	data->smudge_correction_mode =
		STMVL53LX_CFG_DEFAULT_SMUDGE_CORRECTION_MODE;
	data->is_xtalk_value_changed = false;

	data->is_delay_allowed = true;
	/* need to be done once */
	rc = VL53LX_DataInit(&data->stdev);
	data->is_delay_allowed = false;
	if (rc) {
		vl53lx_errmsg("VL53LX_DataInit %d\n", rc);
		goto exit_unregister_dev_ps;
	}

	rc = VL53LX_GetUserROI(&data->stdev, &(data->roi_cfg));
	if (rc) {
		vl53lx_errmsg("VL53LX_GetUserROI %d\n", rc);
		goto exit_unregister_dev_ps;
	}

	rc = VL53LX_GetDeviceInfo(&data->stdev, &dev_info);
	if (rc) {
		vl53lx_errmsg("VL53LX_GetDeviceInfo %d\n", rc);
		goto exit_unregister_dev_ps;
	}
	vl53lx_errmsg("device type %x\n", dev_info.ProductType);

	data->is_delay_allowed = true;
	rc = VL53LX_GetUID(&data->stdev, &dev_uid);
	data->is_delay_allowed = false;
	if (rc) {
		vl53lx_errmsg("VL53LX_GetUID %d\n", rc);
		goto exit_unregister_dev_ps;
	}
	vl53lx_errmsg("device UID 0x%llx\n", dev_uid);
			
	data->unique_id = dev_uid;

	rc = VL53LX_GetOpticalCenter(&data->stdev, &data->optical_offset_x,
		&data->optical_offset_y);
	if (rc) {
		vl53lx_errmsg("VL53LX_GetOpticalCenter %d\n", rc);
		goto exit_unregister_dev_ps;
	}

	/* set tuning from stmvl53lx_tunings.h */
	rc = setup_tunings(data);
	if (rc) {
		vl53lx_errmsg("setup_tunings %d\n", rc);
		goto exit_unregister_dev_ps;
	}

	/* if working in interrupt ask intr to s and hook the handler */
	data->poll_mode = 0;
	rc = stmvl53lx_module_func_tbl.start_intr(data->client_object,
		&data->poll_mode);
	if (rc < 0) {
		vl53lx_errmsg("can't start no  intr\n");
		goto exit_unregister_dev_ps;
	}

	data->is_first_irq = true;
	data->is_first_start_done = false;
	data->is_delay_allowed = false;

	/* to register as a misc device */
	data->miscdev.minor = MISC_DYNAMIC_MINOR;
	/* multiple dev name use id in name but 1st */
	if (data->id == 0)
		strcpy(data->name, VL53LX_MISC_DEV_NAME);
	else
		sprintf(data->name, "%s%d", VL53LX_MISC_DEV_NAME, data->id);

	data->miscdev.name = data->name;
	data->miscdev.fops = &stmvl53lx_ranging_fops;
	vl53lx_errmsg("Misc device registration name:%s\n", data->miscdev.name);
	rc = misc_register(&data->miscdev);
	if (rc != 0) {
		vl53lx_errmsg("misc dev reg fail\n");
		goto exit_unregister_dev_ps;
	}
	/* bring back device under reset */
	reset_hold(data);

	return 0;

exit_unregister_dev_ps:
	sysfs_remove_bin_file(&data->input_dev_ps->dev.kobj,
		&stmvl53lx_calib_data_attr);
	sysfs_remove_group(&data->input_dev_ps->dev.kobj,
		&stmvl53lx_attr_group);
	input_unregister_device(data->input_dev_ps);

exit_func_end:
	return rc;
}


void stmvl53lx_cleanup(struct stmvl53lx_data *data)
{
	int rc = 0;

	vl53lx_dbgmsg("enter\n");
	rc = _ctrl_stop(data);
	if (rc < 0)
		vl53lx_errmsg("stop failed %d aborting anyway\n", rc);

	if (data->input_dev_ps) {
		vl53lx_dbgmsg("to remove sysfs group\n");
		sysfs_remove_group(&data->input_dev_ps->dev.kobj,
				&stmvl53lx_attr_group);
		sysfs_remove_bin_file(&data->input_dev_ps->dev.kobj,
				&stmvl53lx_calib_data_attr);

		vl53lx_dbgmsg("to unregister input dev\n");
		input_unregister_device(data->input_dev_ps);
	}

	if (!IS_ERR(data->miscdev.this_device) &&
			data->miscdev.this_device != NULL) {
		vl53lx_dbgmsg("to unregister misc dev\n");
		misc_deregister(&data->miscdev);
	}
	/* be sure device is put under reset */
	data->force_device_on_en = false;
	reset_hold(data);
	stmvl53lx_module_func_tbl.power_down(data->client_object);
	vl53lx_dbgmsg("done\n");
	deallocate_dev_id(data->id);
	data->is_device_remove = true;
}

#ifdef CONFIG_PM_SLEEP
void stmvl53lx_pm_suspend_stop(struct stmvl53lx_data *data)
{
	int rc = 0;

	vl53lx_dbgmsg("Enter\n");

	rc = _ctrl_stop(data);
	if (rc < 0)
		vl53lx_errmsg("stop failed %d aborting anyway\n", rc);

	vl53lx_dbgmsg("done\n");
}
#endif

static long stmvl53lx_ioctl(struct file *file,
		unsigned int cmd, unsigned long arg)
{
	long ret;
	struct stmvl53lx_data *data =
			container_of(file->private_data,
				struct stmvl53lx_data, miscdev);
	ret = stmvl53lx_ioctl_handler(data, cmd, arg, (void __user *)arg);
	return ret;
}

static int __init stmvl53lx_init(void)
{
	int rc = -1;

	vl53lx_dbgmsg("Enter\n");

	/* i2c/cci client specific init function */
	rc = stmvl53lx_module_func_tbl.init();

	vl53lx_dbgmsg("End %d\n", rc);

	return rc;
}

static void __exit stmvl53lx_exit(void)
{
	vl53lx_dbgmsg("Enter\n");
	stmvl53lx_module_func_tbl.deinit(NULL);
	if (stmvl53lx_module_func_tbl.clean_up != NULL)
		stmvl53lx_module_func_tbl.clean_up();
	vl53lx_dbgmsg("End\n");
}


/* MODULE_DEVICE_TABLE(i2c, stmvl53lx_id); */
MODULE_AUTHOR("STMicroelectronics Imaging Division");
MODULE_DESCRIPTION("ST FlightSense Time-of-Flight sensor driver");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION(DRIVER_VERSION);

module_init(stmvl53lx_init);
module_exit(stmvl53lx_exit);
