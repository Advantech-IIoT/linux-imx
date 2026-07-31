
/* SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause */
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53LX and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */

/**
 *  @file stmvl53lx_if.h  vl53lx kernel driver user interface
 *
 * @note to use this header in a user space application it requires
 *  all st bare/ll driver platform wrapper files (for data struct def)
 *  this files (types etc ..) shall be same or compliant with bar driver version
 *  used in the kernel module
 */

#ifndef STMVL53LX_IF_H
#define STMVL53LX_IF_H


#include "vl53lx_def.h"
/**
 * @addtogroup vl53lx_ioctl
 * @{
 */

/**
 * misc device name for ioctl device
 *
 * for mutli instance all device 2nd and next instance are basic name +"1"+"2"
 * @li stmvl53lx_ranging
 * @li stmvl53lx_ranging1
 * @li stmvl53lx_ranging2
 */
#define VL53LX_MISC_DEV_NAME		"stmvl53lx_ranging"

/**
 * parameter name in @ref stmvl53lx_parameter when using
 * @ref VL53LX_IOCTL_PARAMETER
 */
enum __stmv53lx_parameter_name_e {
	VL53LX_XTALKENABLE_PAR = 2,
	/*!< VL53LX_XTALKENABLE_PAR enable/disable crosstalk compensation\n
	 * valid value :
	 * @li 0 disable crosstalk compensation
	 * @li 1 enable crosstalk compensation
	 *
	 * @warning mode can only be set while not ranging
	 */

	VL53LX_POLLDELAY_PAR = 10,
	/*!< set the polling delay (msec)\n
	 *
	 * @note apply only when operates in polling mode  as no effect
	 * otherwise
	 */
	VL53LX_TIMINGBUDGET_PAR = 11,
	/*!< VL53LX_TIMINGBUDGET_PAR
	 * @ref stmvl53lx_parameter.value field is timing budget in micro second
	 *
	 * @note Please refer to VL53LX user manual for minimum timing budget
	 */

	VL53LX_DISTANCEMODE_PAR = 12,
	/*!< VL53LX_DISTANCEMODE_PAR
	 * valid distance mode value :
	 * @li 1 @a VL53LX_DISTANCEMODE_SHORT
	 * @li 2 @a VL53LX_DISTANCEMODE_MEDIUM
	 * @li 3 @a VL53LX_DISTANCEMODE_LONG
	 *
	 * @warning distance mode can only be set while not ranging
	 */

	VL53LX_FORCEDEVICEONEN_PAR = 14,
	/*!< VL53LX_FORCEDEVICEONEN_PAR
	 * This parameter will control if device is put under reset when
	 * stopped.
	 * valid force device on value :
	 * @li 0 feature is disable. Device is put under reset when stopped.
	 * @li 1 feature is enable. Device is not put under reset when stopped.
	 */

	VL53LX_LASTERROR_PAR = 15,
	/*!< VL53LX_LASTERROR_PAR
	 * This is a read only parameter. It will return last device internal
	 * error. It's valid only after an ioctl/sysfs return an -EIO error.
	 */

	VL53LX_OFFSETCORRECTIONMODE_PAR = 16,
	/*!< VL53LX_OFFSETCORRECTIONMODE_PAR
	 * This parameter will define which mode to use for the offset
	 * correction.
	 * valid force device on value :
	 * @li 1 @a VL53LX_OFFSETCORRECTIONMODE_STANDARD
	 * @li 2 @a VL53LX_OFFSETCORRECTIONMODE_PERZONE
	 *
	 * @warning offset correction mode can only be set while not ranging
	 */

	VL53LX_OPTICALCENTER_PAR = 17,
	/*!< VL53LX_OPTICALCENTER_PAR
	 * This is a read only parameter. It will return optical center issued
	 * from the nvm set at FMT stage. value will contain X position of
	 * center. value2 will contain Y position of center.
	 * Return values have FixPoint1616_t type.
	 */

	VL53LX_TUNING_PAR = 20,
	/*!< VL53LX_TUNING_PAR
	 * This parameter is a write only parameter. It will allow to provide
	 * low level layer with a configuration parameter.
	 * value will be use as a key parameter.
	 * value2 will be use as value parameter.
	 *
	 * @warning those configuration parameter settings are only allowed
	 * before device is start once.
	 */

	VL53LX_SMUDGECORRECTIONMODE_PAR = 21,
	/*!< VL53LX_SMUDGECORRECTIONMODE_PAR
	 * This parameter will control if smudge correction is enable and how
	 * crosstalk values are updated.
	 * @li 0 @a VL53LX_SMUDGE_CORRECTION_NONE
	 * @li 1 @a VL53LX_SMUDGE_CORRECTION_CONTINUOUS
	 * @li 2 @a VL53LX_SMUDGE_CORRECTION_SINGLE
	 * @li 3 @a VL53LX_SMUDGE_CORRECTION_DEBUG
	 */

	VL53LX_ISXTALKVALUECHANGED_PAR = 22,
	/*!< VL53LX_ISXTALKCHANGED_PAR
	 * This is a read only parameter. It will return if Xtalk value has
	 * been updated while ranging. This parameter is reset each time device
	 * start to range.
	 * @li 0 Xtalk values has not been changed.
	 * @li 1 Xtalk values has been changed.
	 */
};
#define stmv53lx_parameter_name_e enum __stmv53lx_parameter_name_e

/**
 * parameter structure use in @ref VL53LX_IOCTL_PARAMETER
 */
struct stmvl53lx_parameter {
	uint32_t is_read;	/*!< [in] 1: Get 0: Set*/
	/*!< [in] parameter to set/get
	 * see @ref stmv53lx_parameter_name_e
	 */
	stmv53lx_parameter_name_e name;
	int32_t value;		/*!< [in/out] value to set /get */
	int32_t value2;		/*!< [in/out] optional 2nd value */
	int32_t status;		/*!< [out] status of the operation */
};

/**
 * roi structure use in @ref VL53LX_IOCTL_ROI
 */
struct stmvl53lx_ioctl_roi_t {
	int32_t	is_read; /*!< [in] 1: Get 0: Set */
	VL53LX_UserRoi_t Roi; /*!<  [in/out] roi data */
};

/**
 * parameter structure use in @ref VL53LX_IOCTL_CALIBRATION_DATA
 */
struct stmvl53lx_ioctl_calibration_data_t {
	int32_t is_read;	/*!< [in] 1: Get 0: Set*/
	VL53LX_CalibrationData_t data;
	/*!< [in/out] data to set /get. Caller
	 * should consider this structure as an opaque one
	 */
};

/** Select reference spad calibration in @ref VL53LX_IOCTL_PERFORM_CALIBRATION.
 *
 * param1, param2 and param3 not use
 */
#define VL53LX_CALIBRATION_REF_SPAD		0

/** Select crosstalk calibration in @ref VL53LX_IOCTL_PERFORM_CALIBRATION.
 *
 * param1, param2 and param3 not use
 */
#define VL53LX_CALIBRATION_CROSSTALK		1

/** Select offset calibration @ref VL53LX_IOCTL_PERFORM_CALIBRATION.
 * param1 is target distance in mm.
 * param2 and param3 are not used
 *
 * Note that VL53LX_CALIBRATION_OFFSET usage is deprecated.
  * Applications should use VL53LX_CALIBRATION_OFFSET_SIMPLE
 * instead.
 */
#define VL53LX_CALIBRATION_OFFSET		2


/* entry 3 is intentionnaly skipped */

/** Select simple offset calibration @ref VL53LX_IOCTL_PERFORM_CALIBRATION.
 * param1 is target distance in mm.
 * param2 and param3 are not used
 * this calibration is used by the VL53LX_OFFSETCORRECTIONMODE_STANDARD mode
 */
#define VL53LX_CALIBRATION_OFFSET_SIMPLE	4

/** Select per Vcsel offset calibration @ref VL53LX_IOCTL_PERFORM_CALIBRATION.
 * param1 is target distance in mm.
 * param2 and param3 are not used
 * this calibration is used by the VL53LX_OFFSETCORRECTIONMODE_PERVCSEL mode
 */
#define VL53LX_CALIBRATION_OFFSET_PER_VCSEL	5

/** Select no Distance offset calibration @ref VL53LX_IOCTL_PERFORM_CALIBRATION.
 * param1, param2 and param3 are not used
 * this calibration is used by the VL53LX_OFFSETCORRECTIONMODE_STANDARD mode
 */
#define VL53LX_CALIBRATION_OFFSET_ZERO_DISTANCE	6

/**
 * parameter structure use in @ref VL53LX_IOCTL_PERFORM_CALIBRATION
 */
struct stmvl53lx_ioctl_perform_calibration_t {
	uint32_t calibration_type;
	/*!< [in] select which calibration to do :
	 * @li @ref VL53LX_CALIBRATION_REF_SPAD
	 * @li @ref VL53LX_CALIBRATION_CROSSTALK
	 * @li @ref VL53LX_CALIBRATION_OFFSET_SIMPLE
	 * @li @ref VL53LX_CALIBRATION_OFFSET_PER_VCSEL
	 * @li @ref VL53LX_CALIBRATION_OFFSET_ZERO_DISTANCE
	 */
	uint32_t param1;
	/*!< [in] first param. Usage depends on calibration_type */
	uint32_t param2;
	/*!< [in] second param. Usage depends on calibration_type */
	uint32_t param3;
	/*!< [in] third param. Usage depends on calibration_type */
};

/*
 * IOCTL definitions
 */


/**
 * Start ranging (no argument)
 *
 * @note  sysfs and ioctl control are assumed mutual exclusive use
 * control from ioctl execute with no consideration of sysfs path.
 *
 * @return :
 *	@li 0 on success
 *	@li -EBUSY if already started
 *	@li -ENXIO failed to change i2c address change after reset release
 *	@li -EIO. Read last_error to get device error code
 *	@li -ENODEV. Device has been removed.
 *
 * example user land  :
 @code
 int smtvl53lx_start(int fd){error
	int rc;
	rc= ioctl(fd, VL53LX_IOCTL_START,NULL);
	if( rc ){
		if( errno == EBUSY){
			//the device is already started
			ioctl_warn("already started");
			return EBUSY;
		}
	}
	if( rc ){
		ioctl_error("%d %s", rc,strerror(errno));
	}
	return rc;
}
 @endcode
*/

#define VL53LX_IOCTL_START			_IO('p', 0x01)

/**
 * stop ranging (no argument)

 * @note  sysfs and ioctl control are assumed mutual exclusive use
 * control from ioctl execute action with no consideration of sysfs path.
 *
 * @return
 * @li 0 on success
 * @li -EBUSY if it was already
 * @li -EIO. Read last_error to get device error code
 * @li -ENODEV. Device has been removed.
 *
 * c example userland :
 @code
int smtvl53lx_stop(int fd){
	int rc;
	rc= ioctl(fd, VL53LX_IOCTL_STOP,NULL);
	if( rc ){
		if( errno == EBUSY ){
			ioctl_warn("already stopped");
			return errno;
		}
		ioctl_error("%d %s", rc,strerror(errno));
	}
	return rc;
}
@endcode
 */
#define VL53LX_IOCTL_STOP			_IO('p', 0x05)

/**
 * set or get parameter
 *
 * @param parameter in/out  @ref stmvl53lx_parameter
 * @sa stmv53lx_parameter_name_e
 *
 * for get if ioctl fail do not check for out params it is not valid
 * for set theirs not copy back only see ioctl status, errno to get error case
 *
 * @return 0 on success else o, error check errno
 * @li -ENODEV. Device has been removed.
 *
 * @note a set parameter may not be absorbed straight aways !
 */
#define VL53LX_IOCTL_PARAMETER \
	_IOWR('p', 0x0d, struct stmvl53lx_parameter)


/**
 * Set/Get roi
 *
 * shall only be used when device is stopped (EBUSY error otherwise)
 *
 * @param roi_cfg [in/out] type @ref stmvl53lx_ioctl_roi_t
 * @warning the coordinate system is not usual image x,y (y down)but traditional
 * euclidean x,y (y up)
 *
 * @note roi validity is only checked at start ranging , as such invalid roi set
 * can make start to fail
 *
 * @return 0 on success , see errno for error detail
 *  @li -EFAULT fault in cpy to/from user. data not copied
 *  @li -EBUSY when trying to set roi while ranging
 *  @li -ENODEV device has been removed
 *  @li other errno code could be ll driver specific
 */
#define VL53LX_IOCTL_ROI\
	_IOWR('p', 0x0e, struct stmvl53lx_ioctl_roi_t)

/**
 * Get multi object ranging data
 *
 * this call is non blocking and will return what available internally
 * in all case (veen error)
 *
 * @param [out] multi range @ref VL53LX_MultiRangingData_t always update
 * but -EFAULT error case
 *
 * @return 0 on success else o, error check errno
 * @li -EFAULT fault in cpy to f/m user out range data not copied
 * @li -ENOEXEC active mode is not multirange
 * @li -ENODEV device is not ranging or device has been removed.
 * as in that case MZ data may not be fully valid
 */
#define VL53LX_IOCTL_MZ_DATA\
	_IOR('p', 0x0f, VL53LX_MultiRangingData_t)

/**
 * Get multi object ranging data
 *
 * this call is equivalent to VL53LX_IOCTL_MZ_DATA but will block until
 * new data are available since previous call.
 *
 * @param [out] multi  range @ref VL53LX_MultiRangingData_t always update
 * but -EFAULT error case
 *
 * @return 0 on success else o, error check errno
 * @li -EFAULT fault in cpy to f/m user out range data not copied
 * @li -ENOEXEC active mode is not multirange
 * @li -ENODEV device is not ranging or device has been removed.
 * @li -ERESTARTSYS interrupt while sleeping.
 * as in that case MZ data may not be fully valid
 */
#define VL53LX_IOCTL_MZ_DATA_BLOCKING\
	_IOR('p', 0x11, VL53LX_MultiRangingData_t)

/**
 * Get / set calibration data
 *
 * this call allow client to either read calibration data after calibration
 * has been performed to store them in the host filesystem or push calibration
 * data before ranging at each start-up.
 *
 * @param [in/out] data struct ptr of type
 * @ref stmvl53lx_ioctl_calibration_data_t. Caller should consider it as an
 * opaque structure.
 *
 * use this after either VL53LX_CALIBRATION_REF_SPAD,
 * VL53LX_CALIBRATION_CROSSTALK or VL53LX_CALIBRATION_OFFSET.
 *
 * @return 0 on success else o, error check errno
 * @li -EFAULT fault in cpy to f/m user out range data not copied
 * @li -EBUSY when trying to set calibration data while ranging
 * @li -EIO. Read last_error to get device error code
 * @li -ENODEV. Device has been removed.
 */
#define VL53LX_IOCTL_CALIBRATION_DATA\
	_IOWR('p', 0x12, struct stmvl53lx_ioctl_calibration_data_t)

/**
 * perform calibration squence according to calibration_type
 *
 * this call is attended to be used during factory calibration. You select
 * calibration to issue using calibration_type.
 *
 * @param [in] data struct ptr of type
 * @ref stmvl53lx_ioctl_perform_calibration_t.
 *
 * @return 0 on success else o, error check errno
 * @li -EFAULT fault in cpy to f/m user out range data not copied
 * @li -EBUSY when trying to perform calibration data while ranging
 * @li -EIO. Read last_error to get device error code
 * @li -ENODEV. Device has been removed.
 */
#define VL53LX_IOCTL_PERFORM_CALIBRATION\
	_IOW('p', 0x13, struct stmvl53lx_ioctl_perform_calibration_t)

/**
 * Get UID data
 *
 * this call is non blocking and will return what available internally
 * in all case (even error)
 *
 * @param parameter out @ref uint64_t
 *
 * @return 0 on success else o, error check errno
 * @li -ENODEV device has been removed.
 */
#define VL53LX_IOCTL_UID_DATA\
	_IOR('p', 0x14, uint64_t)


/** @} */ /* ioctl group */
#endif /* STMVL53LX_IF_H */
