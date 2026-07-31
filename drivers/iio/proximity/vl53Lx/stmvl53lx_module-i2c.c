
// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53LX and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */

/**
 * @file stmvl53lx_module-i2c.c
 *
 *  implement STM VL53LX module interface i2c wrapper + control
 *  using linux native i2c + gpio + reg api
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
#include <linux/version.h>

/*
 * power specific includes
 */
#include <linux/pwm.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/clk.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>

#include "stmvl53lx-i2c.h"
#include "stmvl53lx.h"

#define STMVL53LX_SLAVE_ADDR	(0x52>>1)

/** @ingroup drv_port
 * @{
 */

/**
 * control specific debug message echo
 *
 * Set to 0/1 do not remove
 *
 * most dbg warn err messages goes true main driver macro
 * this one permit some specific debug without activating all main dbg
 */
#define MODI2C_DEBUG	0

/*
 * mutex to handle device i2c address changes. It allow to avoid multiple
 * device active with same i2c addresses at the same time. Note that we don't
 * support case where boot_reg has the same value as a final i2c address of
 * another device.
 */
static DEFINE_MUTEX(dev_addr_change_mutex);

/**
 * i2c client assigned to our driver
 *
 * this is use for stm test purpose as we fake client create and regstration
 * we stores the i2c client for release in clean-up overwise we wan't reload
 * the module multiple time
 *
 * in a normal dev tree prod system this is not required
 */
static struct i2c_client *stm_test_i2c_client;

/*
 * pi3:
 * insmod stmvl53lx.ko force_device=1 adapter_nb=1 xsdn_gpio_nb=19
 * intr_gpio_nb=16 pwren_gpio_nb=12
 *
 * panda
 * insmod stmvl53lx.ko force_device=1 adapter_nb=4 xsdn_gpio_nb=56
 * intr_gpio_nb=59 pwren_gpio_nb=55
 */

static int force_device;
static int adapter_nb = -1;
static int xsdn_gpio_nb = -1;
static int pwren_gpio_nb = -1;
static int intr_gpio_nb = -1;
static int i2c_addr_nb = STMVL53LX_SLAVE_ADDR;

module_param(force_device, int, 0000);
MODULE_PARM_DESC(force_device, "force device insertion at module init");

module_param(adapter_nb, int, 0000);
MODULE_PARM_DESC(adapter_nb, "i2c adapter to use");

module_param(i2c_addr_nb, int, 0000);
MODULE_PARM_DESC(i2c_addr_nb, "desired i2c device address on adapter ");

module_param(xsdn_gpio_nb, int, 0000);
MODULE_PARM_DESC(xsdn_gpio_nb, "select gpio numer to use for vl53lx reset");

module_param(pwren_gpio_nb, int, 0000);
MODULE_PARM_DESC(pwren_gpio_nb, "select gpio numer to use for vl53lx power");

module_param(intr_gpio_nb, int, 0000);
MODULE_PARM_DESC(intr_gpio_nb, "select gpio numer to use for vl53lx interrupt");

/**
 * warn message
 *
 * @warning use only in scope where i2c_data ptr is present
 **/
#define modi2c_warn(fmt, ...)\
	dev_WARN(&i2c_data->client->dev, fmt, ##__VA_ARGS__)

/**
 * err message
 *
 * @warning use only in scope where i2c_data ptr is present
 */
#define modi2c_err(fmt, ...)\
	dev_err(&i2c_data->client->dev, fmt, ##__VA_ARGS__)



#if MODI2C_DEBUG
#	define modi2c_dbg(fmt, ...)\
		pr_devel("%s "fmt"\n", __func__, ##__VA_ARGS__)
#else
#	define modi2c_dbg(...)	(void)0
#endif

static int insert_device(void)
{
	int ret = 0;
	struct i2c_adapter *adapter;
	struct i2c_board_info info = {
		.type = "stmvl53lx",
		.addr = STMVL53LX_SLAVE_ADDR,
	};

	memset(&info, 0, sizeof(info));
	strcpy(info.type, "stmvl53lx");
	info.addr = STMVL53LX_SLAVE_ADDR;
	adapter = i2c_get_adapter(adapter_nb);
	if (!adapter) {
		ret = -EINVAL;
		goto done;
	}
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 5, 0)
	stm_test_i2c_client = i2c_new_device(adapter, &info);
#else
	stm_test_i2c_client = i2c_new_client_device(adapter, &info);
#endif
	if (!stm_test_i2c_client)
		ret = -EINVAL;

done:
	return ret;
}

static int get_xsdn(struct device *dev, struct i2c_data *i2c_data)
{
	int rc = 0;

	i2c_data->io_flag.xsdn_owned = 0;
	if (i2c_data->xsdn_gpio == -1) {
		vl53lx_errmsg("reset gpio is required");
		rc = -ENODEV;
		goto no_gpio;
	}

	vl53lx_dbgmsg("request xsdn_gpio %d", i2c_data->xsdn_gpio);
	rc = gpio_request(i2c_data->xsdn_gpio, "vl53lx_xsdn");
	if (rc) {
		vl53lx_errmsg("fail to acquire xsdn %d", rc);
		goto request_failed;
	}

	rc = gpio_direction_output(i2c_data->xsdn_gpio, 0);
	if (rc) {
		vl53lx_errmsg("fail to configure xsdn as output %d", rc);
		goto direction_failed;
	}
	i2c_data->io_flag.xsdn_owned = 1;

	return rc;

direction_failed:
	gpio_free(i2c_data->xsdn_gpio);

request_failed:
no_gpio:
	return rc;
}

static void put_xsdn(struct i2c_data *i2c_data)
{
	if (i2c_data->io_flag.xsdn_owned) {
		vl53lx_dbgmsg("release xsdn_gpio %d", i2c_data->xsdn_gpio);
		gpio_free(i2c_data->xsdn_gpio);
		i2c_data->io_flag.xsdn_owned = 0;
		i2c_data->xsdn_gpio = -1;
	}
	i2c_data->xsdn_gpio = -1;
}

static int get_pwren(struct device *dev, struct i2c_data *i2c_data)
{
	int rc = 0;

	i2c_data->io_flag.pwr_owned = 0;
	if (i2c_data->pwren_gpio == -1) {
		vl53lx_wanrmsg("pwren gpio disable");
		goto no_gpio;
	}

	vl53lx_dbgmsg("request pwren_gpio %d", i2c_data->pwren_gpio);
	rc = gpio_request(i2c_data->pwren_gpio, "vl53lx_pwren");
	if (rc) {
		vl53lx_errmsg("fail to acquire pwren %d", rc);
		goto request_failed;
	}

	rc = gpio_direction_output(i2c_data->pwren_gpio, 0);
	if (rc) {
		vl53lx_errmsg("fail to configure pwren as output %d", rc);
		goto direction_failed;
	}
	i2c_data->io_flag.pwr_owned = 1;

	return rc;

direction_failed:
	gpio_free(i2c_data->xsdn_gpio);

request_failed:
no_gpio:
	return rc;
}

static void put_pwren(struct i2c_data *i2c_data)
{
	if (i2c_data->io_flag.pwr_owned) {
		vl53lx_dbgmsg("release pwren_gpio %d", i2c_data->pwren_gpio);
		gpio_free(i2c_data->pwren_gpio);
		i2c_data->io_flag.pwr_owned = 0;
		i2c_data->pwren_gpio = -1;
	}
	i2c_data->pwren_gpio = -1;
}

static int get_intr(struct device *dev, struct i2c_data *i2c_data)
{
	int rc = 0;

	i2c_data->io_flag.intr_owned = 0;
	if (i2c_data->intr_gpio == -1) {
		vl53lx_wanrmsg("no interrupt gpio");
		goto no_gpio;
	}

	vl53lx_dbgmsg("request intr_gpio %d", i2c_data->intr_gpio);
	rc = gpio_request(i2c_data->intr_gpio, "vl53lx_intr");
	if (rc) {
		vl53lx_errmsg("fail to acquire intr %d", rc);
		goto request_failed;
	}

	rc = gpio_direction_input(i2c_data->intr_gpio);
	if (rc) {
		vl53lx_errmsg("fail to configure intr as input %d", rc);
		goto direction_failed;
	}

	i2c_data->irq = gpio_to_irq(i2c_data->intr_gpio);
	if (i2c_data->irq < 0) {
		vl53lx_errmsg("fail to map GPIO: %d to interrupt:%d\n",
				i2c_data->intr_gpio, i2c_data->irq);
		goto irq_failed;
	}
	i2c_data->io_flag.intr_owned = 1;

	return rc;

irq_failed:
direction_failed:
	gpio_free(i2c_data->intr_gpio);

request_failed:
no_gpio:
	return rc;
}

static void put_intr(struct i2c_data *i2c_data)
{
	if (i2c_data->io_flag.intr_owned) {
		if (i2c_data->io_flag.intr_started) {
			free_irq(i2c_data->irq, i2c_data);
			i2c_data->io_flag.intr_started = 0;
		}
		vl53lx_dbgmsg("release intr_gpio %d", i2c_data->intr_gpio);
		gpio_free(i2c_data->intr_gpio);
		i2c_data->io_flag.intr_owned = 0;
	}
	i2c_data->intr_gpio = -1;
}

/**
 *  parse dev tree for all platform specific input
 */
static int stmvl53lx_parse_tree(struct device *dev, struct i2c_data *i2c_data)
{
	struct i2c_client *client = (struct i2c_client *) i2c_data->client;
	int rc = 0;
	u32 reg;

	/* if force device is in use then gpio nb comes from module param else
	 * we use devicetree.
	 */
	i2c_data->vdd = NULL;
	i2c_data->pwren_gpio = -1;
	i2c_data->xsdn_gpio = -1;
	i2c_data->intr_gpio = -1;
	i2c_data->boot_reg = STMVL53LX_SLAVE_ADDR;
	if (force_device) {
		i2c_data->xsdn_gpio = xsdn_gpio_nb;
		i2c_data->pwren_gpio = pwren_gpio_nb;
		i2c_data->intr_gpio = intr_gpio_nb;
		client->addr = i2c_addr_nb;
	} else if (dev->of_node) {
		/* power : either vdd or pwren_gpio. try regulator first */
		i2c_data->vdd = regulator_get_optional(dev, "vdd");
		if (IS_ERR(i2c_data->vdd) || i2c_data->vdd == NULL) {
			i2c_data->vdd = NULL;
			/* try gpio */
			rc = of_property_read_u32_array(dev->of_node,
				"pwren-gpio", &i2c_data->pwren_gpio, 1);
			if (rc) {
				i2c_data->pwren_gpio = -1;
				vl53lx_wanrmsg(
			"no regulator, nor power gpio => power ctrl disabled");
			}
		}
		rc = of_property_read_u32_array(dev->of_node, "reg",
			&reg, 1);
		if (rc) {
			vl53lx_wanrmsg("Unable to find reg %d 0x%x",
				rc, i2c_addr_nb);
			reg = i2c_addr_nb;
		}
		client->addr = reg;
		rc = of_property_read_u32_array(dev->of_node, "xsdn-gpio",
			&i2c_data->xsdn_gpio, 1);
		if (rc) {
			vl53lx_wanrmsg("Unable to find xsdn-gpio %d %d",
				rc, i2c_data->xsdn_gpio);
			i2c_data->xsdn_gpio = -1;
		}
		rc = of_property_read_u32_array(dev->of_node, "intr-gpio",
			&i2c_data->intr_gpio, 1);
		if (rc) {
			vl53lx_wanrmsg("Unable to find intr-gpio %d %d",
				rc, i2c_data->intr_gpio);
			i2c_data->intr_gpio = -1;
		}
		rc = of_property_read_u32_array(dev->of_node, "boot-reg",
			&i2c_data->boot_reg, 1);
		if (rc) {
			vl53lx_wanrmsg("Unable to find boot-reg %d %d",
				rc, i2c_data->boot_reg);
			i2c_data->boot_reg = STMVL53LX_SLAVE_ADDR;
		}
	}

	/* configure gpios */
	rc = get_xsdn(dev, i2c_data);
	if (rc)
		goto no_xsdn;
	rc = get_pwren(dev, i2c_data);
	if (rc)
		goto no_pwren;
	rc = get_intr(dev, i2c_data);
	if (rc)
		goto no_intr;

	return rc;

no_intr:
	if (i2c_data->vdd) {
		regulator_put(i2c_data->vdd);
		i2c_data->vdd = NULL;
	}
	put_pwren(i2c_data);
no_pwren:
	put_xsdn(i2c_data);
no_xsdn:
	return rc;
}

static void stmvl53lx_release_gpios(struct i2c_data *i2c_data)
{
	put_xsdn(i2c_data);
	if (i2c_data->vdd) {
		regulator_put(i2c_data->vdd);
		i2c_data->vdd = NULL;
	}
	put_pwren(i2c_data);
	put_intr(i2c_data);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 3, 0)
static int stmvl53lx_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
#else
static int stmvl53lx_probe(struct i2c_client *client)
#endif
{
	int rc = 0;
	struct stmvl53lx_data *vl53lx_data = NULL;
	struct i2c_data *i2c_data = NULL;

	vl53lx_dbgmsg("Enter %s : 0x%02x\n", client->name, client->addr);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE)) {
		rc = -EIO;
		return rc;
	}

	vl53lx_data = kzalloc(sizeof(struct stmvl53lx_data), GFP_KERNEL);
	if (!vl53lx_data) {
		rc = -ENOMEM;
		return rc;
	}
	if (vl53lx_data) {
		vl53lx_data->client_object =
				kzalloc(sizeof(struct i2c_data), GFP_KERNEL);
		if (!vl53lx_data)
			goto done_freemem;
		i2c_data = (struct i2c_data *)vl53lx_data->client_object;
	}
	i2c_data->client = client;
	i2c_data->vl53lx_data = vl53lx_data;
	i2c_data->irq = -1 ; /* init to no irq */

	/* parse and configure hardware */
	rc = stmvl53lx_parse_tree(&i2c_data->client->dev, i2c_data);
	if (rc)
		goto done_freemem;

	/* setup device name */
	/* vl53lx_data->dev_name = dev_name(&client->dev); */

	/* setup client data */
	i2c_set_clientdata(client, vl53lx_data);

	/* end up by core driver setup */
	rc = stmvl53lx_setup(vl53lx_data);
	if (rc)
		goto release_gpios;
	vl53lx_dbgmsg("End\n");

	kref_init(&i2c_data->ref);

	return rc;

release_gpios:
	stmvl53lx_release_gpios(i2c_data);

done_freemem:
	/* kfree safe against NULL */
	kfree(vl53lx_data);
	kfree(i2c_data);

	return -1;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
static int stmvl53lx_remove(struct i2c_client *client)
#else
static void stmvl53lx_remove(struct i2c_client *client)
#endif 
{
	struct stmvl53lx_data *data = i2c_get_clientdata(client);
	struct i2c_data *i2c_data = (struct i2c_data *)data->client_object;

	vl53lx_dbgmsg("Enter\n");
	mutex_lock(&data->work_mutex);
	/* main driver cleanup */
	stmvl53lx_cleanup(data);

	/* release gpios */
	stmvl53lx_release_gpios(i2c_data);

	mutex_unlock(&data->work_mutex);

	stmvl53lx_put(data->client_object);

	vl53lx_dbgmsg("End\n");

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
	return 0;
#endif 
}

#ifdef CONFIG_PM_SLEEP
static int stmvl53lx_suspend(struct device *dev)
{
	struct stmvl53lx_data *data = i2c_get_clientdata(to_i2c_client(dev));

	vl53lx_dbgmsg("Enter\n");
	mutex_lock(&data->work_mutex);
	/* Stop ranging */
	stmvl53lx_pm_suspend_stop(data);

	mutex_unlock(&data->work_mutex);

	vl53lx_dbgmsg("End\n");

	return 0;
}

static int stmvl53lx_resume(struct device *dev)
{
#if 0
	struct stmvl53lx_data *data = i2c_get_clientdata(to_i2c_client(dev));

	vl53lx_dbgmsg("Enter\n");

	mutex_lock(&data->work_mutex);

	/* do nothing user will restart measurements */

	mutex_unlock(&data->work_mutex);

	vl53lx_dbgmsg("End\n");
#else
	vl53lx_dbgmsg("Enter\n");
	vl53lx_dbgmsg("End\n");
#endif
	return 0;
}
#endif


static SIMPLE_DEV_PM_OPS(stmvl53lx_pm_ops, stmvl53lx_suspend, stmvl53lx_resume);

static const struct i2c_device_id stmvl53lx_id[] = {
	{ STMVL53LX_DRV_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, stmvl53lx_id);

static const struct of_device_id st_stmvl53lx_dt_match[] = {
	{ .compatible = "st,"STMVL53LX_DRV_NAME, },
	{ },
};

static struct i2c_driver stmvl53lx_driver = {
	.driver = {
		.name	= STMVL53LX_DRV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = st_stmvl53lx_dt_match,
		.pm	= &stmvl53lx_pm_ops,
	},
	.probe	= stmvl53lx_probe,
	.remove	= stmvl53lx_remove,
	.id_table = stmvl53lx_id,

};

/**
 * give power to device
 *
 * @param object  the i2c layer object
 * @return
 */
int stmvl53lx_power_up_i2c(void *object)
{
	int rc = 0;
	struct i2c_data *data = (struct i2c_data *) object;

	vl53lx_dbgmsg("Enter\n");

	/* turn on power */
	if (data->vdd) {
		rc = regulator_enable(data->vdd);
		if (rc) {
			vl53lx_errmsg("fail to turn on regulator");
			return rc;
		}
	} else if (data->pwren_gpio != -1) {
		gpio_set_value(data->pwren_gpio, 1);
		vl53lx_info("slow power on");
	} else
		vl53lx_wanrmsg("no power control");

	return rc;
}

/**
 * remove power to device (reset it)
 *
 * @param i2c_object the i2c layer object
 * @return 0 on success
 */
int stmvl53lx_power_down_i2c(void *i2c_object)
{
	struct i2c_data *data = (struct i2c_data *) i2c_object;
	int rc = 0;

	vl53lx_dbgmsg("Enter\n");

	/* turn off power */
	if (data->vdd) {
		rc = regulator_disable(data->vdd);
		if (rc)
			vl53lx_errmsg("reg disable failed. rc=%d\n",
				rc);
	} else if (data->pwren_gpio != -1) {
		gpio_set_value(data->pwren_gpio, 0);
	}
	vl53lx_dbgmsg("power off");

	vl53lx_dbgmsg("End\n");

	return rc;
}

static int handle_i2c_address_device_change_lock(struct i2c_data *data)
{
	struct i2c_client *client = (struct i2c_client *) data->client;
	uint8_t buffer[3];
	struct i2c_msg msg;
	int rc = 0;

	vl53lx_dbgmsg("change device i2c address from 0x%02x to 0x%02x",
		data->boot_reg, client->addr);
	/* no i2c-access must occur before fw boot time */
	usleep_range(VL53LX_FIRMWARE_BOOT_TIME_US,
		VL53LX_FIRMWARE_BOOT_TIME_US + 1);

	/* manually send message to update i2c address */
	buffer[0] = (VL53LX_I2C_SLAVE__DEVICE_ADDRESS >> 8) & 0xFF;
	buffer[1] = (VL53LX_I2C_SLAVE__DEVICE_ADDRESS >> 0) & 0xFF;
	buffer[2] = client->addr;
	msg.addr = data->boot_reg;
	msg.flags = client->flags;
	msg.buf = buffer;
	msg.len = 3;
	if (i2c_transfer(client->adapter, &msg, 1) != 1) {
		rc = -ENXIO;
		vl53lx_errmsg("Fail to change i2c address to 0x%02x",
			client->addr);
	}

	return rc;
}

/* reset release will also handle device address change. It will avoid state
 * where multiple stm53l3cx are bring out of reset at the same time with the
 * same boot address.
 * Note that we don't manage case where boot_reg has the same value as a final
 * i2c address of another device. This case is not supported and will lead
 * to unpredictable behavior.
 */
static int release_reset(struct i2c_data *data)
{
	struct i2c_client *client = (struct i2c_client *) data->client;
	int rc = 0;
	bool is_address_change = client->addr != data->boot_reg;

	if (is_address_change)
		mutex_lock(&dev_addr_change_mutex);

	gpio_set_value(data->xsdn_gpio, 1);
	if (is_address_change) {
		rc = handle_i2c_address_device_change_lock(data);
		if (rc)
			gpio_set_value(data->xsdn_gpio, 0);
	}

	if (is_address_change)
		mutex_unlock(&dev_addr_change_mutex);

	return rc;
}

/**
 * release device reset
 *
 * @param i2c_object the i2c layer object
 * @return 0 on success
 */
int stmvl53lx_reset_release_i2c(void *i2c_object)
{
	int rc;
	struct i2c_data *data = (struct i2c_data *) i2c_object;

	vl53lx_dbgmsg("Enter\n");

	rc = release_reset(data);
	if (rc)
		goto error;

	/* and now wait for device end of boot */
	data->vl53lx_data->is_delay_allowed = true;
	rc = VL53LX_WaitDeviceBooted(&data->vl53lx_data->stdev);
	data->vl53lx_data->is_delay_allowed = false;
	if (rc) {
		gpio_set_value(data->xsdn_gpio, 0);
		vl53lx_errmsg("boot fail with error %d", rc);
		data->vl53lx_data->last_error = rc;
		rc = -EIO;
	}

error:
	vl53lx_dbgmsg("End\n");

	return rc;
}

/**
 * put device under reset
 *
 * @param i2c_object the i2c layer object
 * @return 0 on success
 */
int stmvl53lx_reset_hold_i2c(void *i2c_object)
{
	struct i2c_data *data = (struct i2c_data *) i2c_object;

	vl53lx_dbgmsg("Enter\n");

	gpio_set_value(data->xsdn_gpio, 0);

	vl53lx_dbgmsg("End\n");

	return 0;
}

int stmvl53lx_init_i2c(void)
{
	int ret = 0;

	vl53lx_dbgmsg("Enter\n");

	/* register as a i2c client device */
	ret = i2c_add_driver(&stmvl53lx_driver);
	if (ret)
		vl53lx_errmsg("%d erro ret:%d\n", __LINE__, ret);

	if (!ret && force_device)
		ret = insert_device();

	if (ret)
		i2c_del_driver(&stmvl53lx_driver);

	vl53lx_dbgmsg("End with rc:%d\n", ret);

	return ret;
}


void stmvl53lx_clean_up_i2c(void)
{
	if (stm_test_i2c_client) {
		vl53lx_dbgmsg("to unregister i2c client\n");
		i2c_unregister_device(stm_test_i2c_client);
	}
}

static irqreturn_t stmvl53lx_irq_handler_i2c(int vec, void *info)
{
	struct i2c_data *i2c_data = (struct i2c_data *)info;

	if (i2c_data->irq == vec) {
		modi2c_dbg("irq");
		stmvl53lx_intr_handler(i2c_data->vl53lx_data);
		modi2c_dbg("over");
	} else {
		if (!i2c_data->msg_flag.unhandled_irq_vec) {
			modi2c_warn("unmatching vec %d != %d\n",
					vec, i2c_data->irq);
			i2c_data->msg_flag.unhandled_irq_vec = 1;
		}
	}

	return IRQ_HANDLED;
}

/**
 * enable and start intr handling
 *
 * @param object  our i2c_data specific object
 * @param poll_mode [in/out] set to force mode clear to use irq
 * @return 0 on success and set ->poll_mode if it faill ranging wan't start
 */
int stmvl53lx_start_intr(void *object, int *poll_mode)
{
	struct i2c_data *i2c_data;
	int rc;

	i2c_data = (struct i2c_data *)object;
	/* irq and gpio acquire config done in parse_tree */
	if (i2c_data->irq < 0) {
		/* the i2c tree as no intr force polling mode */
		*poll_mode = -1;
		return 0;
	}
	/* clear irq warning report enabe it again for this session */
	i2c_data->msg_flag.unhandled_irq_vec = 0;
	/* if started do no nothing */
	if (i2c_data->io_flag.intr_started) {
		/* nothing to do */
		*poll_mode = 0;
		return 0;
	}

	vl53lx_dbgmsg("to register_irq:%d\n", i2c_data->irq);
	rc = request_threaded_irq(i2c_data->irq, NULL,
			stmvl53lx_irq_handler_i2c,
			IRQF_TRIGGER_FALLING|IRQF_ONESHOT,
			"vl53lx_interrupt",
			(void *)i2c_data);
	if (rc) {
		vl53lx_errmsg("fail to req threaded irq rc=%d\n", rc);
		*poll_mode = 0;
	} else {
		vl53lx_dbgmsg("irq %d now handled\n", i2c_data->irq);
		i2c_data->io_flag.intr_started = 1;
		*poll_mode = 0;
	}
	return rc;
}

void *stmvl53lx_get(void *object)
{
	struct i2c_data *data = (struct i2c_data *) object;

	vl53lx_dbgmsg("Enter\n");
	kref_get(&data->ref);
	vl53lx_dbgmsg("End\n");

	return object;
}

static void memory_release(struct kref *kref)
{
	struct i2c_data *data = container_of(kref, struct i2c_data, ref);

	vl53lx_dbgmsg("Enter\n");
	kfree(data->vl53lx_data);
	kfree(data);
	vl53lx_dbgmsg("End\n");
}

void stmvl53lx_put(void *object)
{
	struct i2c_data *data = (struct i2c_data *) object;

	vl53lx_dbgmsg("Enter\n");
	kref_put(&data->ref, memory_release);
	vl53lx_dbgmsg("End\n");
}

void __exit stmvl53lx_exit_i2c(void *i2c_object)
{
	vl53lx_dbgmsg("Enter\n");
	i2c_del_driver(&stmvl53lx_driver);
	vl53lx_dbgmsg("End\n");
}
/** @} */ /* ingroup drv_port */
