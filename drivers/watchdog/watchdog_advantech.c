/*******************************************************************************
 ** Copyright © 2023 Advantech IIoT
 ** All rights reserved.
 **
 ** This program is free software: you can redistribute it and/or modify
 ** it under the terms of the GNU General Public License as published by
 ** the Free Software Foundation; under version 2 of the License.
 **
 ** This program is distributed in the hope that it will be useful,
 ** but WITHOUT ANY WARRANTY; without even the implied warranty of
 ** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 ** GNU General Public License for more details.
 **
 ** You should have received a copy of the GNU General Public License
 ** along with this program.  If not, see <http://www.gnu.org/licenses/>.
 **
 ** SPDX-License-Identifier: GPL-2.0-only
 **
 ** filename: watchdog_advantech.c
 ** description: The driver is for Advantech i2c HW Watchdog (MSP430 FW:V17).
 ** created: chic.lee@advantech.com.tw
 **
 ******************************************************************************/

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/watchdog.h>
#include <linux/clk.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/reboot.h>

#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/watchdog.h>

#include <asm/system_misc.h>

#define VERSION_STR			        "1.0"
#define DRIVER_NAME                 "adv-wdt-i2c"

#define ADV_WDT_MAX_TIME            6527	/* in seconds */
#define ADV_WDT_DEFAULT_TIME        60		/* in seconds */
#define WDOG_COUNT_UNIT_MULTIPLE    10      // 1s (1000ms) / 100ms
#define WDOG_SEC_TO_COUNT(s)        (s * WDOG_COUNT_UNIT_MULTIPLE)	/* Time unit for register: 100ms */

#define REG_WDT_WATCHDOG_TIME_OUT	0x15
#define REG_WDT_POWER_OFF_TIME 		0x16
#define REG_WDT_INT_PRE_TIME 		0x17
#define REG_WDT_REMAIN_TIME_OUT		0x25
#define REG_WDT_REMAIN_PRE_TIME 	0x26
#define REG_WDT_VERSION 			0x27
#define REG_WDT_POWER_BTN_MODE 		0x28

static struct {
    struct watchdog_device wdog;
	int wdt_ping_status;
	int wdt_en_off;
    int gpio_wdt_en;
    int gpio_wdt_ping;
	unsigned char version[2];
} adv_wdt;

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started (default="
				__MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

static unsigned int timeout = ADV_WDT_DEFAULT_TIME;
module_param(timeout, uint, 0);
MODULE_PARM_DESC(timeout, "Watchdog timeout in seconds (default="
				__MODULE_STRING(ADV_WDT_DEFAULT_TIME) ")");

struct watchdog_info adv_wdt_info = {
	.identity = "Advantech i2c watchdog",
	.options = WDIOF_KEEPALIVEPING | WDIOF_SETTIMEOUT | WDIOF_MAGICCLOSE,
	.firmware_version = 0,
};

int adv_wdt_i2c_set_timeout(struct i2c_client *client, unsigned int val)
{
	int ret = 0;
	val = WDOG_SEC_TO_COUNT(val) & 0x0000FFFF;
    if((ret=i2c_smbus_write_word_data(client, (u8)REG_WDT_WATCHDOG_TIME_OUT, (u16)val))!=0){
		printk("%s, %d: ERROR!! ret: %d\n", __FUNCTION__, __LINE__, ret);
		return 0;
	}

	msleep(100);
	return 0;
}

static int adv_wdt_set_timeout(struct watchdog_device *wdog,
				   unsigned int timeout)
{
    struct i2c_client *client = to_i2c_client(wdog->parent);
    unsigned int actual;
    actual = min(timeout, ADV_WDT_MAX_TIME);
    int ret = adv_wdt_i2c_set_timeout(client, actual);
    if(ret == 0)
    {
        wdog->timeout = actual;
    }

	return ret;
}

int adv_wdt_i2c_read_timeout(struct i2c_client *client, unsigned int *val)
{
	int ret = 0;

	*val=(i2c_smbus_read_word_data(client, (u8)REG_WDT_WATCHDOG_TIME_OUT) & 0xFFFF);
	// write data is value x 10, so get data we need value / 10
	*val/=10;
	return 0;
}

int adv_wdt_i2c_read_remain_time(struct i2c_client *client, unsigned int *val)
{
	int ret = 0;

	*val=(i2c_smbus_read_word_data(client, (u8)REG_WDT_REMAIN_TIME_OUT) & 0xFFFF);
    *val/=WDOG_COUNT_UNIT_MULTIPLE;
	return 0;
}

static unsigned int adv_wdt_get_timeleft(struct watchdog_device *wdog)
{
    struct i2c_client *client = to_i2c_client(wdog->parent);
    unsigned int val = 0;
	int ret = adv_wdt_i2c_read_remain_time(client, &val);
    if(ret == 0) return val & 0xffff;

    return 0;
}

int adv_wdt_i2c_read_version(struct i2c_client *client, unsigned int *val)
{
	*val=(i2c_smbus_read_word_data(client, (u8)REG_WDT_VERSION) & 0xFFFF);
	return 0;
}

static int adv_wdt_ping(struct watchdog_device *wdog)
{
	/* watchdog counter refresh input. Both edge trigger */
	adv_wdt.wdt_ping_status= !adv_wdt.wdt_ping_status;
	gpio_set_value(adv_wdt.gpio_wdt_ping, adv_wdt.wdt_ping_status);

    return 0;
}

static int adv_wdt_start(struct watchdog_device *wdog)
{
	gpio_set_value(adv_wdt.gpio_wdt_en, !adv_wdt.wdt_en_off);
	adv_wdt_ping(wdog);
    return 0;
}

static int adv_wdt_stop(struct watchdog_device *wdog)
{
	adv_wdt_ping(wdog);

	/* we don't need a clk_disable, it cannot be disabled once started.
	 * We use a timer to ping the watchdog while /dev/watchdog is closed */
	gpio_set_value(adv_wdt.gpio_wdt_en, adv_wdt.wdt_en_off);
    msleep(100);

    return 0;
}

static const struct watchdog_ops adv_wdt_ops = {
	.owner          = THIS_MODULE,
	.start          = adv_wdt_start,
	.stop           = adv_wdt_stop,
	.ping           = adv_wdt_ping,
	.set_timeout	= adv_wdt_set_timeout,
	.get_timeleft	= adv_wdt_get_timeleft,
};

static int adv_wdt_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
	unsigned int tmp_version, init_timeout = 0;
	struct device_node *np = client->dev.of_node;
	enum of_gpio_flags flags;

	if (!np)
	{
		return -ENODEV;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		return -ENODEV;
	}	

	i2c_set_clientdata(client, &adv_wdt);

	//Setting GPIO
	adv_wdt.gpio_wdt_en = of_get_named_gpio_flags(np, "wdt-en", 0, &flags);
	if (!gpio_is_valid(adv_wdt.gpio_wdt_en))
		return -ENODEV;	
	adv_wdt.wdt_en_off = flags;
	ret = devm_gpio_request_one(&client->dev, adv_wdt.gpio_wdt_en,
				GPIOF_OUT_INIT_LOW, "adv_wdt.wdt_en");
	if (ret < 0) {
		dev_err(&client->dev, "request gpio failed: %d\n", ret);
		return ret;
	}
	gpio_direction_output(adv_wdt.gpio_wdt_en, adv_wdt.wdt_en_off);

	adv_wdt.gpio_wdt_ping = of_get_named_gpio_flags(np, "wdt-ping", 0, &flags);
	if (!gpio_is_valid(adv_wdt.gpio_wdt_ping))
		return -ENODEV;	

	ret = devm_gpio_request_one(&client->dev, adv_wdt.gpio_wdt_ping, 
				GPIOF_OUT_INIT_LOW, "adv_wdt.wdt_ping");
	if (ret < 0) {
		dev_err(&client->dev, "request gpio failed: %d\n", ret);
		return ret;
	}
	adv_wdt.wdt_ping_status=flags;
	gpio_direction_output(adv_wdt.gpio_wdt_ping, !flags);
	msleep(10);
	gpio_direction_output(adv_wdt.gpio_wdt_ping, flags);

	init_timeout = clamp_t(unsigned, timeout, 1, ADV_WDT_MAX_TIME);
	if (init_timeout != timeout)
		dev_warn(&client->dev, "Initial timeout out of range! "
			"Clamped from %u to %u\n", timeout, init_timeout);

	ret = adv_wdt_i2c_read_version(client, &tmp_version);
	
	if (ret == 0 )
	{
     adv_wdt.version[0]= (tmp_version & 0xFF00) >> 8;
     adv_wdt.version[1]= tmp_version & 0xFF;
     adv_wdt_info.firmware_version = (unsigned int)(adv_wdt.version[1] - '0') * 10 + (unsigned int)(adv_wdt.version[0] - '0');
	} else {
		pr_err("Read watchdog version err=%d\n", ret);
		goto fail;
	}
	
	dev_info(&client->dev,
						"Advantech Watchdog Driver (V%s). timeout=%ds (nowayout=%d), FW Ver.%d\n", VERSION_STR,
						init_timeout, nowayout, adv_wdt_info.firmware_version);

    adv_wdt.wdog.info = &adv_wdt_info;
    adv_wdt.wdog.ops = &adv_wdt_ops;
    adv_wdt.wdog.min_timeout = 1;
    adv_wdt.wdog.max_timeout = ADV_WDT_MAX_TIME;
    adv_wdt.wdog.parent = &client->dev;
	
    watchdog_init_timeout(&adv_wdt.wdog, init_timeout, &client->dev);

    watchdog_set_nowayout(&adv_wdt.wdog, nowayout);

	return watchdog_register_device(&adv_wdt.wdog);

fail:
	return ret;
}

static int __exit adv_wdt_i2c_remove(struct i2c_client *client)
{
    gpio_set_value(adv_wdt.gpio_wdt_en, adv_wdt.wdt_en_off);
	return 0;
}

static const struct i2c_device_id adv_wdt_i2c_id[] = {
	{DRIVER_NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, adv_wdt_i2c_id);

static const struct of_device_id adv_wdt_i2c_dt_ids[] = {
	{ .compatible = "fsl,adv-wdt-i2c", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, adv_wdt_i2c_dt_ids);

static struct i2c_driver adv_wdt_i2c_driver = {
	.driver = {
		   .name = DRIVER_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = adv_wdt_i2c_dt_ids,
		   },
	.probe = adv_wdt_i2c_probe,
	.remove = adv_wdt_i2c_remove,
	.id_table = adv_wdt_i2c_id,
};

static int __init adv_wdt_i2c_init(void)
{
	return i2c_add_driver(&adv_wdt_i2c_driver);
}

static void __exit adv_wdt_i2c_exit(void)
{
	i2c_del_driver(&adv_wdt_i2c_driver);
}

module_init(adv_wdt_i2c_init);
module_exit(adv_wdt_i2c_exit);

MODULE_AUTHOR("Chic Lee <chic.lee@advantech.com.tw>");
MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_DESCRIPTION("Advantech Watchdog I2C Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(VERSION_STR);
MODULE_INFO(Copyright, "Copyright 2023 Advantech IIoT");
