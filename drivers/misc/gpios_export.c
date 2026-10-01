#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio.h>

static int gpio_export_probe(struct platform_device *pdev)
{
	char gpio_name[32] = {0};
	char output_name[48] = {0};
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct gpio_desc *desc;
	u32 count, i;
	int ret = 0;

	if (!np)
		return -ENODEV;

	ret = of_property_read_u32(np, "gpio_counts", &count);
	if (ret) {
		dev_err(dev, "can't get gpio_counts\n");
		return ret;
	}

	for( i= 0; i < count; i++){
		/*
		 * GPIO property:
		 *
		 *     gpio-0-gpios = <...>;
		 *     gpio-1-gpios = <...>;
		 *
		 * con_id = "gpio-0", "gpio-1", ...
		 */
		snprintf(gpio_name, sizeof(gpio_name),
			 "gpio-%u", i);

		/*
		 * Optional direction property:
		 *
		 *     gpio-0-output;
		 *
		 * If present:
		 *     output, initial value = 0
		 *
		 * If absent:
		 *     input
		 */
		snprintf(output_name, sizeof(output_name),
			 "%s-output", gpio_name);

		if (of_property_read_bool(np, output_name)) {
			desc = devm_gpiod_get_index(
				dev,
				gpio_name,
				0,
				GPIOD_OUT_LOW);

			if (IS_ERR(desc)) {
				ret = PTR_ERR(desc);

				dev_err(dev,
					"unable to get %s as output: %d\n",
					gpio_name, ret);
				return ret;
			}

			dev_info(dev,
				 "exported %s as output\n",
				 gpio_name);
		} else {
			desc = devm_gpiod_get_index(
				dev,
				gpio_name,
				0,
				GPIOD_IN);

			if (IS_ERR(desc)) {
				ret = PTR_ERR(desc);

				dev_err(dev,
					"unable to get %s as input: %d\n",
					gpio_name, ret);
				return ret;
			}

			dev_info(dev,
				 "exported %s as input\n",
				 gpio_name);
		}

		/*
		 * Export GPIO to /sys/class/gpio/
		 *
		 * direction_may_change = true
		 */
		ret = gpiod_export(desc, true);
		if (ret) {
			dev_err(dev,
				"unable to export %s: %d\n",
				gpio_name, ret);
			return ret;
		}
	}

	return 0;
}

static const struct of_device_id of_gpios_export_match[] = {
	{ .compatible = "adv,gpio_export", },
	{},
};

static struct platform_driver gpio_export_driver = {
	.driver		= {
		.name	= "gpios_export",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(of_gpios_export_match),
	},
	.probe		= gpio_export_probe,
};

static int __init gpio_export_init(void)
{
	return platform_driver_register(&gpio_export_driver);
}

module_init(gpio_export_init);

MODULE_AUTHOR("chang.qing");
MODULE_DESCRIPTION("GIPO export driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:gpio_export");
