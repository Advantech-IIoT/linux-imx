#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio.h>

static int gpio_export_probe(struct platform_device *pdev)
{
	char gpio_name[32]={0};
	struct device_node *np = pdev->dev.of_node;
	struct gpio_desc *desc;
	u32 count, i;
	unsigned int export_gpio;
	bool output;
	int ret = 0;

	ret = of_property_read_u32(np, "gpio_counts", &count);
	if (ret) {
		dev_err(&pdev->dev, "can't get reg property (gpio counts)\n");
		return ret;
	}

	for( i= 0; i < count; i++){
		snprintf(gpio_name, sizeof(gpio_name), "gpio-%u", i);

		if (of_property_read_bool(np, "adv,export-gpio-use-number")){
			u32 gpio_prop[2];

			ret = of_property_read_u32_array(np, gpio_name, gpio_prop, 2);
			if (ret) {
				dev_err(&pdev->dev, "invalid %s property\n", gpio_name);
				return ret;
			}

			export_gpio = gpio_prop[0];
			output = !!gpio_prop[1];

			desc = gpio_to_desc(export_gpio);
			if (!desc) {
				dev_err(&pdev->dev,
					"%s: invalid GPIO %u\n",
					gpio_name, export_gpio);
				return -EINVAL;
			}

			/*
			 * Request GPIO using descriptor API.
			 */
			ret = devm_gpio_request(&pdev->dev,
						export_gpio, gpio_name);
			if (ret) {
				dev_err(&pdev->dev,
					"unable to request GPIO %u (%s): %d\n",
					export_gpio, gpio_name, ret);
				return ret;
			}

			/*
			 * Set direction.
			 */
			if (output)
				ret = gpiod_direction_output(desc, 0);
			else
				ret = gpiod_direction_input(desc);

			if (ret) {
				dev_err(&pdev->dev,
					"unable to set direction for GPIO %u: %d\n",
					export_gpio, ret);
				return ret;
			}
		}else {
			desc = devm_fwnode_gpiod_get_index(
				&pdev->dev,
				of_fwnode_handle(np),
				gpio_name,
				0,
				GPIOD_ASIS,
				gpio_name);

			if (IS_ERR(desc)) {
				ret = PTR_ERR(desc);
				dev_err(&pdev->dev,
					"unable to get %s: %d\n",
					gpio_name, ret);
				return ret;
			}

			export_gpio = desc_to_gpio(desc);

			ret = gpiod_get_direction(desc);
			if (ret < 0) {
				dev_err(&pdev->dev,
					"unable to get direction of %s: %d\n",
					gpio_name, ret);
				return ret;
			}

			if (ret == 0)
				output = true;
			else
				output = false;

			if (output)
				ret = gpiod_direction_output(desc, 0);
			else
				ret = gpiod_direction_input(desc);

			if (ret) {
				dev_err(&pdev->dev,
					"unable to set direction of %s: %d\n",
					gpio_name, ret);
				return ret;
			}
		}

		/*
		 * Export GPIO to /sys/class/gpio/
		 *
		 * direction_may_change = true
		 */
		ret = gpiod_export(desc, true);
		if (ret) {
			dev_err(&pdev->dev,
				"unable to export %s: %d\n",
				gpio_name, ret);
			return ret;
		}
		dev_info(&pdev->dev,
			 "exported %s (GPIO %u, %s)\n",
			 gpio_name,
			 export_gpio,
			 output ? "output" : "input");
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
