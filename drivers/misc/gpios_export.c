#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>

static int gpio_export_probe(struct platform_device *pdev)
{
	char gpio_name[32]={0};
	struct device_node *np = pdev->dev.of_node;
	enum of_gpio_flags gpio_flag;	
	u32 count, i;
	int flags;
	int export_gpio;
	int ret = 0;

	ret = of_property_read_u32(np, "count", &count);
	if (ret) {
		dev_err(&pdev->dev, "can't get reg property (gpio counts)\n");
		return ret;
	}
	
	for( i= 0; i < count; i++){
		snprintf(gpio_name, sizeof(gpio_name), "gpio-%u", i);
		export_gpio = of_get_named_gpio_flags(np, gpio_name, 0, &gpio_flag);

		if (gpio_is_valid(export_gpio)) {
			if(gpio_flag)
				flags = GPIOF_OUT_INIT_HIGH;
			else
				flags = GPIOF_OUT_INIT_LOW;

			ret = devm_gpio_request_one(&pdev->dev,
					export_gpio, flags, gpio_name);
			if (ret) {
				dev_err(&pdev->dev, "unable to get gpio %s\n", gpio_name);
				return ret;
			}

			//export the gpio.
			gpio_export(export_gpio, 1);
		}else{
			if (export_gpio == PTR_ERR(-EPROBE_DEFER)) {
				return PTR_ERR(-EPROBE_DEFER);
			}

			dev_err(&pdev->dev, "%s is invalid\n", gpio_name);
			return -1;
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
		.of_match_table = of_match_ptr(of_gpios_export_match),
	},
	.probe		= gpio_export_probe,
};

static int __init gpio_export_init(void)
{
	return platform_driver_register(&gpio_export_driver);
}

subsys_initcall(gpio_export_init);

MODULE_AUTHOR("chang.qing");
MODULE_DESCRIPTION("GIPO export driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:gpio_export");
