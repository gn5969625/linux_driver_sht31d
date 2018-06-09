#include <linux/module.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <asm/mach/map.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/sysfs.h>

#define DRV_VERSION "V1.0"
static struct i2c_client *sht31_client;
static ssize_t sht31_reg_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	return 0;
}
static ssize_t sht31_reg_set(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	return 0;
}
static DEVICE_ATTR(sht31_reg, 0644 , sht31_reg_show, sht31_reg_set);
static struct attribute *sht31_attrs[] = {
    &dev_attr_sht31_reg.attr,
    NULL
};
static struct attribute_group sht31_attr_group = {
    .name = "sht31_reg",
    .attrs = sht31_attrs,
};

static int sht31_dev_init(void)
{
	char res;
	printk("%s called\n", __func__);
	return 0;
}

static int sht31_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	int ret;
	dev_dbg(&i2c->dev, "%s\n", __func__);
	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C))
                return -ENODEV;

	dev_info(&i2c->dev, "chip found, driver version " DRV_VERSION "\n");
	sht31_client = i2c;
	sht31_dev_init();
	printk("sht31 device component found!~\n");
	ret = sysfs_create_group(&i2c->dev.kobj, &sht31_attr_group);
	return 0;
}
static int sht31_remove(struct i2c_client *i2c)
{
	sysfs_remove_group(&i2c->dev.kobj, &sht31_attr_group);
	return 0;
}

static const struct i2c_device_id sht31_id[] = {  
    {"sht31", 0},
    {}
};
MODULE_DEVICE_TABLE(i2c, sht31_id);

static struct of_device_id sht31_of_match[] = {
	{ .compatible = "sensiron,sht31"},
	{}
};
MODULE_DEVICE_TABLE(of, sht31_of_match);
struct i2c_driver sht31_driver = {
    .driver = {
        .name           = "sht31",
        .owner          = THIS_MODULE,
        .of_match_table = of_match_ptr(sht31_of_match),
    },
    .probe      = sht31_probe,
    .remove     = sht31_remove,
    .id_table   = sht31_id,
};
module_i2c_driver(sht31_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kevin.Shen");
MODULE_DESCRIPTION("A i2c-sht31 driver for testing module");
MODULE_VERSION(DRV_VERSION);
