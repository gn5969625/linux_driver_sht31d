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
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/types.h>
#include "sht31.h"

//CRC-8 x8+x5+x4+1 0x31
// * IIO driver for sht31; 7-bit I2C address: 0x44 (default)
//the sht31 use command code format to communcate,but you could  use reg address concept to send it...
//#include "sht31.h"
#include <linux/sysfs.h>
#define DRV_VERSION "V2.0"

typedef struct sht31_data_format{
        unsigned char val_msb;
        unsigned char val2_lsb;
}data_format;

struct sht31 {
        struct device                   *dev;
	struct i2c_client		*client;
        /* The iio sysfs interface doesn't prevent concurrent reads: */
        struct mutex                    lock;
	data_format			temperature;
	data_format			humidity;
};
static struct i2c_client *sht31_client;

static int i2c_sht31_read_len(struct i2c_client *client,unsigned char len,unsigned char *result_buf)
{
	int ret;
        struct i2c_msg msg[] = {
                {client->addr,I2C_M_RD,len,result_buf}
        };
        ret = i2c_transfer(client->adapter,msg,ARRAY_SIZE(msg));
        if(ret < 0) {
                printk("i2c_transfer read len error\n");
                return -EINVAL;
        }
        return len;
}
/*
static int i2c_sht31_write_command(struct i2c_client *client,unsigned char msb,unsigned char lsb)
{
	int ret;
	unsigned char txbuf[] = {msb,lsb};
	struct i2c_msg msg[] = {client->addr,0,2,txbuf};
	ret = i2c_transfer(client->adapter,msg,1);
	if(ret < 0) {
		printk("i2c_transfer write error\n");
		return -EINVAL;
	}
	return 0;
}
*/
//now we just support fot one shot read mode
static ssize_t sht31_outreg_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t sht31_reg_set(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	return 0;
}

static DEVICE_ATTR(sht31_reg, 0644 , sht31_outreg_show, sht31_reg_set);
static struct attribute *sht31_attrs[] = {
    &dev_attr_sht31_reg.attr,
    NULL
};
static struct attribute_group sht31_attr_group = {
    .name = "sht31_reg",
    .attrs = sht31_attrs,
};

// https://github.com/ControlEverythingCommunity/CE_ARDUINO_LIB/blob/master/SHT30/SHT30.cpp
unsigned char CRC8(const  unsigned char *idata, int len)
{
    /*
        Polynomial: 0x31 (x8 + x5 + x4 + 1)
        Initialization: 0xFF
        Final XOR: 0x00
        Example: CRC (0xBEEF) = 0x92
    */
    const unsigned char POLYNOMIAL = 0x31;
    unsigned char CRC = 0xFF;
    int j = 0,i = 0;
    for ( j = len; j; --j )
    {
        CRC ^= *idata++;
        for (i = 8; i; --i )
        {
            CRC = ( CRC & 0x80 )
            ? (CRC << 1) ^ POLYNOMIAL
            : (CRC << 1);
        }
    }
    return CRC;
}

static int sht31_read_measurement(struct sht31 *data, bool temp) {
	char ret;
	int i;
	unsigned char data_buf[6],checksum;
	struct i2c_client *client = data->client;
	//ONE_SHOT_CS and repeatily Medium
	ret = i2c_smbus_write_byte_data(client,MEASUREMENT_ONE_SHOT_CS_MSB,MEASUREMENT_ONE_SHOT_CS_LSB_M);
	//assum delay 7ms
	mdelay(7);
	ret = i2c_sht31_read_len(client,sizeof(data_buf),data_buf);
	if(temp) {
		if(data_buf[2] == CRC8(data_buf,2)) {
			data->temperature.val_msb = data_buf[0];
			data->temperature.val2_lsb = data_buf[1];
		}
		else {
			//error crc8 checksum
			data->temperature.val_msb = 0xff;
			data->temperature.val2_lsb = 0xff;
			return -EINVAL;
		}

	}
	else {
		if(data_buf[5] = CRC8(data_buf+3,2)) {
			data->humidity.val_msb = data_buf[3];
			data->humidity.val2_lsb = data_buf[4];

		}
		else {
			data->humidity.val_msb = 0xff;
                        data->humidity.val2_lsb = 0xff;
			return -EINVAL;
		}
	}	
	return 0;
}
static int sht31_read_raw(struct iio_dev *iio_dev,
                          const struct iio_chan_spec *chan,
                        int *val, int *val2, long mask)
{
	int ret;
	struct sht31 *data = iio_priv(iio_dev);
	int tmp;
	//use i2c protocal to read humidity or temp or humidity
	switch (mask) {
		case IIO_CHAN_INFO_PROCESSED:
			mutex_lock(&data->lock);
			ret = sht31_read_measurement(data, chan->type == IIO_TEMP);
			mutex_unlock(&data->lock);
			//temp
			if( chan->type == IIO_TEMP) {
				tmp = (data->temperature.val_msb << 8) | data->temperature.val2_lsb;
				//Degree c
				*val = -45 + 175*(tmp/65535);
				//Degree f
				//*val2 = -49.0 + 315.0*(temp/65535.0);
			}
			else if ( chan->type == IIO_HUMIDITYRELATIVE) {
				tmp = (data->humidity.val_msb << 8) | data->humidity.val2_lsb;
				*val = 100*(tmp/65535);
				//*val2 = 0;
			}
			else
				return -EINVAL;
			
			return IIO_VAL_INT_PLUS_MICRO;
		default:
			break;
	}

	return -EINVAL;
}
/*
static int sht31_write_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan,
			 int val, int val2, long mask) 
{
	return -EINVAL;
};
*/
static const struct iio_info sht31_iio_info = {
        .driver_module          = THIS_MODULE,
        .read_raw               = sht31_read_raw,
	//.write_raw		= sht31_write_raw,
};

static const struct iio_chan_spec sht31_chan_spec[] = {
        { 
		.type = IIO_TEMP,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
	},
        {
		.type = IIO_HUMIDITYRELATIVE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED), 
	}
};

static int sht31_dev_init(void)
{
	char res;
	printk("%s called\n", __func__);
	res = i2c_smbus_write_byte_data(sht31_client,(SOFT_RESET & 0xff00) >> 8,(SOFT_RESET & 0xff)); //soft reset
	mdelay(2);
	res = i2c_smbus_write_byte_data(sht31_client,(CLEAR_STATUS_REG & 0xff00) >> 8,(CLEAR_STATUS_REG & 0xff)); //clear status register
	return 0;
}

static int sht31_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	int ret;
	struct iio_dev *iio;
	struct sht31 *sht31;
	iio = iio_device_alloc(sizeof(*sht31));
	dev_dbg(&i2c->dev, "%s\n", __func__);
	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C))
                return -ENODEV;

	dev_info(&i2c->dev, "chip found, driver version " DRV_VERSION "\n");
	sht31_client = i2c;

	sht31 = iio_priv(iio);
	sht31->dev = &i2c->dev;
	sht31->client = i2c;
	i2c_set_clientdata(i2c, iio);
	mutex_init(&sht31->lock);
	iio->name = i2c->name;
	iio->dev.parent = &i2c->dev;
	iio->info = &sht31_iio_info;
	iio->modes = INDIO_DIRECT_MODE;
	iio->channels = sht31_chan_spec;
	iio->num_channels = ARRAY_SIZE(sht31_chan_spec);

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
    { "sht31", 0},
    {}
};
MODULE_DEVICE_TABLE(i2c, sht31_id);

static struct of_device_id sht31_of_match[] = {
        { .compatible = "sensirion,sht31"},
        { }
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
MODULE_DESCRIPTION("A i2c-sht31 driver for testing module ");
MODULE_VERSION(DRV_VERSION);
