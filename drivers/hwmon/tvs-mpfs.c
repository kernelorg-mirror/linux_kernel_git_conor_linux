// SPDX-License-Identifier: GPL-2.0+
/*
 * Author: Lars Randers <lranders@mail.dk>
 */

#include <linux/delay.h>
#include "linux/gpio/consumer.h"
#include "linux/interrupt.h"
#include <linux/bitfield.h>
#include <linux/err.h>
#include <linux/freezer.h>
#include <linux/hwmon.h>
#include <linux/io.h>
#include <linux/kthread.h>
#include <linux/math.h>
#include <linux/mfd/syscon.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#define MPFS_TVS_CTRL 0x08
#define MPFS_TVS_OUTPUT0 0x24
#define MPFS_TVS_OUTPUT1 0x28
#define MPFS_TVS_TRIGGER 0x2c

#define MPFS_TVS_CTRL_TEMP_VALID	BIT(19)
#define MPFS_TVS_CTRL_V2P5_VALID	BIT(18)
#define MPFS_TVS_CTRL_V1P8_VALID	BIT(17)
#define MPFS_TVS_CTRL_V1P05_VALID	BIT(16)

#define MPFS_TVS_CTRL_TEMP_ENABLE	BIT(3)
#define MPFS_TVS_CTRL_V2P5_ENABLE	BIT(2)
#define MPFS_TVS_CTRL_V1P8_ENABLE	BIT(1)
#define MPFS_TVS_CTRL_V1P05_ENABLE	BIT(0)
#define MPFS_TVS_CTRL_ENABLE_ALL	GENMASK(3, 0)

/*
 * For all of these the value in millivolts is stored in 16 bits, with an upper
 * sign bit and a lower 3 bits of decimal. These masks discard the sign bit and
 * decimal places, because if Linux is running these voltages cannot be negative
 * and so avoid having to convert to two's complement.
 */
#define MPFS_OUTPUT0_V1P8_MASK	GENMASK(30, 19)
#define MPFS_OUTPUT0_V1P05_MASK	GENMASK(14, 3)
#define MPFS_OUTPUT1_V2P5_MASK	GENMASK(14, 3)

/*
 * The register map claims that the temperature is stored in bits 31:16, but
 * application note "AN4682: PolarFire FPGA Temperature and Voltage Sensor"
 * says that 31 is reserved. Temperature is in kelvin, so what's probably a
 * sign bit has no value anyway. Same applies to the triggers that are
 * nominally 16 bits wide but are functionally 15.
 */
#define MPFS_OUTPUT1_TEMP_MASK GENMASK(30, 16)
#define MPFS_TRIGGER_MAX_MASK GENMASK(30, 16)
#define MPFS_TRIGGER_MIN_MASK GENMASK(14, 0)

#define MPFS_TVS_INTERVAL_MASK GENMASK(15, 8)
#define MPFS_TVS_INTERVAL_OFFSET 8
/* The interval register is in increments of 32 us */
#define MPFS_TVS_INTERVAL_SCALE 32
/* with 254 usable increments of 32 us available, 8 ms is the integer limit */
#define MPFS_TVS_INTERVAL_MAX_MS 8

/* 273.1875 in 11.4 fixed-point notation */
#define MPFS_TVS_K_TO_C 0x1113

enum mpfs_tvs_sensors {
	SENSOR_V1P05 = 0,
	SENSOR_V1P8,
	SENSOR_V2P5,
};

static const char * const mpfs_tvs_voltage_labels[] = { "1P05", "1P8", "2P5" };

struct mpfs_tvs {
	struct regmap *regmap;
	struct gpio_descs *clear_gpios;
	bool workaround;
	bool max_alarm;
	bool min_alarm;
};

static int mpfs_tvs_voltage_read(struct mpfs_tvs *data, u32 attr,
				 int channel, long *val)
{
	u32 tmp, control;

	if (attr != hwmon_in_input && attr != hwmon_in_enable)
		return -EOPNOTSUPP;

	regmap_read(data->regmap, MPFS_TVS_CTRL, &control);

	switch (channel) {
	case SENSOR_V2P5:
		if (attr == hwmon_in_enable) {
			*val = FIELD_GET(MPFS_TVS_CTRL_V2P5_ENABLE, control);
			break;
		}

		if (!(control & MPFS_TVS_CTRL_V2P5_VALID))
			return -ENODATA;

		regmap_read(data->regmap, MPFS_TVS_OUTPUT1, &tmp);
		*val = FIELD_GET(MPFS_OUTPUT1_V2P5_MASK, tmp);
		break;
	case SENSOR_V1P8:
		if (attr == hwmon_in_enable) {
			*val = FIELD_GET(MPFS_TVS_CTRL_V1P8_ENABLE, control);
			break;
		}

		if (!(control & MPFS_TVS_CTRL_V1P8_VALID))
			return -ENODATA;

		regmap_read(data->regmap, MPFS_TVS_OUTPUT0, &tmp);
		*val = FIELD_GET(MPFS_OUTPUT0_V1P8_MASK, tmp);
		break;
	case SENSOR_V1P05:
		if (attr == hwmon_in_enable) {
			*val = FIELD_GET(MPFS_TVS_CTRL_V1P05_ENABLE, control);
			break;
		}

		if (!(control & MPFS_TVS_CTRL_V1P05_VALID))
			return -ENODATA;

		regmap_read(data->regmap, MPFS_TVS_OUTPUT0, &tmp);
		*val = FIELD_GET(MPFS_OUTPUT0_V1P05_MASK, tmp);
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int mpfs_tvs_voltage_write(struct mpfs_tvs *data, u32 attr,
				  int channel, long val)
{
	u32 tmp;

	if (attr != hwmon_in_enable)
		return -EOPNOTSUPP;

	if (val > 1 || val < 0)
		return -EINVAL;

	switch (channel) {
	case SENSOR_V2P5:
		tmp = FIELD_PREP(MPFS_TVS_CTRL_V2P5_ENABLE, val);
		regmap_update_bits(data->regmap, MPFS_TVS_CTRL,
				   MPFS_TVS_CTRL_V2P5_ENABLE, tmp);
		break;
	case SENSOR_V1P8:
		tmp = FIELD_PREP(MPFS_TVS_CTRL_V1P8_ENABLE, val);
		regmap_update_bits(data->regmap, MPFS_TVS_CTRL,
				   MPFS_TVS_CTRL_V1P8_ENABLE, tmp);
		break;
	case SENSOR_V1P05:
		tmp = FIELD_PREP(MPFS_TVS_CTRL_V1P05_ENABLE, val);
		regmap_update_bits(data->regmap, MPFS_TVS_CTRL,
				   MPFS_TVS_CTRL_V1P05_ENABLE, tmp);
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int mpfs_tvs_temp_read(struct mpfs_tvs *data, u32 attr, long *val)
{
	u32 tmp, control;

	switch(attr) {
	case hwmon_temp_enable:
		regmap_read(data->regmap, MPFS_TVS_CTRL, &control);

		*val = FIELD_GET(MPFS_TVS_CTRL_TEMP_ENABLE, control);
		break;
	case hwmon_temp_input:
		regmap_read(data->regmap, MPFS_TVS_CTRL, &control);

		if (!(control & MPFS_TVS_CTRL_TEMP_VALID))
			return -ENODATA;

		regmap_read(data->regmap, MPFS_TVS_OUTPUT1, &tmp);
		*val = FIELD_GET(MPFS_OUTPUT1_TEMP_MASK, tmp);
		*val -= MPFS_TVS_K_TO_C;
		*val = (1000 * *val) >> 4; /* fixed point (11.4) to millidegrees */
		break;
	case hwmon_temp_max:
		regmap_read(data->regmap, MPFS_TVS_TRIGGER, &tmp);
		*val = FIELD_GET(MPFS_TRIGGER_MAX_MASK, tmp);
		*val -= MPFS_TVS_K_TO_C;
		*val = (1000 * *val) >> 4; /* fixed point (11.4) to millidegrees */
		break;
	case hwmon_temp_max_alarm:
		*val = data->max_alarm;
		break;
	case hwmon_temp_min:
		regmap_read(data->regmap, MPFS_TVS_TRIGGER, &tmp);
		*val = FIELD_GET(MPFS_TRIGGER_MIN_MASK, tmp);
		*val -= MPFS_TVS_K_TO_C;
		*val = (1000 * *val) >> 4; /* fixed point (11.4) to millidegrees */
		break;
	case hwmon_temp_min_alarm:
		*val = data->min_alarm;
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int mpfs_tvs_temp_write(struct mpfs_tvs *data, u32 attr, long val)
{
	u32 tmp;
	int temp;

	switch(attr) {
	case hwmon_temp_enable:
		if (val > 1 || val < 0)
			return -EINVAL;

		tmp = FIELD_PREP(MPFS_TVS_CTRL_TEMP_ENABLE, val);
		regmap_update_bits(data->regmap, MPFS_TVS_CTRL,
				   MPFS_TVS_CTRL_TEMP_ENABLE, tmp);
		break;
	case hwmon_temp_max:
		if (val < -273)
			return -EINVAL;

		temp = val << 4;
		temp /= 1000;
		temp += MPFS_TVS_K_TO_C;

		if (temp > GENMASK(14, 0))
			return -EINVAL;

		tmp = FIELD_PREP(MPFS_TRIGGER_MAX_MASK, temp);
		regmap_update_bits(data->regmap, MPFS_TVS_TRIGGER,
				   MPFS_TRIGGER_MAX_MASK, tmp);
		break;
	case hwmon_temp_min:
		if (val < -273)
			return -EINVAL;

		temp = val << 4;
		temp /= 1000;
		temp += MPFS_TVS_K_TO_C;

		if (temp > GENMASK(14, 0))
			return -EINVAL;

		tmp = FIELD_PREP(MPFS_TRIGGER_MIN_MASK, temp);
		regmap_update_bits(data->regmap, MPFS_TVS_TRIGGER,
				   MPFS_TRIGGER_MIN_MASK, tmp);
		break;
	default:
		return EOPNOTSUPP;
	};

	return 0;
}

static int mpfs_tvs_interval_read(struct mpfs_tvs *data, u32 attr, long *val)
{
	u32 tmp;

	if (attr != hwmon_chip_update_interval)
		return -EOPNOTSUPP;

	regmap_read(data->regmap, MPFS_TVS_CTRL, &tmp);
	*val = FIELD_GET(MPFS_TVS_INTERVAL_MASK, tmp);
	*val *= MPFS_TVS_INTERVAL_SCALE;
	*val = roundup(*val, 1000);
	*val /= 1000;

	return 0;
}

static int mpfs_tvs_interval_write(struct mpfs_tvs *data, u32 attr, long val)
{
	long temp = val;

	if (attr != hwmon_chip_update_interval)
		return -EOPNOTSUPP;

	temp = clamp(temp, 0, MPFS_TVS_INTERVAL_MAX_MS);

	temp *= 1000;
	temp /= MPFS_TVS_INTERVAL_SCALE;

	temp <<= MPFS_TVS_INTERVAL_OFFSET;
	regmap_update_bits(data->regmap, MPFS_TVS_CTRL,
			   MPFS_TVS_INTERVAL_MASK, temp);

	return 0;
}

static umode_t mpfs_tvs_is_visible(const void *d,
				   enum hwmon_sensor_types type,
				   u32 attr, int channel)
{
	struct mpfs_tvs *data = (struct mpfs_tvs *)d;

	if (type == hwmon_chip && attr == hwmon_chip_update_interval)
		return 0644;

	if (type == hwmon_temp) {
		switch (attr) {
		case hwmon_temp_max:
		case hwmon_temp_min:
			if (!data->workaround)
				return 0;
			fallthrough;
		case hwmon_temp_enable:
			return 0644;
		case hwmon_temp_max_alarm:
		case hwmon_temp_min_alarm:
			if (!data->workaround)
				return 0;
			fallthrough;
		case hwmon_temp_input:
		case hwmon_temp_label:
			return 0444;
		default:
			return 0;
		}
	}

	if (type == hwmon_in) {
		switch (attr) {
		case hwmon_in_enable:
			return 0644;
		case hwmon_in_input:
		case hwmon_in_label:
			return 0444;
		default:
			return 0;
		}
	}

	return 0;
}

static int mpfs_tvs_read(struct device *dev, enum hwmon_sensor_types type,
			 u32 attr, int channel, long *val)
{
	struct mpfs_tvs *data = dev_get_drvdata(dev);

	switch (type) {
	case hwmon_temp:
		return mpfs_tvs_temp_read(data, attr, val);
	case hwmon_in:
		return mpfs_tvs_voltage_read(data, attr, channel, val);
	case hwmon_chip:
		return mpfs_tvs_interval_read(data, attr, val);
	default:
		return -EOPNOTSUPP;
	}
}

static int mpfs_tvs_write(struct device *dev, enum hwmon_sensor_types type,
			  u32 attr, int channel, long val)
{
	struct mpfs_tvs *data = dev_get_drvdata(dev);

	switch (type) {
	case hwmon_temp:
		return mpfs_tvs_temp_write(data, attr, val);
	case hwmon_in:
		return mpfs_tvs_voltage_write(data, attr, channel, val);
	case hwmon_chip:
		return mpfs_tvs_interval_write(data, attr, val);
	default:
		return -EOPNOTSUPP;
	}
}

static int mpfs_tvs_read_labels(struct device *dev,
				enum hwmon_sensor_types type,
				u32 attr, int channel,
				const char **str)
{
	switch (type) {
	case hwmon_temp:
		*str = "Die Temp";
		return 0;
	case hwmon_in:
		*str = mpfs_tvs_voltage_labels[channel];
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static const struct hwmon_ops mpfs_tvs_ops = {
	.is_visible = mpfs_tvs_is_visible,
	.read_string = mpfs_tvs_read_labels,
	.read = mpfs_tvs_read,
	.write = mpfs_tvs_write,
};

static const struct hwmon_channel_info *mpfs_tvs_info[] = {
	HWMON_CHANNEL_INFO(chip,
			   HWMON_C_REGISTER_TZ | HWMON_C_UPDATE_INTERVAL),
	HWMON_CHANNEL_INFO(temp,
			   HWMON_T_INPUT | HWMON_T_LABEL | HWMON_T_ENABLE |
			   HWMON_T_MIN | HWMON_T_MAX | HWMON_T_MIN_ALARM |
			   HWMON_T_MAX_ALARM),
	HWMON_CHANNEL_INFO(in,
			   HWMON_I_INPUT | HWMON_I_LABEL | HWMON_I_ENABLE,
			   HWMON_I_INPUT | HWMON_I_LABEL | HWMON_I_ENABLE,
			   HWMON_I_INPUT | HWMON_I_LABEL | HWMON_I_ENABLE),
	NULL
};

static const struct hwmon_chip_info mpfs_tvs_chip_info = {
	.ops = &mpfs_tvs_ops,
	.info = mpfs_tvs_info,
};

static irqreturn_t mpfs_tvs_high_rising_handler(int irq, void *d)
{
	struct mpfs_tvs *data = d;
	static int count;
	u32 tmp;
	int val;

	count++;

	pr_info("rising count %d\n", count);
	regmap_read(data->regmap, MPFS_TVS_OUTPUT1, &tmp);
	val = FIELD_GET(MPFS_OUTPUT1_TEMP_MASK, tmp);
	val -= MPFS_TVS_K_TO_C;
	val = (1000 * val) >> 4; /* fixed point (11.4) to millidegrees */
	pr_info("temp: %d\n", val);

	gpiod_set_value(data->clear_gpios->desc[0], 1);
	udelay(100000); //TODO what's the correct delay? (Ask Brian)
	gpiod_set_value(data->clear_gpios->desc[0], 0);

	//TODO do I want to check the level bit before setting this?
	data->max_alarm = true;

	return IRQ_HANDLED;
}

static irqreturn_t mpfs_tvs_high_falling_handler(int irq, void *d)
{
	struct mpfs_tvs *data = d;
	static int count;
	u32 tmp;
	int val;

	count++;

	pr_info("falling count %d\n", count);
	regmap_read(data->regmap, MPFS_TVS_OUTPUT1, &tmp);
	val = FIELD_GET(MPFS_OUTPUT1_TEMP_MASK, tmp);
	val -= MPFS_TVS_K_TO_C;
	val = (1000 * val) >> 4; /* fixed point (11.4) to millidegrees */
	pr_info("temp: %d\n", val);

	gpiod_set_value(data->clear_gpios->desc[1], 1);
	udelay(100000);
	gpiod_set_value(data->clear_gpios->desc[1], 0);

	data->max_alarm = false;

	return IRQ_HANDLED;
}

static irqreturn_t mpfs_tvs_low_rising_handler(int irq, void *d)
{
	struct mpfs_tvs *data = d;
	static int count;
	u32 tmp;
	int val;

	count++;

	pr_info("low rising count %d\n", count);
	regmap_read(data->regmap, MPFS_TVS_OUTPUT1, &tmp);
	val = FIELD_GET(MPFS_OUTPUT1_TEMP_MASK, tmp);
	val -= MPFS_TVS_K_TO_C;
	val = (1000 * val) >> 4; /* fixed point (11.4) to millidegrees */
	pr_info("temp: %d\n", val);

	gpiod_set_value(data->clear_gpios->desc[2], 1);
	udelay(100000);
	gpiod_set_value(data->clear_gpios->desc[2], 0);

	//TODO do I want to check the level bit before setting this?
	data->min_alarm = true;

	return IRQ_HANDLED;
}

static irqreturn_t mpfs_tvs_low_falling_handler(int irq, void *d)
{
	struct mpfs_tvs *data = d;
	static int count;
	u32 tmp;
	int val;

	count++;

	pr_info("low falling count %d\n", count);
	regmap_read(data->regmap, MPFS_TVS_OUTPUT1, &tmp);
	val = FIELD_GET(MPFS_OUTPUT1_TEMP_MASK, tmp);
	val -= MPFS_TVS_K_TO_C;
	val = (1000 * val) >> 4; /* fixed point (11.4) to millidegrees */
	pr_info("temp: %d\n", val);

	gpiod_set_value(data->clear_gpios->desc[3], 1);
	udelay(100000);
	gpiod_set_value(data->clear_gpios->desc[3], 0);

	data->min_alarm = false;

	return IRQ_HANDLED;
}

static int mpfs_tvs_workaround_probe(struct platform_device *pdev, struct mpfs_tvs *data)
{
	int high_rising_irq, high_falling_irq, low_rising_irq, low_falling_irq;
	int ret;

	if (!of_property_present(pdev->dev.parent->of_node, "interrupt-clear-gpios"))
		return 0;

	data->workaround = true;

	//TODO hack in terms of the requesting device
	data->clear_gpios = devm_gpiod_get_array(pdev->dev.parent, "interrupt-clear",
						     GPIOD_OUT_HIGH);
	ret = PTR_ERR_OR_ZERO(data->clear_gpios);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "Failed to request clear gpios\n");
	if (data->clear_gpios->ndescs < 4)
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "Minimum of 4 clear gpios not provided\n");

	high_rising_irq = of_irq_get(pdev->dev.parent->of_node, 0);
	if (high_rising_irq < 0) {
		dev_err(&pdev->dev, "Failed to get high rising irq\n");
		return high_rising_irq;
	}

	ret = devm_request_irq(&pdev->dev, high_rising_irq, mpfs_tvs_high_rising_handler,
			       0, NULL, data);
	if (ret)
		return ret;

	high_falling_irq = of_irq_get(pdev->dev.parent->of_node, 1);
	if (high_falling_irq < 0) {
		dev_err(&pdev->dev, "Failed to get high falling irq\n");
		return high_falling_irq;
	}

	ret = devm_request_irq(&pdev->dev, high_falling_irq, mpfs_tvs_high_falling_handler,
			       0, NULL, data);
	if (ret)
		return ret;

	low_rising_irq = of_irq_get(pdev->dev.parent->of_node, 2);
	if (low_rising_irq < 0) {
		dev_err(&pdev->dev, "Failed to get low rising irq\n");
		return low_rising_irq;
	}

	ret = devm_request_irq(&pdev->dev, low_rising_irq, mpfs_tvs_low_rising_handler,
			       0, NULL, data);
	if (ret)
		return ret;

	low_falling_irq = of_irq_get(pdev->dev.parent->of_node, 3);
	if (low_falling_irq < 0) {
		dev_err(&pdev->dev, "Failed to get low falling irq\n");
		return low_falling_irq;
	}

	ret = devm_request_irq(&pdev->dev, low_falling_irq, mpfs_tvs_low_falling_handler,
			       0, NULL, data);

	return ret;
}

static int mpfs_tvs_probe(struct platform_device *pdev)
{
	struct device *hwmon_dev;
	struct mpfs_tvs *data;
	int ret;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->regmap = device_node_to_regmap(pdev->dev.parent->of_node);
	if (IS_ERR(data->regmap))
		return dev_err_probe(&pdev->dev, PTR_ERR(data->regmap),
				     "Failed to find syscon regmap\n");

	ret = mpfs_tvs_workaround_probe(pdev, data);
	if (ret)
		return ret;

	/*
	 * It's an MMIO regmap with no resources, there's nothing that can fail
	 * and return an error
	 */
	regmap_write(data->regmap, MPFS_TVS_CTRL, MPFS_TVS_CTRL_ENABLE_ALL);

	hwmon_dev = devm_hwmon_device_register_with_info(&pdev->dev, "mpfs_tvs",
							 data,
							 &mpfs_tvs_chip_info,
							 NULL);
	if (IS_ERR(hwmon_dev))
		return dev_err_probe(&pdev->dev, PTR_ERR(hwmon_dev),
				     "hwmon device registration failed.\n");

	return 0;
}

static struct platform_driver mpfs_tvs_driver = {
	.probe = mpfs_tvs_probe,
	.driver = {
		.name = "mpfs-tvs",
	},
};
module_platform_driver(mpfs_tvs_driver);

MODULE_AUTHOR("Lars Randers <lranders@mail.dk>");
MODULE_DESCRIPTION("PolarFire SoC temperature & voltage sensor driver");
MODULE_LICENSE("GPL");
