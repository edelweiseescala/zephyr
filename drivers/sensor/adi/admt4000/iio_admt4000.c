/*
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief IIO wrapper for the ADMT4000 True Power-On Multiturn Sensor.
 *
 * Exposes the ADMT4000 Zephyr sensor driver to the libiio IIOD server as an
 * IIO device, mirroring the channel and attribute surface of the no-OS
 * ADMT4000 IIO driver (drivers/position/admt4000/iio_admt4000.c).
 *
 * Value representation
 * --------------------
 * The Zephyr sensor API hands back values already converted to SI units via
 * struct sensor_value (val1 = integer part, val2 = fractional part in
 * micro-units). IIO expresses a reading as:
 *
 *     processed = raw * scale
 *
 * so every channel here publishes:
 *
 *     raw   - the reading in micro-units (val1 * 1000000 + val2)
 *     scale - "0.000001"
 *
 * Micro-units are used rather than the milli-units of the generic
 * "iio,sensor" wrapper because the CORDIC sine/cosine channels are
 * normalized to [-1, 1) and would otherwise be quantized to three digits.
 * All ADMT4000 channel ranges fit comfortably in int32_t at micro
 * resolution (the widest is angle, 360 * 10^6 = 3.6e8).
 *
 * Not supported
 * -------------
 * The Zephyr libiio backend (modules/lib/libiio/zephyr/backend.c) implements
 * only create/read_attr/write_attr/get_trigger - it has no open_buffer or
 * readbuf hook. Buffered/streaming capture, and therefore the no-OS
 * trigger_handler and submit paths, have no counterpart here. Readings are
 * obtained through the per-channel "raw" attributes.
 */

#define DT_DRV_COMPAT adi_admt4000_iio

#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/admt4000.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <iio/iio-backend.h>
#include <iio_device.h>

LOG_MODULE_REGISTER(admt4000_iio, CONFIG_LIBIIO_LOG_LEVEL);

/* Longest value this driver emits is the 8-field harmonics string. */
#define ADMT4000_IIO_RAW_LEN      12
#define ADMT4000_IIO_HARMONICS_LEN 48

/*
 * Channel scale. Raw readings are micro-units, so processed = raw * 1e-6.
 */
static const char admt4000_iio_scale_str[] = "0.000001";

/*
 * Harmonic magnitude scale = ADMT4000_HMAG_RES / ADMT4000_CORDIC_SCALER
 *                          = 54930 / 6072000 = 0.009046
 * Harmonic phase scale     = ADMT4000_HPHA_RES expressed in 10^-7 units
 *                          = 878910 * 10^-7 = 0.087891
 */
static const char admt4000_iio_hmag_scale_str[] = "0.009046";
static const char admt4000_iio_hpha_scale_str[] = "0.087891";

static const char *const raw_name = "raw";
static const char *const scale_name = "scale";
static const char *const offset_name = "offset";

/* Channel table ------------------------------------------------------- */

struct admt4000_iio_channel {
	/** IIO channel id; the leading token selects the IIO channel type. */
	const char *id;
	/** Human readable channel name, as reported by iio_channel_get_name(). */
	const char *name;
	/** Backing Zephyr sensor channel. */
	enum sensor_channel chan;
	bool has_scale;
	bool has_offset;
};

/*
 * Channel ids are chosen so that libiio infers the same IIO channel types the
 * no-OS driver declares explicitly:
 *   "rot"     -> IIO_ROT      "angl"    -> IIO_ANGL
 *   "temp"    -> IIO_TEMP     "count"   -> IIO_COUNT
 *   "voltage" -> IIO_VOLTAGE
 */
static const struct admt4000_iio_channel admt4000_iio_channels[] = {
	{"rot0", "turns", SENSOR_CHAN_ROTATION, true, false},
	{"angl0", "angle", SENSOR_CHAN_ADMT4000_ANGLE, true, false},
	{"temp0", "temp", SENSOR_CHAN_AMBIENT_TEMP, true, true},
	{"count1", "cosine", SENSOR_CHAN_ADMT4000_COS, false, false},
	{"count2", "sine", SENSOR_CHAN_ADMT4000_SIN, false, false},
	{"voltage0", "radius", SENSOR_CHAN_ADMT4000_RADIUS, true, false},
};

#define ADMT4000_IIO_NUM_CHANNELS ARRAY_SIZE(admt4000_iio_channels)

/* Device attribute table ---------------------------------------------- */

enum admt4000_iio_dev_attr {
	ADMT4000_IIO_ATTR_CONVERSION_MODE,
	ADMT4000_IIO_ATTR_CONV_SYNC_MODE,
	ADMT4000_IIO_ATTR_CONV_SYNC_MODE_AVAIL,
	ADMT4000_IIO_ATTR_ANGLE_FILTER_ENABLE,
	ADMT4000_IIO_ATTR_H8_CORR_SRC,
	ADMT4000_IIO_ATTR_H8_CORR_SRC_AVAIL,
	ADMT4000_IIO_ATTR_ECK_TYPE,
	ADMT4000_IIO_ATTR_ECK_TYPE_AVAIL,
	ADMT4000_IIO_ATTR_HARMONICS,
	ADMT4000_IIO_ATTR_HARMONICS_MAG_SCALE,
	ADMT4000_IIO_ATTR_HARMONICS_PH_SCALE,
	ADMT4000_IIO_ATTR_COIL_RESET,
};

static const char *const admt4000_iio_dev_attr_names[] = {
	[ADMT4000_IIO_ATTR_CONVERSION_MODE] = "conversion_mode",
	[ADMT4000_IIO_ATTR_CONV_SYNC_MODE] = "conv_sync_mode",
	[ADMT4000_IIO_ATTR_CONV_SYNC_MODE_AVAIL] = "conv_sync_mode_available",
	[ADMT4000_IIO_ATTR_ANGLE_FILTER_ENABLE] = "angle_filter_enable",
	[ADMT4000_IIO_ATTR_H8_CORR_SRC] = "h8_corr_src",
	[ADMT4000_IIO_ATTR_H8_CORR_SRC_AVAIL] = "h8_corr_src_available",
	[ADMT4000_IIO_ATTR_ECK_TYPE] = "eck_type",
	[ADMT4000_IIO_ATTR_ECK_TYPE_AVAIL] = "eck_type_available",
	[ADMT4000_IIO_ATTR_HARMONICS] = "harmonics",
	[ADMT4000_IIO_ATTR_HARMONICS_MAG_SCALE] = "harmonics_magnitude_scale",
	[ADMT4000_IIO_ATTR_HARMONICS_PH_SCALE] = "harmonics_phase_scale",
	[ADMT4000_IIO_ATTR_COIL_RESET] = "coil_reset",
};

/*
 * Enumerated attribute values. The underlying enums are sparse
 * (ADMT4000_START_EDGE == 3), so value/string pairs are used rather than a
 * densely indexed array.
 */
struct admt4000_iio_enum_item {
	int value;
	const char *name;
};

static const struct admt4000_iio_enum_item admt4000_iio_conv_sync_mode[] = {
	{ADMT4000_SEQ_CTRL, "seq_ctrl"},
	{ADMT4000_START_EDGE, "start_edge"},
};

static const struct admt4000_iio_enum_item admt4000_iio_h8_corr_src[] = {
	{ADMT4000_FACTORY, "factory"},
	{ADMT4000_USER, "user"},
};

static const struct admt4000_iio_enum_item admt4000_iio_eck_type[] = {
	{ADMT4000_ALWAYS_FLAG, "always_flag"},
	{ADMT4000_VALID, "valid"},
	{ADMT4000_DISABLE_CHECK, "disable_check"},
};

/* Harmonic attributes, in the interleaved order used by the "harmonics"
 * attribute of the no-OS driver: h1mag h1ph h2mag h2ph h3mag h3ph h8mag h8ph.
 */
static const enum sensor_attribute admt4000_iio_harmonic_attrs[] = {
	SENSOR_ATTR_ADMT4000_HARMONIC_MAG_1, SENSOR_ATTR_ADMT4000_HARMONIC_PH_1,
	SENSOR_ATTR_ADMT4000_HARMONIC_MAG_2, SENSOR_ATTR_ADMT4000_HARMONIC_PH_2,
	SENSOR_ATTR_ADMT4000_HARMONIC_MAG_3, SENSOR_ATTR_ADMT4000_HARMONIC_PH_3,
	SENSOR_ATTR_ADMT4000_HARMONIC_MAG_8, SENSOR_ATTR_ADMT4000_HARMONIC_PH_8,
};

/* Driver state --------------------------------------------------------- */

struct admt4000_iio_config {
	const struct device *sensor_dev;
};

/*
 * One sensor_sample_fetch(SENSOR_CHAN_ALL) per sample round, then
 * sensor_channel_get() for every channel into a cache. The ADMT4000 fetch
 * triggers a conversion and walks several register pages, so re-fetching per
 * channel would be both slow and inconsistent across channels.
 */
struct admt4000_iio_data {
	struct sensor_value cache[ADMT4000_IIO_NUM_CHANNELS];
	bool fetched;
};

/* Helpers -------------------------------------------------------------- */

static int admt4000_iio_fetch_all(const struct device *dev)
{
	const struct admt4000_iio_config *config = dev->config;
	struct admt4000_iio_data *data = dev->data;
	int ret;

	if (data->fetched) {
		return 0;
	}

	ret = sensor_sample_fetch(config->sensor_dev);
	if (ret < 0) {
		LOG_ERR("Sample fetch failed: %d", ret);
		return ret;
	}

	for (size_t i = 0; i < ADMT4000_IIO_NUM_CHANNELS; i++) {
		ret = sensor_channel_get(config->sensor_dev,
					 admt4000_iio_channels[i].chan,
					 &data->cache[i]);
		if (ret < 0) {
			LOG_ERR("Channel %s get failed: %d",
				admt4000_iio_channels[i].name, ret);
			data->cache[i].val1 = 0;
			data->cache[i].val2 = 0;
		}
	}

	data->fetched = true;

	return 0;
}

static int admt4000_iio_read_channel_raw(const struct device *dev, int index,
					 char *dst, size_t len)
{
	struct admt4000_iio_data *data = dev->data;
	int64_t micro;
	int ret;

	if (len < ADMT4000_IIO_RAW_LEN) {
		LOG_ERR("Buffer too small for raw value");
		return -ENOMEM;
	}

	/*
	 * Reading channel 0 marks the start of a new sample round. libiio
	 * walks the channels in index order, so the remaining channels are
	 * served from the cache and stay coherent with each other.
	 */
	if (index == 0) {
		data->fetched = false;
	}

	ret = admt4000_iio_fetch_all(dev);
	if (ret < 0) {
		return ret;
	}

	micro = (int64_t)data->cache[index].val1 * 1000000 + data->cache[index].val2;

	/*
	 * Every ADMT4000 channel range fits in int32_t at micro resolution
	 * (widest is angle, 3.6e8). Clamp defensively so a bogus reading
	 * cannot wrap, and keep the print 32-bit so no 64-bit cbprintf
	 * support is required.
	 */
	micro = CLAMP(micro, (int64_t)INT32_MIN, (int64_t)INT32_MAX);

	return snprintk(dst, len, "%d", (int32_t)micro) + 1;
}

static int admt4000_iio_lookup_enum_name(const struct admt4000_iio_enum_item *items,
					 size_t count, int value, const char **name)
{
	for (size_t i = 0; i < count; i++) {
		if (items[i].value == value) {
			*name = items[i].name;
			return 0;
		}
	}

	return -EINVAL;
}

static int admt4000_iio_lookup_enum_value(const struct admt4000_iio_enum_item *items,
					  size_t count, const char *name, int *value)
{
	for (size_t i = 0; i < count; i++) {
		if (strcmp(items[i].name, name) == 0) {
			*value = items[i].value;
			return 0;
		}
	}

	return -EINVAL;
}

static int admt4000_iio_read_enum(const struct device *dev,
				  enum sensor_attribute attr,
				  const struct admt4000_iio_enum_item *items,
				  size_t count, char *dst, size_t len)
{
	const struct admt4000_iio_config *config = dev->config;
	struct sensor_value val;
	const char *name;
	int ret;

	ret = sensor_attr_get(config->sensor_dev, SENSOR_CHAN_ALL, attr, &val);
	if (ret < 0) {
		return ret;
	}

	ret = admt4000_iio_lookup_enum_name(items, count, val.val1, &name);
	if (ret < 0) {
		LOG_ERR("Device reported unknown value %d for attr %d", val.val1, attr);
		return ret;
	}

	return snprintk(dst, len, "%s", name) + 1;
}

static int admt4000_iio_write_enum(const struct device *dev,
				   enum sensor_attribute attr,
				   const struct admt4000_iio_enum_item *items,
				   size_t count, const char *src, size_t len)
{
	const struct admt4000_iio_config *config = dev->config;
	struct sensor_value val = {0};
	int value;
	int ret;

	ret = admt4000_iio_lookup_enum_value(items, count, src, &value);
	if (ret < 0) {
		LOG_ERR("Invalid value '%s' for attr %d", src, attr);
		return ret;
	}

	val.val1 = value;

	ret = sensor_attr_set(config->sensor_dev, SENSOR_CHAN_ALL, attr, &val);
	if (ret < 0) {
		return ret;
	}

	return (int)len;
}

static int admt4000_iio_read_enum_avail(const struct admt4000_iio_enum_item *items,
					size_t count, char *dst, size_t len)
{
	int written = 0;

	for (size_t i = 0; i < count; i++) {
		int ret = snprintk(dst + written, len - written, "%s%s",
				   i ? " " : "", items[i].name);

		if (ret < 0 || ret >= (int)(len - written)) {
			return -ENOMEM;
		}

		written += ret;
	}

	return written + 1;
}

static int admt4000_iio_read_bool(const struct device *dev,
				  enum sensor_attribute attr, char *dst, size_t len)
{
	const struct admt4000_iio_config *config = dev->config;
	struct sensor_value val;
	int ret;

	ret = sensor_attr_get(config->sensor_dev, SENSOR_CHAN_ALL, attr, &val);
	if (ret < 0) {
		return ret;
	}

	return snprintk(dst, len, "%d", val.val1 ? 1 : 0) + 1;
}

static int admt4000_iio_write_bool(const struct device *dev,
				   enum sensor_attribute attr, const char *src,
				   size_t len)
{
	const struct admt4000_iio_config *config = dev->config;
	struct sensor_value val = {0};
	char *end;
	long parsed;
	int ret;

	parsed = strtol(src, &end, 10);
	if (end == src || (parsed != 0 && parsed != 1)) {
		LOG_ERR("Invalid boolean value '%s'", src);
		return -EINVAL;
	}

	val.val1 = (int32_t)parsed;

	ret = sensor_attr_set(config->sensor_dev, SENSOR_CHAN_ALL, attr, &val);
	if (ret < 0) {
		return ret;
	}

	return (int)len;
}

/*
 * Harmonic correction registers, as eight raw register values in the order
 * h1mag h1ph h2mag h2ph h3mag h3ph h8mag h8ph. This matches the format of the
 * no-OS "harmonics" attribute.
 *
 * Read-only: the ADMT4000 sensor driver's attr_get path returns raw register
 * values, but its attr_set path (admt4000_set_converted_hmag_config /
 * admt4000_set_converted_hphase_config) expects pre-scaled "converted" units
 * and is not the inverse of the getter, so a raw value written back through
 * it would not round-trip.
 */
static int admt4000_iio_read_harmonics(const struct device *dev, char *dst, size_t len)
{
	const struct admt4000_iio_config *config = dev->config;
	int written = 0;

	if (len < ADMT4000_IIO_HARMONICS_LEN) {
		LOG_ERR("Buffer too small for harmonics");
		return -ENOMEM;
	}

	for (size_t i = 0; i < ARRAY_SIZE(admt4000_iio_harmonic_attrs); i++) {
		struct sensor_value val;
		int ret;

		ret = sensor_attr_get(config->sensor_dev, SENSOR_CHAN_ALL,
				      admt4000_iio_harmonic_attrs[i], &val);
		if (ret < 0) {
			return ret;
		}

		ret = snprintk(dst + written, len - written, "%s%u",
			       i ? " " : "", (unsigned int)val.val1);
		if (ret < 0 || ret >= (int)(len - written)) {
			return -ENOMEM;
		}

		written += ret;
	}

	return written + 1;
}

static int admt4000_iio_write_coil_reset(const struct device *dev, size_t len)
{
	const struct admt4000_iio_config *config = dev->config;
	struct sensor_value val = {0};
	int ret;

	ret = sensor_attr_set(config->sensor_dev, SENSOR_CHAN_ALL,
			      SENSOR_ATTR_ADMT4000_COIL_RESET, &val);
	if (ret < 0) {
		return ret;
	}

	return (int)len;
}

/* IIO device API ------------------------------------------------------- */

static int admt4000_iio_add_channels(const struct device *dev,
				     struct iio_device *iio_device)
{
	ARG_UNUSED(dev);

	for (size_t index = 0; index < ADMT4000_IIO_NUM_CHANNELS; index++) {
		const struct admt4000_iio_channel *chan = &admt4000_iio_channels[index];
		struct iio_channel *iio_channel;

		/*
		 * Raw readings are micro-units, so 32 bits are required (the
		 * widest is angle at 3.6e8). Signed throughout to keep the
		 * format uniform across channels.
		 */
		const struct iio_data_format fmt = {
			.length = 32,
			.bits = 32,
			.is_signed = true,
			.with_scale = true,
			.scale = 0.000001,
			.is_be = false,
		};

		iio_channel = iio_device_add_channel(iio_device, index, chan->id,
						     chan->name, NULL, false, true, &fmt);
		if (iio_err(iio_channel)) {
			LOG_ERR("Could not add channel %s", chan->id);
			return -EINVAL;
		}

		iio_channel_set_pdata(iio_channel, (struct iio_channel_pdata *)(intptr_t)index);

		if (iio_channel_add_attr(iio_channel, raw_name, IIO_ATTR_TYPE_CHANNEL, NULL)) {
			LOG_ERR("Could not add %s attr %s", chan->id, raw_name);
			return -EINVAL;
		}

		if (chan->has_scale &&
		    iio_channel_add_attr(iio_channel, scale_name, IIO_ATTR_TYPE_CHANNEL, NULL)) {
			LOG_ERR("Could not add %s attr %s", chan->id, scale_name);
			return -EINVAL;
		}

		if (chan->has_offset &&
		    iio_channel_add_attr(iio_channel, offset_name, IIO_ATTR_TYPE_CHANNEL, NULL)) {
			LOG_ERR("Could not add %s attr %s", chan->id, offset_name);
			return -EINVAL;
		}
	}

	for (size_t i = 0; i < ARRAY_SIZE(admt4000_iio_dev_attr_names); i++) {
		if (iio_device_add_attr(iio_device, admt4000_iio_dev_attr_names[i],
					IIO_ATTR_TYPE_DEVICE)) {
			LOG_ERR("Could not add device attr %s",
				admt4000_iio_dev_attr_names[i]);
			return -EINVAL;
		}
	}

	return 0;
}

static int admt4000_iio_read_channel_attr(const struct device *dev,
					  const struct iio_attr *attr,
					  char *dst, size_t len)
{
	int index = (int)(intptr_t)iio_channel_get_pdata(attr->iio.chn);

	if (index < 0 || index >= (int)ADMT4000_IIO_NUM_CHANNELS) {
		LOG_ERR("Invalid channel index %d", index);
		return -EINVAL;
	}

	if (strcmp(attr->name, raw_name) == 0) {
		return admt4000_iio_read_channel_raw(dev, index, dst, len);
	}

	if (strcmp(attr->name, scale_name) == 0) {
		return snprintk(dst, len, "%s", admt4000_iio_scale_str) + 1;
	}

	if (strcmp(attr->name, offset_name) == 0) {
		/*
		 * The sensor API already returns absolute temperature, so the
		 * IIO offset is zero. Present for parity with the no-OS
		 * driver's temperature channel.
		 */
		return snprintk(dst, len, "0") + 1;
	}

	LOG_ERR("Invalid channel attr: %s", attr->name);

	return -EINVAL;
}

static int admt4000_iio_read_device_attr(const struct device *dev,
					 const struct iio_attr *attr,
					 char *dst, size_t len)
{
	if (strcmp(attr->name, admt4000_iio_dev_attr_names[ADMT4000_IIO_ATTR_CONVERSION_MODE]) == 0) {
		return admt4000_iio_read_bool(dev, SENSOR_ATTR_ADMT4000_CONV_MODE, dst, len);
	}

	if (strcmp(attr->name, admt4000_iio_dev_attr_names[ADMT4000_IIO_ATTR_ANGLE_FILTER_ENABLE]) == 0) {
		return admt4000_iio_read_bool(dev, SENSOR_ATTR_ADMT4000_FILTER_ENABLE, dst, len);
	}

	if (strcmp(attr->name, admt4000_iio_dev_attr_names[ADMT4000_IIO_ATTR_CONV_SYNC_MODE]) == 0) {
		return admt4000_iio_read_enum(dev, SENSOR_ATTR_ADMT4000_SYNC_MODE,
					      admt4000_iio_conv_sync_mode,
					      ARRAY_SIZE(admt4000_iio_conv_sync_mode),
					      dst, len);
	}

	if (strcmp(attr->name, admt4000_iio_dev_attr_names[ADMT4000_IIO_ATTR_CONV_SYNC_MODE_AVAIL]) == 0) {
		return admt4000_iio_read_enum_avail(admt4000_iio_conv_sync_mode,
						    ARRAY_SIZE(admt4000_iio_conv_sync_mode),
						    dst, len);
	}

	if (strcmp(attr->name, admt4000_iio_dev_attr_names[ADMT4000_IIO_ATTR_H8_CORR_SRC]) == 0) {
		return admt4000_iio_read_enum(dev, SENSOR_ATTR_ADMT4000_H8_CORR_SRC,
					      admt4000_iio_h8_corr_src,
					      ARRAY_SIZE(admt4000_iio_h8_corr_src),
					      dst, len);
	}

	if (strcmp(attr->name, admt4000_iio_dev_attr_names[ADMT4000_IIO_ATTR_H8_CORR_SRC_AVAIL]) == 0) {
		return admt4000_iio_read_enum_avail(admt4000_iio_h8_corr_src,
						    ARRAY_SIZE(admt4000_iio_h8_corr_src),
						    dst, len);
	}

	if (strcmp(attr->name, admt4000_iio_dev_attr_names[ADMT4000_IIO_ATTR_ECK_TYPE]) == 0) {
		return admt4000_iio_read_enum(dev, SENSOR_ATTR_ADMT4000_ECK_TYPE,
					      admt4000_iio_eck_type,
					      ARRAY_SIZE(admt4000_iio_eck_type),
					      dst, len);
	}

	if (strcmp(attr->name, admt4000_iio_dev_attr_names[ADMT4000_IIO_ATTR_ECK_TYPE_AVAIL]) == 0) {
		return admt4000_iio_read_enum_avail(admt4000_iio_eck_type,
						    ARRAY_SIZE(admt4000_iio_eck_type),
						    dst, len);
	}

	if (strcmp(attr->name, admt4000_iio_dev_attr_names[ADMT4000_IIO_ATTR_HARMONICS]) == 0) {
		return admt4000_iio_read_harmonics(dev, dst, len);
	}

	if (strcmp(attr->name, admt4000_iio_dev_attr_names[ADMT4000_IIO_ATTR_HARMONICS_MAG_SCALE]) == 0) {
		return snprintk(dst, len, "%s", admt4000_iio_hmag_scale_str) + 1;
	}

	if (strcmp(attr->name, admt4000_iio_dev_attr_names[ADMT4000_IIO_ATTR_HARMONICS_PH_SCALE]) == 0) {
		return snprintk(dst, len, "%s", admt4000_iio_hpha_scale_str) + 1;
	}

	if (strcmp(attr->name, admt4000_iio_dev_attr_names[ADMT4000_IIO_ATTR_COIL_RESET]) == 0) {
		/* Write-only. */
		return -EACCES;
	}

	LOG_ERR("Invalid device attr: %s", attr->name);

	return -EINVAL;
}

static int admt4000_iio_read_attr(const struct device *dev,
				  const struct iio_device *iio_device,
				  const struct iio_attr *attr, char *dst, size_t len)
{
	struct admt4000_iio_data *data = dev->data;

	ARG_UNUSED(iio_device);

	switch (attr->type) {
	case IIO_ATTR_TYPE_CHANNEL:
		return admt4000_iio_read_channel_attr(dev, attr, dst, len);

	case IIO_ATTR_TYPE_DEVICE:
		return admt4000_iio_read_device_attr(dev, attr, dst, len);

	case IIO_ATTR_TYPE_BUFFER:
		/* Invalidate the cache so the next sample round re-fetches. */
		data->fetched = false;
		dst[0] = '\0';
		return 1;

	default:
		break;
	}

	LOG_ERR("Invalid attr type %d", attr->type);

	return -EINVAL;
}

static int admt4000_iio_write_attr(const struct device *dev,
				   const struct iio_device *iio_device,
				   const struct iio_attr *attr, const char *src,
				   size_t len)
{
	ARG_UNUSED(iio_device);

	if (attr->type != IIO_ATTR_TYPE_DEVICE) {
		/* All channel attributes are read-only. */
		return -EACCES;
	}

	if (strcmp(attr->name, admt4000_iio_dev_attr_names[ADMT4000_IIO_ATTR_CONVERSION_MODE]) == 0) {
		return admt4000_iio_write_bool(dev, SENSOR_ATTR_ADMT4000_CONV_MODE, src, len);
	}

	if (strcmp(attr->name, admt4000_iio_dev_attr_names[ADMT4000_IIO_ATTR_ANGLE_FILTER_ENABLE]) == 0) {
		return admt4000_iio_write_bool(dev, SENSOR_ATTR_ADMT4000_FILTER_ENABLE, src, len);
	}

	if (strcmp(attr->name, admt4000_iio_dev_attr_names[ADMT4000_IIO_ATTR_CONV_SYNC_MODE]) == 0) {
		return admt4000_iio_write_enum(dev, SENSOR_ATTR_ADMT4000_SYNC_MODE,
					       admt4000_iio_conv_sync_mode,
					       ARRAY_SIZE(admt4000_iio_conv_sync_mode),
					       src, len);
	}

	if (strcmp(attr->name, admt4000_iio_dev_attr_names[ADMT4000_IIO_ATTR_H8_CORR_SRC]) == 0) {
		return admt4000_iio_write_enum(dev, SENSOR_ATTR_ADMT4000_H8_CORR_SRC,
					       admt4000_iio_h8_corr_src,
					       ARRAY_SIZE(admt4000_iio_h8_corr_src),
					       src, len);
	}

	if (strcmp(attr->name, admt4000_iio_dev_attr_names[ADMT4000_IIO_ATTR_ECK_TYPE]) == 0) {
		return admt4000_iio_write_enum(dev, SENSOR_ATTR_ADMT4000_ECK_TYPE,
					       admt4000_iio_eck_type,
					       ARRAY_SIZE(admt4000_iio_eck_type),
					       src, len);
	}

	if (strcmp(attr->name, admt4000_iio_dev_attr_names[ADMT4000_IIO_ATTR_COIL_RESET]) == 0) {
		return admt4000_iio_write_coil_reset(dev, len);
	}

	/* harmonics, the *_scale and the *_available attributes are read-only. */
	LOG_ERR("Attr %s is not writable", attr->name);

	return -EACCES;
}

static int admt4000_iio_init(const struct device *dev)
{
	const struct admt4000_iio_config *config = dev->config;

	if (!device_is_ready(config->sensor_dev)) {
		LOG_ERR("ADMT4000 device %s is not ready", config->sensor_dev->name);
		return -ENODEV;
	}

	return 0;
}

static DEVICE_API(iio_device, admt4000_iio_driver_api) = {
	.add_channels = admt4000_iio_add_channels,
	.read_attr = admt4000_iio_read_attr,
	.write_attr = admt4000_iio_write_attr,
};

#define ADMT4000_IIO_INIT(inst)                                                             \
	static struct admt4000_iio_data admt4000_iio_data_##inst;                           \
                                                                                            \
	static const struct admt4000_iio_config admt4000_iio_config_##inst = {              \
		.sensor_dev = DEVICE_DT_GET(DT_INST_PHANDLE(inst, sensor_device)),          \
	};                                                                                  \
                                                                                            \
	IIO_DEVICE_DT_INST_DEFINE(inst, admt4000_iio_init, NULL,                            \
				  &admt4000_iio_data_##inst, &admt4000_iio_config_##inst,   \
				  POST_KERNEL, CONFIG_ADMT4000_IIO_INIT_PRIORITY,           \
				  &admt4000_iio_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ADMT4000_IIO_INIT)
