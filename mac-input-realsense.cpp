#include <obs-module.h>
#include <graphics/image-file.h>
#include <util/platform.h>
#include <util/dstr.h>
#include <sys/stat.h>

#include <librealsense2/rs.hpp>

#include <string>
#include <set>
#include <vector>
#include <tuple>
#include <thread>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>

#include "cam-source.hpp"

#define PLUGIN_NAME "mac-input-realsense"

const char *streams[] = {"depth data",
			 "color data",
			 "infrared data",
			 "fish-eye (wide) from the dedicate motion camera",
			 "gyroscope motion data",
			 "accelerometer motion data",
			 "Signals from external device connected through GPIO",
			 "6 Degrees of Freedom pose data"};

static const char *GetCamName(void *)
{
	return "Realsense video/3d sources";
}

static void DestroyCamSource(void *)
{
	// delete reinterpret_cast<CamSource *>(data);
}

static void realsense_source_defaults(obs_data_t *settings)
{
	info("defaults");
	auto hw = CamSource::Instance().Hardware();

	obs_data_set_default_int(settings, S_CUTOFF, 500);
	obs_data_set_default_int(settings, S_OFFSET, 50);
	obs_data_set_default_string(settings, S_DEVICE, !hw.devices.empty() ? hw.devices.front().c_str() : "");
	obs_data_set_default_string(settings, S_SENSOR,
				    // !hw.sensors.empty() ? hw.sensors.front().c_str() : "");
				    "");
	obs_data_set_default_int(settings, S_RESOLUTION, (WIDTH << 16) + HEIGHT);
	obs_data_set_default_int(settings, S_FPS, FPS);
}

static obs_properties_t *realsense_source_properties(void *unused)
{
	UNUSED_PARAMETER(unused);

	obs_properties_t *props = obs_properties_create();

	obs_properties_add_int_slider(props, S_CUTOFF, "Depth cutoff", 0, 56535, 10);
	obs_properties_add_int_slider(props, S_OFFSET, "Horizonral offset", -500, 500, 5);

	obs_property_t *devices_prop = obs_properties_add_list(props, S_DEVICE, obs_module_text("RS.Device"), OBS_COMBO_TYPE_LIST, OBS_COMBO_FORMAT_STRING);

	obs_property_t *sensors_prop = obs_properties_add_list(props, S_SENSOR, obs_module_text("RS.Sensor"), OBS_COMBO_TYPE_LIST, OBS_COMBO_FORMAT_INT);

	obs_property_t *resolutions_prop = obs_properties_add_list(props, S_RESOLUTION, obs_module_text("RS.Resolution"), OBS_COMBO_TYPE_LIST, OBS_COMBO_FORMAT_INT);

	obs_property_t *fps_prop = obs_properties_add_list(props, S_FPS, obs_module_text("RS.FPS"), OBS_COMBO_TYPE_LIST, OBS_COMBO_FORMAT_INT);

	auto hw = CamSource::Instance().Hardware();

	for (auto name : hw.devices) {
		obs_property_list_add_string(devices_prop, name.c_str(), name.c_str());
	}

	for (int stream = RS2_STREAM_DEPTH; stream < RS2_STREAM_COUNT - 1; stream++) {
		obs_property_list_add_int(sensors_prop, streams[stream - RS2_STREAM_DEPTH], stream);
	}

	for (auto fps : hw.fpss) {
		obs_property_list_add_int(fps_prop, std::to_string(fps).c_str(), fps);
	}

	for (auto res : hw.resolutions) {
		int w = std::get<0>(res);
		int h = std::get<1>(res);
		obs_property_list_add_int(resolutions_prop, (std::to_string(w) + " x " + std::to_string(h)).c_str(), HWInfo::pack(res));
	}

	return props;
}

static struct obs_source_info input_realsense_info = {
	.id = "input_realsense",
	.type = OBS_SOURCE_TYPE_INPUT,
	.output_flags = OBS_SOURCE_ASYNC_VIDEO,
	.get_name = GetCamName,
	.create = CamSource::CreateCamSource,
	.destroy = DestroyCamSource,
	.update = CamSource::UpdateCamSource,
	.icon_type = OBS_ICON_TYPE_CAMERA,
	.get_defaults = realsense_source_defaults,
	.get_properties = realsense_source_properties,
};

OBS_DECLARE_MODULE()
OBS_MODULE_USE_DEFAULT_LOCALE("reaslsense-input", "en-US")
MODULE_EXPORT const char *obs_module_description(void)
{
	return "Realsense video/3d sources";
}

bool obs_module_load(void)
{
	obs_register_source(&input_realsense_info);
	return true;
}
