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

#define S_DEVICE "Device"
#define S_RESOLUTION "Resolution"
#define S_FPS "Fps"

#define PLUGIN_NAME "mac-input-realsense"

#define blog(level, msg, ...) \
	blog(level, "[" PLUGIN_NAME "] " msg, ##__VA_ARGS__)

#define debug(format, ...) blog(LOG_DEBUG, format, ##__VA_ARGS__)
#define info(format, ...) blog(LOG_INFO, format, ##__VA_ARGS__)
#define warn(format, ...) blog(LOG_WARNING, format, ##__VA_ARGS__)


#define STREAM \
	RS2_STREAM_COLOR // rs2_stream is a types of data provided by RealSense device           //
#define FORMAT \
	RS2_FORMAT_BGRA8 // rs2_format is identifies how binary data is encoded within a frame   //
#define STREAM_INDEX \
	-1 // Defines the stream index, used for multiple streams of the same type //

// Defaults
#define WIDTH 640
#define HEIGHT 480
#define FPS 30

struct HWInfo {
	std::vector<std::string> devices;
	std::set<std::string> sensors;
	std::set<int> fpss;
	std::set<std::tuple<int, int>> resolutions;

	static int pack(std::tuple<int, int> pair)
	{
		return (std::get<0>(pair) << 16) + std::get<1>(pair);
	}

	static std::tuple<int, int> unpack(int packed)
	{
		return std::tuple<int, int>(packed >> 16, packed & 0xffff);
	}

	static int width(int packed) { return std::get<0>(unpack(packed)); }

	static int height(int packed) { return std::get<1>(unpack(packed)); }
};

class CamSource {
	obs_source_t *source;
	int width = WIDTH;
	int height = HEIGHT;
	int fps = FPS;
	std::unique_ptr<HWInfo> hardware;
	bool stopThread;
	std::thread work_thread_;
	std::vector<uint8_t> frame_data;

	CamSource() { gatherHardware(); }

	void run()
	{
		try {
			info("run %d x %d @ %d", width, height, fps);
			rs2::pipeline pipe;
			rs2::config config;
			// config.enable_stream(STREAM, STREAM_INDEX, WIDTH,
			// 		     HEIGHT, FORMAT, FPS);
			frame_data.resize(width * height * 4);
			config.enable_stream(STREAM, STREAM_INDEX, width,
					     height, FORMAT, fps);
			// Start streaming with default recommended configuration
			pipe.start(config);

			while (!stopThread) {
				for (auto &&frame : pipe.wait_for_frames()) {
					if (auto vf = frame.as<
						      rs2::video_frame>()) {
						auto stream =
							frame.get_profile()
								.stream_type();

						if (stream !=
						    RS2_STREAM_COLOR) {
							continue;
						}

						memcpy(frame_data.data(),
						       vf.get_data(),
						       vf.get_data_size());

						obs_source_frame frame = {};
						frame.data[0] =
							frame_data.data();
						// (uint8_t *)vf.get_data();
						// frame.linesize[0] = 0;
						frame.linesize[0] =
							vf.get_stride_in_bytes();
						frame.format =
							VIDEO_FORMAT_BGRA;
						frame.width = vf.get_width();
						frame.height = vf.get_height();
						frame.timestamp =
							os_gettime_ns();
						obs_source_output_video(source,
									&frame);

						// s.send((const char *)vf.get_data(), vf.get_data_size());
					}
				}
			}
		} catch (const rs2::error &e2) {
			warn("r %s %s %s", e2.what(),
			     e2.get_failed_function().c_str(),
			     e2.get_failed_args().c_str());
		} catch (std::exception &e) {
			warn("r %s", e.what());
		} catch (...) {
			warn("r WOOT");
		}
	}

	void gatherHardware()
	{
		info("gather hw");
		rs2::context ctx;

		rs2::device_list devices = ctx.query_devices();

		hardware = std::unique_ptr<HWInfo>(new HWInfo());

		info("Got %u devices", devices.size());

		for (auto device : devices) {
			std::string name(device.get_info(RS2_CAMERA_INFO_NAME));
			hardware->devices.push_back(name);
			info("%s", name.c_str());

			auto sensors = device.query_sensors();
			info("%lu sensors", sensors.size());
			for (auto sensor : sensors) {

				std::string sname(
					sensor.get_info(RS2_CAMERA_INFO_NAME));
				info("SS %s", sname.c_str());
				hardware->sensors.insert(name);

				try {
					for (auto profile :
					     sensor.get_stream_profiles()) {

						info("%s", profile.stream_name()
								   .c_str());

						auto cp = profile.get();
						int w, h;
						rs2_get_video_stream_resolution(
							cp, &w, &h, NULL);

						info("%d x %d", w, h);
						hardware->resolutions.insert(
							std::tuple<int, int>(
								w, h));

						auto fps = profile.fps();
						info("%s: %d fps",
						     profile.stream_name()
							     .c_str(),
						     fps);

						hardware->fpss.insert(fps);
					}
				} catch (const rs2::error e2) {
					warn("r %s %s %s", e2.what(),
					     e2.get_failed_function().c_str(),
					     e2.get_failed_args().c_str());
				}
			}
		}
	}

	void update(obs_data_t *settings_) { init(settings_, source); }

	void init(obs_data_t *settings, obs_source_t *newsource)
	{
		info("reinit");
		if (!stopThread && work_thread_.joinable()) {
			stopThread = true;
			work_thread_.join();
		}

		source = newsource;

		auto packed = obs_data_get_int(settings, S_RESOLUTION);

		width = HWInfo::width(packed);
		height = HWInfo::height(packed);
		fps = obs_data_get_int(settings, S_FPS);

		gatherHardware();

		stopThread = false;
		auto self = this;
		work_thread_ = std::thread([self]() { self->run(); });
	}

public:
	static CamSource &Instance()
	{
		info("instance");
		static CamSource theInstance;

		return theInstance;
	}

	static void *CreateCamSource(obs_data_t *settings, obs_source_t *source)
	{
		CamSource::Instance().init(settings, source);

		return (void *)&CamSource::Instance();
	}

	static void UpdateCamSource(void *, obs_data_t *settings)
	{
		CamSource::Instance().update(settings);
	}

	const HWInfo &Hardware() { return *hardware; }

	inline ~CamSource()
	{
		info("destroy");
		stopThread = true;
		work_thread_.join();
	}
};

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

	obs_data_set_default_string(
		settings, S_DEVICE,
		!hw.devices.empty() ? hw.devices.front().c_str() : "");
	obs_data_set_default_int(settings, S_RESOLUTION,
				 (WIDTH << 16) + HEIGHT);
	obs_data_set_default_int(settings, S_FPS, FPS);
}

static obs_properties_t *realsense_source_properties(void *unused)
{
	UNUSED_PARAMETER(unused);

	obs_properties_t *props = obs_properties_create();

	obs_property_t *devices_prop = obs_properties_add_list(
		props, S_DEVICE, obs_module_text("RS.Device"),
		OBS_COMBO_TYPE_LIST, OBS_COMBO_FORMAT_STRING);

	obs_property_t *resolutions_prop = obs_properties_add_list(
		props, S_RESOLUTION, obs_module_text("RS.Resolution"),
		OBS_COMBO_TYPE_LIST, OBS_COMBO_FORMAT_INT);

	obs_property_t *fps_prop = obs_properties_add_list(
		props, S_FPS, obs_module_text("RS.FPS"), OBS_COMBO_TYPE_LIST,
		OBS_COMBO_FORMAT_INT);

	auto hw = CamSource::Instance().Hardware();

	for (auto name : hw.devices) {
		obs_property_list_add_string(devices_prop, name.c_str(),
					     name.c_str());
	}

	for (auto fps : hw.fpss) {
		obs_property_list_add_int(fps_prop, std::to_string(fps).c_str(),
					  fps);
	}

	for (auto res : hw.resolutions) {
		int w = std::get<0>(res);
		int h = std::get<1>(res);
		obs_property_list_add_int(
			resolutions_prop,
			(std::to_string(w) + " x " + std::to_string(h)).c_str(),
			HWInfo::pack(res));
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
