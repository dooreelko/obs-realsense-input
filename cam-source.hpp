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
#include <fstream>

#define PLUGIN_NAME "mac-input-realsense"

#define blog(level, msg, ...) blog(level, "[" PLUGIN_NAME "] " msg, ##__VA_ARGS__)

#define debug(format, ...) blog(LOG_DEBUG, format, ##__VA_ARGS__)
#define info(format, ...) blog(LOG_INFO, format, ##__VA_ARGS__)
#define warn(format, ...) blog(LOG_WARNING, format, ##__VA_ARGS__)

// #define STREAM \
// 	RS2_STREAM_COLOR // rs2_stream is a types of data provided by RealSense device           //
#define FORMAT RS2_FORMAT_BGRA8 // rs2_format is identifies how binary data is encoded within a frame   //
#define STREAM_INDEX -1         // Defines the stream index, used for multiple streams of the same type //

// Defaults
#define WIDTH 640
#define HEIGHT 480
#define FPS 30

#define S_CUTOFF "Cutoff"
#define S_OFFSET "Offset"
#define S_DEVICE "Device"
#define S_SENSOR "Sensor"
#define S_RESOLUTION "Resolution"
#define S_FPS "Fps"

struct HWInfo {
	std::vector<std::string> devices;
	std::set<std::string> sensors;
	std::set<int> fpss;
	std::set<std::tuple<int, int>> resolutions;

	static int pack(std::tuple<int, int> pair) { return (std::get<0>(pair) << 16) + std::get<1>(pair); }

	static std::tuple<int, int> unpack(int packed) { return std::tuple<int, int>(packed >> 16, packed & 0xffff); }

	static int width(int packed) { return std::get<0>(unpack(packed)); }

	static int height(int packed) { return std::get<1>(unpack(packed)); }
};

class CamSource {
	obs_source_t *source;

	rs2::pipeline pipe;
	rs2::config config;

	int width = WIDTH;
	int height = HEIGHT;
	int cutoff = 500;
	int xoffset = 50;
	int fps = FPS;
	rs2_stream streamType = RS2_STREAM_COLOR;
	rs2_format format = RS2_FORMAT_BGRA8;
	std::unique_ptr<HWInfo> hardware;
	bool stopThread = true;
	bool processing = false;
	std::thread work_thread_;
	std::vector<uint8_t> frame_data;
	std::vector<uint8_t> result_frame_data;
	std::vector<uint16_t> depth_data;

	rs2_intrinsics color_intrinsics;
	rs2_intrinsics depth_intrinsics;
	rs2_extrinsics depth_extrinsics;
	rs2_extrinsics color_extrinsics;
	rs2::points points;

	CamSource() { gatherHardware(); }

	void run()
	{
		try {
			processing = true;
			info("run type(%d), index(%d) %d x %d @ %d", streamType, STREAM_INDEX, width, height, fps);

			config.enable_stream(RS2_STREAM_COLOR, STREAM_INDEX, width, height, RS2_FORMAT_BGRA8, fps);
			config.enable_stream(RS2_STREAM_DEPTH, STREAM_INDEX, width, height, RS2_FORMAT_Z16, fps);

			// Start streaming with default recommended configuration
			rs2::pipeline_profile pipe_profile = pipe.start(config);
			auto color_profile = pipe_profile.get_stream(RS2_STREAM_COLOR, STREAM_INDEX).as<rs2::video_stream_profile>();
			auto depth_profile = pipe_profile.get_stream(RS2_STREAM_DEPTH, STREAM_INDEX).as<rs2::video_stream_profile>();

			color_intrinsics = color_profile.get_intrinsics();
			depth_intrinsics = depth_profile.get_intrinsics();
			depth_extrinsics = depth_profile.get_extrinsics_to(color_profile);
			color_extrinsics = color_profile.get_extrinsics_to(depth_profile);

			// auto sensor = pipe_profile.get_device().first<rs2::depth_sensor>();
			// if (sensor && sensor.is<rs2::depth_stereo_sensor>()) {
			// 	sensor.set_option(RS2_OPTION_VISUAL_PRESET, RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY);
			// 	info("STEREO");
			// }

			int fstride = 0;
			int fwidth = 0;
			int fheight = 0;

			rs2::pointcloud pc;

			rs2::align align_to_depth(RS2_STREAM_DEPTH);
			rs2::align align_to_color(RS2_STREAM_COLOR);
			rs2::decimation_filter dec;
			dec.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
			rs2::disparity_transform depth2disparity;
			rs2::disparity_transform disparity2depth(false);
			rs2::spatial_filter spat;
			spat.set_option(RS2_OPTION_HOLES_FILL, 5); // 5 = fill all the zero pixels

			// rs2::hole_filling_filter deholer;
			// rs2::threshold_filter threshold(0, cutoff/100.0f);

			rs2::rates_printer printer;

			while (!stopThread) {
				auto frameset = pipe.wait_for_frames();
				frameset = frameset.apply_filter(printer);
				frameset = frameset.apply_filter(align_to_depth);
				// frameset = frameset.apply_filter(dec);
				frameset = frameset.apply_filter(depth2disparity);
				frameset = frameset.apply_filter(spat);
				frameset = frameset.apply_filter(disparity2depth);
				// frameset = frameset.apply_filter(deholer);
				// frameset = frameset.apply_filter(threshold);

				for (auto &&frame : frameset) {

					// info("frames");
					// for (auto frame : frameset) {

					auto stype = frame.get_profile().stream_type();
					// info("got %d", stype);

					if (stype == RS2_STREAM_COLOR) {
						info("got color");
						// color = deholer.process(frame.as<rs2::video_frame>());
						auto color = frame.as<rs2::video_frame>();

						// frameset.apply_filter(deholer);

						pc.map_to(color);

						fstride = color.get_stride_in_bytes();
						fwidth = color.get_width();
						fheight = color.get_height();
						info("color %d %d", fwidth, fheight);

						if (frame_data.size() == 0) {
							frame_data.resize(fwidth * fheight * 4);
						}

						memcpy(frame_data.data(), color.get_data(), color.get_data_size());

						if (result_frame_data.size() != frame_data.size()) {
							result_frame_data.resize(frame_data.size());
							result_frame_data.assign(result_frame_data.size(), 0);
						}
						memcpy(result_frame_data.data(), color.get_data(), color.get_data_size());

						// app_state.tex.upload(color);
					} else if (stype == RS2_STREAM_DEPTH) {
						info("got depth");
						auto depth = frame.as<rs2::depth_frame>();

						if (depth) {
							info("depth %d %d", depth.get_width(), depth.get_height());

							if (depth_data.size() == 0) {
								depth_data.resize(depth.get_width() * depth.get_height());
							}

							points = pc.calculate(depth);

							memcpy(depth_data.data(), depth.get_data(), depth.get_data_size());
						}
					}
				}

				info("-- %d", frame_data.size());

				do_cutoff(result_frame_data, depth_data);

				if (result_frame_data.size() != 0) {
					obs_source_frame obsframe = {};
					obsframe.data[0] = result_frame_data.data();
					obsframe.linesize[0] = fstride;
					obsframe.format = VIDEO_FORMAT_BGRA;
					obsframe.width = fwidth;
					obsframe.height = fheight;
					obsframe.timestamp = os_gettime_ns();
					obs_source_output_video(source, &obsframe);
				}
			}
		} catch (const rs2::error &e2) {
			warn("r %s %s %s", e2.what(), e2.get_failed_function().c_str(), e2.get_failed_args().c_str());
		} catch (std::exception &e) {
			warn("r %s", e.what());
		} catch (...) {
			warn("r WOOT");
		}

		processing = false;
	}

	void do_cutoff(std::vector<uint8_t> &frame_data, std::vector<uint16_t> &depth_data)
	{
		if (frame_data.size() != 0 && depth_data.size() != 0) {
			for (size_t i = 0; i < depth_data.size(); i++) {

				size_t x = i % color_intrinsics.width;
				size_t y = i / color_intrinsics.width;

				if (x >= color_intrinsics.width) {
					continue;
				}

				size_t newi = y * color_intrinsics.width + x;

				if (depth_data[i] > cutoff) {
					frame_data[newi * 4 + 3] = 0;
				}
			}
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

				std::string sname(sensor.get_info(RS2_CAMERA_INFO_NAME));
				// info("SS %s", sname.c_str());
				hardware->sensors.insert(sname);

				try {
					for (auto profile : sensor.get_stream_profiles()) {

						// info("%s", profile.stream_name()
						// 		   .c_str());

						auto cp = profile.get();
						int w, h;
						rs2_get_video_stream_resolution(cp, &w, &h, NULL);

						// info("%d x %d", w, h);
						hardware->resolutions.insert(std::tuple<int, int>(w, h));

						auto fps = profile.fps();
						info("%s / %s: %dfps @ %d x %d", sname.c_str(), profile.stream_name().c_str(), fps, w, h);

						hardware->fpss.insert(fps);
					}
				} catch (const rs2::error e2) {
					warn("r %s %s %s", e2.what(), e2.get_failed_function().c_str(), e2.get_failed_args().c_str());
				}
			}
		}
	}

	void update(obs_data_t *settings_) { init(settings_, source); }

	void init(obs_data_t *settings, obs_source_t *newsource)
	{
		info("reinit");
		// if (!stopThread && work_thread_.joinable()) {
		// 	stopThread = true;

		// 	while (processing)
		// 		std::this_thread::yield();

		// 	work_thread_.join();
		// }

		source = newsource;

		auto packed = obs_data_get_int(settings, S_RESOLUTION);
		cutoff = obs_data_get_int(settings, S_CUTOFF);
		xoffset = obs_data_get_int(settings, S_OFFSET);

		width = HWInfo::width(packed);
		height = HWInfo::height(packed);
		fps = obs_data_get_int(settings, S_FPS);
		streamType = (rs2_stream)obs_data_get_int(settings, S_SENSOR);

		if (stopThread) {
			gatherHardware();
			stopThread = false;
			auto self = this;
			work_thread_ = std::thread([self]() { self->run(); });
		}
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

	static void UpdateCamSource(void *, obs_data_t *settings) { CamSource::Instance().update(settings); }

	const HWInfo &Hardware() { return *hardware; }

	inline ~CamSource()
	{
		info("destroy");
		stopThread = true;
		work_thread_.join();
	}
};
