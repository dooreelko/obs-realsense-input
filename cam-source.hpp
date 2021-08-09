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

#include "render.h"

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

	CamSource() { gatherHardware(); }

	void run()
	{
		try {
			processing = true;
			info("run type(%d), index(%d) %d x %d @ %d", streamType, STREAM_INDEX, width, height, fps);

			config.enable_stream(RS2_STREAM_COLOR, STREAM_INDEX, width, height, RS2_FORMAT_BGRA8, fps);
			config.enable_stream(RS2_STREAM_DEPTH, STREAM_INDEX, width, height, RS2_FORMAT_Z16, fps);

			// Start streaming with default recommended configuration
			pipe.start(config);

			int fstride = 0;
			int fwidth = 0;
			int fheight = 0;

			rs2::pointcloud pc;
			rs2::points points;

			// rs2::align align_to_depth(RS2_STREAM_DEPTH);
			// rs2::align align_to_color(RS2_STREAM_COLOR);

			rs2::hole_filling_filter deholer;

			rs2::rates_printer printer;

			glfw_state app_state;

			while (!stopThread) {
				auto frameset = pipe.wait_for_frames();
				frameset.apply_filter(printer);

				for (auto &&frame : frameset) {

					// info("frames");
					// for (auto frame : frameset) {

					auto stype = frame.get_profile().stream_type();
					// info("got %d", stype);

					if (stype == RS2_STREAM_COLOR) {
						info("got color");
						rs2::video_frame color = deholer.process(frame.as<rs2::video_frame>());

						// frameset.apply_filter(deholer);

						// pc.map_to(color);

						fstride = color.get_stride_in_bytes();
						fwidth = color.get_width();
						fheight = color.get_height();
						info("color %d %d", fwidth, fheight);

						if (frame_data.size() == 0) {
							frame_data.resize(fwidth * fheight * 4);
						}

						memcpy(frame_data.data(), color.get_data(), color.get_data_size());
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

							// info("points %d %d", points.get_width(), points.get_height());

							memcpy(depth_data.data(), depth.get_data(), depth.get_data_size());

							if (frame_data.size() != 0) {
								// render_to_mem(fwidth, fheight, app_state, points, frame_data);
							} else {
								info("no color frame");
							}
						}
					}

					// info("got frames %d %d", !color, !depth);
					// }
				}

				info("--");

				translate_points(fwidth, fheight, points, frame_data, depth_data, result_frame_data, cutoff);

				// do_cutoff(fwidth, frame_data, depth_data);

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

	static void translate_points(int fwidth, int fheight, rs2::points &points, std::vector<uint8_t> &color, std::vector<uint16_t> &depth_data, std::vector<uint8_t> &dest, int cutoff)
	{
		if (color.size() == 0 && depth_data.size() == 0) {
			return;
		}

		dest.resize(color.size());
		dest.assign(dest.size(), 0);

		auto vertices = points.get_vertices();
		auto tex_coords = points.get_texture_coordinates();

		float minx, maxx, miny, maxy, minz, maxz = 0;
		for (int i = 0; i < points.size(); i++) {
			if (vertices[i].x < minx) {
				minx = vertices[i].x;
			}
			if (vertices[i].x > maxx) {
				maxx = vertices[i].x;
			}
			if (vertices[i].y < miny) {
				miny = vertices[i].y;
			}
			if (vertices[i].y > maxy) {
				maxy = vertices[i].y;
			}
			if (vertices[i].z && vertices[i].z < minz) {
				minz = vertices[i].z;
			}
			if (vertices[i].z && vertices[i].z > maxz) {
				maxz = vertices[i].z;
			}
		}

		info("m %f %f %f %f %f %f", minx, maxx, miny, maxy, minz, maxz);

		float depths[fwidth][fheight];

		for (int i = 0; i < points.size(); i++) {

			if (vertices[i].z) {
				float x = (vertices[i].x - minx) / (maxx - minx) * fwidth;
				float y = (vertices[i].y - miny) / (maxy - miny) * fheight;
				float z = (vertices[i].z - minz) / (maxz - minz) * 65535;

				if (x >= fwidth || y >= fheight || x < 0 || y < 0) {
					continue;
				}

				// info("v %f %f %f %f %f %f", vertices[i].x, vertices[i].y, vertices[i].z, x, y, z);
				depths[int(x)][int(y)] = vertices[i].z;

				if (z < cutoff) {
					continue;
				}

				size_t newi = y * fwidth + x;

				if (newi > dest.size()) {
					info("woot %d", newi);
					continue;
				}

				// dest[newi * 4 + 0] = color[newi * 4 + 0];
				// dest[newi * 4 + 1] = color[newi * 4 + 1];
				// dest[newi * 4 + 2] = color[newi * 4 + 2];
				// dest[newi * 4 + 3] = color[newi * 4 + 3];

				auto zcolor = (vertices[i].z - minz) / (maxz - minz) * 256;
				// auto zcolor = int((float(depth_data[newi]) / 65535) * 256);
				// info("%f", zcolor);

				dest[newi * 4 + 0] = zcolor;
				dest[newi * 4 + 1] = zcolor;
				dest[newi * 4 + 2] = zcolor;
				dest[newi * 4 + 3] = 255;
			}
		}

		info("fw");

		// FILE *filePtr;

		// filePtr = fopen("floatArray", "w");

		// for (int x = 0; x < fwidth; x++) {
		// 	for (int y = 0; y < fheight; y++) {
		// 		fprintf(filePtr, "%.3g ", depths[x][y]);
		// 	}
		// 	fprintf(filePtr, "\n");
		// }

		// fclose(filePtr);
	}

	void do_cutoff(int fwidth, std::vector<uint8_t> &frame_data, std::vector<uint16_t> &depth_data)
	{
		// int xoffset = 50;

		if (frame_data.size() != 0 && depth_data.size() != 0) {
			// auto frames = align_to_depth.process(frameset);
			// auto frames = align_to_color.process(frameset);

			for (size_t i = 0; i < depth_data.size(); i++) {
				size_t x = i % fwidth + xoffset;
				size_t y = i / fwidth;

				if (x >= fwidth) {
					continue;
				}

				size_t newi = y * fwidth + x;

				if (depth_data[i] > cutoff) {
					frame_data[newi * 4 + 3] = 0;
				} else {
					// frame_data[i * 4 + 0] = 255;
					// frame_data[i * 4 + 1] = 255;
					// frame_data[i * 4 + 2] = 255;
					// frame_data[i * 4 + 3] = 255;
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
