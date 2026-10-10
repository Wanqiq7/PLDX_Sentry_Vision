#include "io/camera.hpp"

#include <opencv2/opencv.hpp>

#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ? |                     | 输出命令行参数说明}"
  "{config-path c  | configs/camera.yaml | yaml配置文件路径 }"
  "{frames         | 0 | 采集帧数，0 表示持续采集 }"
  "{save           | | 保存第一帧的路径 }"
  "{d display      |                     | 显示视频流       }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;

  auto config_path = cli.get<std::string>("config-path");
  auto display = cli.has("display");
  const auto frame_limit = cli.get<int>("frames");
  const auto save_path = cli.get<std::string>("save");
  io::Camera camera(config_path);

  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  auto last_stamp = std::chrono::steady_clock::now();
  int frames = 0;
  std::chrono::steady_clock::time_point first_stamp;
  while (!exiter.exit()) {
    camera.read(img, timestamp);
    if (img.empty()) return 1;
    if (frames++ == 0) {
      first_stamp = timestamp;
      tools::logger()->info("Image: {}x{}, channels={}", img.cols, img.rows, img.channels());
      if (!save_path.empty() && !cv::imwrite(save_path, img)) return 1;
    }

    auto dt = tools::delta_time(timestamp, last_stamp);
    last_stamp = timestamp;

    tools::logger()->info("{:.2f} fps", 1 / dt);
    if (frame_limit > 0 && frames >= frame_limit) {
      tools::logger()->info("Captured {} frames, average {:.2f} fps", frames,
        (frames - 1) / tools::delta_time(timestamp, first_stamp));
      break;
    }

    if (!display) continue;
    cv::imshow("img", img);
    if (cv::waitKey(1) == 'q') break;
  }
}
