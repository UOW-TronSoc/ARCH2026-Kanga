#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/init_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <yaml-cpp/yaml.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

namespace kanga_cameras {

struct CameraEntry {
  int video_id{0};
  std::string name;
  double fps{30.0};
};

std::vector<CameraEntry> load_camera_config(const std::string& config_path,
                                            rclcpp::Logger logger) {
  std::vector<CameraEntry> out;
  try {
    YAML::Node root = YAML::LoadFile(config_path);
    YAML::Node cameras = root["cameras"];
    if (!cameras || !cameras.IsSequence()) {
      RCLCPP_ERROR(logger, "Config 'cameras' must be a list");
      return out;
    }
    for (size_t i = 0; i < cameras.size(); ++i) {
      const YAML::Node& c = cameras[i];
      if (!c["video_id"] || !c["name"]) {
        RCLCPP_WARN(logger, "Skipping camera entry without video_id or name");
        continue;
      }
      CameraEntry e;
      e.video_id = c["video_id"].as<int>();
      e.name = c["name"].as<std::string>();
      e.fps = c["fps"] ? c["fps"].as<double>() : 30.0;
      if (e.name.empty()) {
        RCLCPP_WARN(logger, "Skipping camera with empty name");
        continue;
      }
      if (e.fps <= 0 || e.fps > 120) {
        RCLCPP_WARN(logger, "Invalid fps %.1f for '%s', using 30", e.fps, e.name.c_str());
        e.fps = 30.0;
      }
      out.push_back(std::move(e));
    }
  } catch (const std::exception& ex) {
    RCLCPP_ERROR(logger, "Failed to load config %s: %s", config_path.c_str(),
                 ex.what());
  }
  return out;
}

bool device_available(int video_id) {
  cv::VideoCapture cap(video_id, cv::CAP_V4L2);
  bool ok = cap.isOpened();
  if (ok) cap.release();
  return ok;
}

struct CameraHandle {
  std::string name;
  cv::VideoCapture cap;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub;
  rclcpp::TimerBase::SharedPtr timer;
};

class CameraPublisherNode : public rclcpp::Node {
 public:
  explicit CameraPublisherNode() : Node("camera_publisher_node") {
    declare_parameter<std::string>("config_file", "");

    std::string config_path = get_parameter("config_file").get_value<std::string>();
    if (config_path.empty()) {
      const char* env = std::getenv("KANGA_CAMERAS_CONFIG");
      if (env && env[0] != '\0') {
        config_path = env;
      }
    }
    if (config_path.empty() || !std::ifstream(config_path).good()) {
      try {
        std::string pkg_share =
            ament_index_cpp::get_package_share_directory("kanga_cameras");
        config_path = pkg_share + "/config/cameras.yaml";
      } catch (const std::exception&) {
        config_path = "config/cameras.yaml";
      }
    }

    std::ifstream check(config_path);
    if (!check.good()) {
      RCLCPP_FATAL(get_logger(), "Config file not found: %s", config_path.c_str());
      throw std::runtime_error(std::string("Config file not found: ") + config_path);
    }

    std::vector<CameraEntry> cameras =
        load_camera_config(config_path, get_logger());
    if (cameras.empty()) {
      RCLCPP_WARN(get_logger(), "No cameras in config");
    }

    std::vector<std::string> names;
    for (const auto& c : cameras) {
      names.push_back(c.name);
    }
    RCLCPP_INFO(get_logger(), "Config loaded: cameras from %s",
                config_path.c_str());

    for (const auto& entry : cameras) {
      std::string dev = "/dev/video" + std::to_string(entry.video_id);
      if (!device_available(entry.video_id)) {
        RCLCPP_WARN(get_logger(), "Skipping '%s' (%s): device not available",
                    entry.name.c_str(), dev.c_str());
        continue;
      }
      cv::VideoCapture cap(entry.video_id, cv::CAP_V4L2);
      if (!cap.isOpened()) {
        RCLCPP_WARN(get_logger(), "Skipping '%s' (%s): open failed",
                    entry.name.c_str(), dev.c_str());
        continue;
      }
      
      // Set camera properties for higher frame rate
      cap.set(cv::CAP_PROP_FPS, 30.0);
      cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
      
      // Get actual properties after setting
      double actual_fps = cap.get(cv::CAP_PROP_FPS);
      int width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
      int height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
      
      std::string topic = "camera/" + entry.name;
      auto pub = create_publisher<sensor_msgs::msg::CompressedImage>(topic, 1);
      
      // Create a timer for this specific camera at configured FPS
      int period_ms = static_cast<int>(1000.0 / entry.fps);
      auto timer = create_wall_timer(
          std::chrono::milliseconds(period_ms),
          [this, idx = handles_.size()]() { this->camera_callback(idx); });
      
      handles_.push_back(
          CameraHandle{entry.name, std::move(cap), std::move(pub), std::move(timer)});
      RCLCPP_INFO(get_logger(), 
                  "Publishing '%s' on /%s from %s (%dx%d, camera @ %.1f fps, publish @ %.1f Hz)",
                  entry.name.c_str(), topic.c_str(), dev.c_str(),
                  width, height, actual_fps, entry.fps);
    }

    if (handles_.empty()) {
      RCLCPP_WARN(get_logger(), "No cameras available; node will do nothing.");
    }
  }

  ~CameraPublisherNode() override {
    for (auto& h : handles_) {
      h.timer.reset();
      h.cap.release();
    }
  }

 private:
  void camera_callback(size_t idx) {
    if (idx >= handles_.size()) return;
    
    auto& h = handles_[idx];
    cv::Mat frame;
    if (!h.cap.read(frame) || frame.empty()) {
      return;
    }
    
    try {
      std::vector<uchar> buf;
      if (!cv::imencode(".jpg", frame, buf)) {
        return;
      }
      sensor_msgs::msg::CompressedImage msg;
      msg.header.stamp = now();
      msg.header.frame_id = "camera_" + h.name;
      msg.format = "jpeg";
      msg.data = buf;
      h.pub->publish(msg);
    } catch (const std::exception& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Publish failed for '%s': %s", h.name.c_str(),
                           e.what());
    }
  }

  std::vector<CameraHandle> handles_;
};

}  // namespace kanga_cameras

static std::atomic<bool> g_shutdown_requested{false};

static void signal_handler(int) { g_shutdown_requested.store(true); }

int main(int argc, char* argv[]) {
  rclcpp::InitOptions opts;
  opts.shutdown_on_signal = false;
  rclcpp::init(argc, argv, opts);
  std::signal(SIGINT, signal_handler);

  rclcpp::Node::SharedPtr node;
  try {
    node = std::make_shared<kanga_cameras::CameraPublisherNode>();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("camera_publisher_node"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  while (rclcpp::ok() && !g_shutdown_requested.load()) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  node.reset();
  rclcpp::shutdown();
  return 0;
}
