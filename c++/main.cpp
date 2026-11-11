#include <libobsensor/ObSensor.hpp>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <cstdint>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

namespace {

cv::Mat toBgr(const std::shared_ptr<ob::ColorFrame> &frame) {
    const auto width = static_cast<int>(frame->getWidth());
    const auto height = static_cast<int>(frame->getHeight());
    const auto format = frame->getFormat();

    if(format == OB_FORMAT_MJPG) {
        cv::Mat encoded(1, static_cast<int>(frame->getDataSize()), CV_8UC1,
                        frame->getData());
        return cv::imdecode(encoded, cv::IMREAD_COLOR);
    }

    if(format == OB_FORMAT_RGB) {
        cv::Mat rgb(height, width, CV_8UC3, frame->getData());
        cv::Mat bgr;
        cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
        return bgr.clone();
    }

    if(format == OB_FORMAT_BGR) {
        return cv::Mat(height, width, CV_8UC3, frame->getData()).clone();
    }

    if(format == OB_FORMAT_RGBA) {
        cv::Mat rgba(height, width, CV_8UC4, frame->getData());
        cv::Mat bgr;
        cv::cvtColor(rgba, bgr, cv::COLOR_RGBA2BGR);
        return bgr.clone();
    }

    if(format == OB_FORMAT_BGRA) {
        cv::Mat bgra(height, width, CV_8UC4, frame->getData());
        cv::Mat bgr;
        cv::cvtColor(bgra, bgr, cv::COLOR_BGRA2BGR);
        return bgr.clone();
    }

    throw std::runtime_error("当前彩色流格式暂不支持: " +
                             std::to_string(static_cast<int>(format)));
}

void printDeviceInfo(const std::shared_ptr<ob::Device> &device) {
    auto info = device->getDeviceInfo();
    std::cout << "Device: " << info->name() << '\n'
              << "Serial number: " << info->serialNumber() << '\n'
              << "Firmware version: " << info->firmwareVersion() << '\n'
              << "PID: " << info->pid() << ", VID: " << info->vid() << '\n';
}

}  // namespace

int main() {
    try {
        std::cout << "Orbbec SDK version: " << ob::Version::getMajor() << '.'
                  << ob::Version::getMinor() << '.' << ob::Version::getPatch()
                  << '\n';

        ob::Context context;
        auto devices = context.queryDeviceList();
        if(!devices || devices->deviceCount() == 0) {
            std::cerr << "未检测到 Orbbec 相机。\n";
            return 1;
        }

        auto device = devices->getDevice(0);
        printDeviceInfo(device);

        ob::Pipeline pipeline(device);
        pipeline.start(nullptr);
        std::cout << "RGB 预览已启动，按 q 或 ESC 退出。\n";

        while(true) {
            auto frames = pipeline.waitForFrames(1000);
            if(!frames) {
                continue;
            }

            auto color = frames->colorFrame();
            if(!color) {
                continue;
            }

            auto image = toBgr(color);
            if(image.empty()) {
                throw std::runtime_error("无法解码彩色帧。");
            }

            cv::imshow("Orbbec Gemini 335 RGB", image);
            const auto key = cv::waitKey(1) & 0xff;
            if(key == 'q' || key == 27) {
                break;
            }
        }

        pipeline.stop();
    } catch(const ob::Error &error) {
        std::cerr << "Orbbec SDK error: " << error.getMessage() << '\n';
        return 2;
    } catch(const std::exception &error) {
        std::cerr << "Error: " << error.what() << '\n';
        return 3;
    }

    return 0;
}
