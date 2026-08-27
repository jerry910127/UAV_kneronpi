//WEIWEIYE 從windows 搬過來僅供測試用

#include "TemperatureProcessor.h"

#include <exception>
#include <iostream>
#include <stdexcept>

void runCameraDemo(int cameraIndex = DEFAULT_CAMERA_INDEX, int width = DEFAULT_WIDTH, int height = DEFAULT_HEIGHT) {
    cv::VideoCapture cap(cameraIndex);
    cap.set(cv::CAP_PROP_CONVERT_RGB, 0);
    if (!cap.isOpened()) {
        throw std::runtime_error("Cannot open camera.");
    }

    bool showHigh = false;
    bool showLow = false;
    int frameNumber = 0;
    int minValue = DEFAULT_MIN_VALUE;
    int maxValue = DEFAULT_MAX_VALUE;

    while (true) {
        cv::Mat frame;
        const bool ok = cap.read(frame);
        ++frameNumber;
        if (!ok) {
            throw std::runtime_error("Cannot receive frame from camera.");
        }

        ProcessFrameResult result = processFrame(
            frame,
            showHigh,
            showLow,
            width,
            height,
            DEFAULT_CROP_BORDER,
            minValue,
            maxValue
        );
        cv::imshow("final", result.display_image);

        if (frameNumber > 8) {
            frameNumber = 0;
            const auto window = updateDisplayWindow(result.raw_image);
            minValue = window.first;
            maxValue = window.second;
        }

        const int key = cv::waitKey(5) & 0xFF;
        if (key == 'q' || key == 'Q') {
            break;
        }
        if (key == 'o' || key == 'O') {
            showHigh = !showHigh;
        }
        if (key == 'p' || key == 'P') {
            showLow = !showLow;
        }
    }

    cap.release();
    cv::destroyAllWindows();
}

int main() {
    try {
        runCameraDemo();
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}

