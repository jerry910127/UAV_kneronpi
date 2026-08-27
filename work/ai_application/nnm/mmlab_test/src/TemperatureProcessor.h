#pragma once

#include <opencv2/opencv.hpp>

#include <limits>
#include <string>
#include <utility>

constexpr int DEFAULT_WIDTH = 480;
constexpr int DEFAULT_HEIGHT = 640;
constexpr int DEFAULT_CAMERA_INDEX = 0;
constexpr int DEFAULT_CROP_BORDER = 5;
constexpr int DEFAULT_MIN_VALUE = 31500;
constexpr int DEFAULT_MAX_VALUE = 34000;
constexpr double DEFAULT_PERCENTILE = 1.0;
constexpr int DEFAULT_TCAM_ROW = -1;
constexpr int DEFAULT_TCAM_COL = 479;
constexpr int DEFAULT_K_COUNT = 100;

struct TemperatureResult {
    bool ok = false;
    std::string type;
    std::string selection;
    std::string reason;

    double tcam_kelvin = std::numeric_limits<double>::quiet_NaN();
    double tcam_c = std::numeric_limits<double>::quiet_NaN();

    int location_x = 0;
    int location_y = 0;
    int rect_x = 0;
    int rect_y = 0;
    int rect_width = 0;
    int rect_height = 0;
    int pixel_count = 0;

    double raw_value = std::numeric_limits<double>::quiet_NaN();
    double raw_region_mean = std::numeric_limits<double>::quiet_NaN();
    double raw_region_k100_mean = std::numeric_limits<double>::quiet_NaN();
    double temperature_c = std::numeric_limits<double>::quiet_NaN();
    double temperature_region_mean_c = std::numeric_limits<double>::quiet_NaN();
    double temperature_region_k100_mean_c = std::numeric_limits<double>::quiet_NaN();

    cv::Mat region_mask;
};

struct ProcessFrameResult {
    bool ok = false;
    std::string reason;

    double tcam_kelvin = std::numeric_limits<double>::quiet_NaN();
    double tcam_c = std::numeric_limits<double>::quiet_NaN();

    cv::Mat raw_image;
    cv::Mat display_image;
    TemperatureResult high;
    TemperatureResult low;
};

cv::Mat processFrameForDisplay(const cv::Mat& frame, int width = DEFAULT_WIDTH, int height = DEFAULT_HEIGHT);
double getTcamKelvin(const cv::Mat& rawImage, int row = DEFAULT_TCAM_ROW, int col = DEFAULT_TCAM_COL);
double calculateTemperature(double rawValue, double tcamKelvin);
cv::Mat makeDisplayImage(const cv::Mat& rawImage, int minValue = DEFAULT_MIN_VALUE, int maxValue = DEFAULT_MAX_VALUE);

TemperatureResult analyzeHighTemperature(
    const cv::Mat& rawImage,
    double tcamKelvin = std::numeric_limits<double>::quiet_NaN(),
    double percentile = DEFAULT_PERCENTILE
);

TemperatureResult analyzeLowTemperature(
    const cv::Mat& rawImage,
    double tcamKelvin = std::numeric_limits<double>::quiet_NaN(),
    double percentile = DEFAULT_PERCENTILE
);

void drawHighTemperature(cv::Mat& displayImage, const TemperatureResult& result);
void drawLowTemperature(cv::Mat& displayImage, const TemperatureResult& result);
void drawTcam(cv::Mat& displayImage, double tcamKelvin);

ProcessFrameResult processFrame(
    const cv::Mat& frame,
    bool showHigh = false,
    bool showLow = false,
    int width = DEFAULT_WIDTH,
    int height = DEFAULT_HEIGHT,
    int cropBorder = DEFAULT_CROP_BORDER,
    int minValue = DEFAULT_MIN_VALUE,
    int maxValue = DEFAULT_MAX_VALUE,
    double percentile = DEFAULT_PERCENTILE
);

std::pair<int, int> updateDisplayWindow(const cv::Mat& rawImage, double sigma = 3.0);

