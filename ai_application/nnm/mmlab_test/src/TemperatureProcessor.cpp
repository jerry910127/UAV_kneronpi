#include "TemperatureProcessor.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <vector>

constexpr double T1 = 2.0;
constexpr double T2 = -2.004524681379069;
constexpr double T0 = -509.29998779296875;
constexpr double RADIOMETRIC = 0.0078125;

struct RegionSelection {
    bool ok = false;
    cv::Mat mask;
    cv::Rect rect;
};

struct RawExtreme {
    cv::Point location;
    double raw_value = std::numeric_limits<double>::quiet_NaN();
};

double nanValue() {
    return std::numeric_limits<double>::quiet_NaN();
}

void require2DImage(const cv::Mat& image) {
    if (image.empty() || image.channels() != 1) {
        throw std::invalid_argument("Expected a non-empty single-channel 2D image.");
    }
}

double matValueAt(const cv::Mat& image, int y, int x) {
    switch (image.depth()) {
        case CV_8U:
            return static_cast<double>(image.at<std::uint8_t>(y, x));
        case CV_8S:
            return static_cast<double>(image.at<std::int8_t>(y, x));
        case CV_16U:
            return static_cast<double>(image.at<std::uint16_t>(y, x));
        case CV_16S:
            return static_cast<double>(image.at<std::int16_t>(y, x));
        case CV_32S:
            return static_cast<double>(image.at<std::int32_t>(y, x));
        case CV_32F:
            return static_cast<double>(image.at<float>(y, x));
        case CV_64F:
            return image.at<double>(y, x);
        default:
            throw std::invalid_argument("Unsupported image depth.");
    }
}

std::vector<double> valuesInMask(const cv::Mat& image, const cv::Mat& mask) {
    require2DImage(image);
    require2DImage(mask);
    if (image.size() != mask.size() || mask.type() != CV_8U) {
        throw std::invalid_argument("Mask must be CV_8U and have the same size as image.");
    }

    std::vector<double> values;
    values.reserve(static_cast<std::size_t>(cv::countNonZero(mask)));
    for (int y = 0; y < image.rows; ++y) {
        for (int x = 0; x < image.cols; ++x) {
            if (mask.at<std::uint8_t>(y, x) != 0) {
                values.push_back(matValueAt(image, y, x));
            }
        }
    }
    return values;
}

double meanValue(const std::vector<double>& values) {
    if (values.empty()) {
        return nanValue();
    }
    const double sum = std::accumulate(values.begin(), values.end(), 0.0);
    return sum / static_cast<double>(values.size());
}

std::string fixed2(double value) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(2) << value;
    return out.str();
}

std::string titleText(std::string text) {
    if (text.empty()) {
        return text;
    }
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    text[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(text[0])));
    return text;
}

TemperatureResult emptyTemperatureResult(
    const std::string& type,
    const std::string& selection,
    const std::string& reason,
    double tcamKelvin
) {
    TemperatureResult result;
    result.ok = false;
    result.type = type;
    result.selection = selection;
    result.reason = reason;
    result.tcam_kelvin = tcamKelvin;
    result.tcam_c = std::isfinite(tcamKelvin) ? tcamKelvin - 273.15 : nanValue();
    return result;
}

double percentileValue(std::vector<double> values, double percentile) {
    values.erase(std::remove_if(values.begin(), values.end(), [](double value) {
        return !std::isfinite(value);
    }), values.end());
    if (values.empty()) {
        return nanValue();
    }

    std::sort(values.begin(), values.end());
    const double clipped = std::clamp(percentile, 0.0, 100.0);
    const double rank = (static_cast<double>(values.size()) - 1.0) * clipped / 100.0;
    const auto lo = static_cast<std::size_t>(std::floor(rank));
    const auto hi = static_cast<std::size_t>(std::ceil(rank));
    if (lo == hi) {
        return values[lo];
    }

    const double weight = rank - static_cast<double>(lo);
    return values[lo] * (1.0 - weight) + values[hi] * weight;
}

RegionSelection largestConnectedComponent(const cv::Mat& mask) {
    require2DImage(mask);
    if (mask.type() != CV_8U) {
        throw std::invalid_argument("Connected component mask must be CV_8U.");
    }

    RegionSelection result;
    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int labelsCount = cv::connectedComponentsWithStats(mask, labels, stats, centroids, 8, CV_32S);
    if (labelsCount <= 1) {
        return result;
    }

    int largestLabel = 1;
    int largestArea = stats.at<int>(1, cv::CC_STAT_AREA);
    for (int label = 2; label < labelsCount; ++label) {
        const int area = stats.at<int>(label, cv::CC_STAT_AREA);
        if (area > largestArea) {
            largestArea = area;
            largestLabel = label;
        }
    }

    result.ok = true;
    result.mask = labels == largestLabel;
    result.rect = cv::Rect(
        stats.at<int>(largestLabel, cv::CC_STAT_LEFT),
        stats.at<int>(largestLabel, cv::CC_STAT_TOP),
        stats.at<int>(largestLabel, cv::CC_STAT_WIDTH),
        stats.at<int>(largestLabel, cv::CC_STAT_HEIGHT)
    );
    return result;
}

RegionSelection findLargestRawRegion(const cv::Mat& rawImage, const std::string& mode, double percentile) {
    require2DImage(rawImage);

    std::vector<double> values;
    values.reserve(static_cast<std::size_t>(rawImage.rows * rawImage.cols));
    for (int y = 0; y < rawImage.rows; ++y) {
        for (int x = 0; x < rawImage.cols; ++x) {
            values.push_back(matValueAt(rawImage, y, x));
        }
    }

    const bool selectHighRaw = mode == "low";
    const double threshold = percentileValue(values, selectHighRaw ? 100.0 - percentile : percentile);
    if (!std::isfinite(threshold)) {
        return {};
    }

    cv::Mat mask(rawImage.rows, rawImage.cols, CV_8U, cv::Scalar(0));
    for (int y = 0; y < rawImage.rows; ++y) {
        for (int x = 0; x < rawImage.cols; ++x) {
            const double value = matValueAt(rawImage, y, x);
            const bool selected = selectHighRaw ? value >= threshold : value <= threshold;
            if (selected) {
                mask.at<std::uint8_t>(y, x) = 255;
            }
        }
    }

    return largestConnectedComponent(mask);
}

RawExtreme rawExtremeInImage(const cv::Mat& rawImage, const std::string& mode) {
    require2DImage(rawImage);

    RawExtreme result;
    bool found = false;
    double best = mode == "high" ? std::numeric_limits<double>::infinity() : -std::numeric_limits<double>::infinity();

    for (int y = 0; y < rawImage.rows; ++y) {
        for (int x = 0; x < rawImage.cols; ++x) {
            const double value = matValueAt(rawImage, y, x);
            const bool better = mode == "high" ? value < best : value > best;
            if (!found || better) {
                found = true;
                best = value;
                result.location = cv::Point(x, y);
                result.raw_value = value;
            }
        }
    }

    return result;
}

double oneSidedKMean(std::vector<double> values, int k, const std::string& mode) {
    if (values.empty() || k <= 0) {
        return nanValue();
    }

    std::sort(values.begin(), values.end());
    const int count = std::min(k, static_cast<int>(values.size()));
    const int start = mode == "high" ? 0 : static_cast<int>(values.size()) - count;

    double sum = 0.0;
    for (int i = 0; i < count; ++i) {
        sum += values[static_cast<std::size_t>(start + i)];
    }
    return sum / static_cast<double>(count);
}

std::pair<double, double> radiometricParameters(double tcamKelvin) {
    const double gt = RADIOMETRIC * (1.0 - T0 * (tcamKelvin - 320.0));
    const double rgain = -gt / (tcamKelvin * tcamKelvin) * (1.0 + T2 / 2.0);
    const double roffset = std::pow(tcamKelvin - T1, 4.0);
    return {rgain, roffset};
}

cv::Mat processFrameForDisplay(const cv::Mat& frame, int width, int height) {
    if (frame.empty()) {
        throw std::invalid_argument("Frame is empty.");
    }

    const int pixelCount = width * height;
    if (frame.type() == CV_32S && frame.rows == height && frame.cols == width) {
        return frame.clone();
    }

    const cv::Mat continuous = frame.isContinuous() ? frame : frame.clone();
    const std::size_t scalarCount = continuous.total() * static_cast<std::size_t>(continuous.channels());
    cv::Mat rawImage(height, width, CV_32S);

    if (continuous.depth() == CV_16S && scalarCount >= static_cast<std::size_t>(pixelCount)) {
        const auto* data = reinterpret_cast<const std::int16_t*>(continuous.data);
        for (int i = 0; i < pixelCount; ++i) {
            rawImage.at<std::int32_t>(i / width, i % width) = static_cast<std::int32_t>(data[i]);
        }
        return rawImage;
    }

    const std::size_t expectedBytes = static_cast<std::size_t>(pixelCount) * sizeof(std::int16_t);
    const std::size_t actualBytes = continuous.total() * continuous.elemSize();
    if (actualBytes < expectedBytes) {
        std::ostringstream message;
        message << "Frame buffer is too small: need " << expectedBytes << " bytes, got " << actualBytes << " bytes.";
        throw std::invalid_argument(message.str());
    }

    const auto* bytes = reinterpret_cast<const std::uint8_t*>(continuous.data);
    for (int i = 0; i < pixelCount; ++i) {
        const std::uint16_t word = static_cast<std::uint16_t>(bytes[i * 2]) | (static_cast<std::uint16_t>(bytes[i * 2 + 1]) << 8);
        printf("TEST");
        rawImage.at<std::int32_t>(i / width, i % width) = static_cast<std::int16_t>(word);
    }
    return rawImage;
}

double getTcamKelvin(const cv::Mat& rawImage, int row, int col) {
    require2DImage(rawImage);
    int y = row < 0 ? rawImage.rows + row : row;
    int x = std::min(col, rawImage.cols - 1);
    y = std::clamp(y, 0, rawImage.rows - 1);
    x = std::clamp(x, 0, rawImage.cols - 1);
    return matValueAt(rawImage, y, x) / 100.0 + 273.15;
}

double calculateTemperature(double rawValue, double tcamKelvin) {
    try {
        const auto [rgain, roffset] = radiometricParameters(tcamKelvin);
        const double innerValue = (rawValue / rgain) + roffset;
        if (rgain == 0.0 || !std::isfinite(innerValue) || innerValue < 0.0) {
            return nanValue();
        }
        return std::pow(innerValue, 0.25) - 273.15;
    } catch (...) {
        return nanValue();
    }
}

cv::Mat makeDisplayImage(const cv::Mat& rawImage, int minValue, int maxValue) {
    require2DImage(rawImage);
    if (maxValue == minValue) {
        throw std::invalid_argument("maxValue and minValue must be different.");
    }

    const double gain = -255.0 / static_cast<double>(maxValue - minValue);
    const double offset = -static_cast<double>(maxValue) * gain;

    cv::Mat gray(rawImage.rows, rawImage.cols, CV_8U);
    for (int y = 0; y < rawImage.rows; ++y) {
        for (int x = 0; x < rawImage.cols; ++x) {
            const double clamped = std::clamp(
                matValueAt(rawImage, y, x),
                static_cast<double>(minValue),
                static_cast<double>(maxValue)
            );
            const double pixel = clamped * gain + offset;
            gray.at<std::uint8_t>(y, x) = static_cast<std::uint8_t>(std::clamp(pixel, 0.0, 255.0));
        }
    }

    cv::Mat bgr;
    cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);
    return bgr;
}

TemperatureResult analyzeTemperatureRegion(
    const cv::Mat& rawImage,
    double tcamKelvin,
    double percentile,
    const std::string& mode
) {
    require2DImage(rawImage);

    const double resolvedTcam = std::isfinite(tcamKelvin) ? tcamKelvin : getTcamKelvin(rawImage);
    const RegionSelection region = findLargestRawRegion(rawImage, mode, percentile);
    if (!region.ok) {
        return emptyTemperatureResult(mode, "raw_data", "no raw region was found", resolvedTcam);
    }

    const std::vector<double> selectedRaw = valuesInMask(rawImage, region.mask);
    if (selectedRaw.empty()) {
        return emptyTemperatureResult(mode, "raw_data", "selected raw region has no pixels", resolvedTcam);
    }

    const RawExtreme arrow = rawExtremeInImage(rawImage, mode);
    const double rawMean = meanValue(selectedRaw);
    const double rawK100Mean = oneSidedKMean(selectedRaw, DEFAULT_K_COUNT, mode);

    TemperatureResult result;
    result.ok = true;
    result.type = mode;
    result.selection = "raw_data";
    result.tcam_kelvin = resolvedTcam;
    result.tcam_c = std::isfinite(resolvedTcam) ? resolvedTcam - 273.15 : nanValue();
    result.location_x = arrow.location.x;
    result.location_y = arrow.location.y;
    result.rect_x = region.rect.x;
    result.rect_y = region.rect.y;
    result.rect_width = region.rect.width;
    result.rect_height = region.rect.height;
    result.pixel_count = static_cast<int>(selectedRaw.size());
    result.raw_value = arrow.raw_value;
    result.raw_region_mean = rawMean;
    result.raw_region_k100_mean = rawK100Mean;
    result.temperature_c = calculateTemperature(arrow.raw_value, resolvedTcam);
    result.temperature_region_mean_c = calculateTemperature(rawMean, resolvedTcam);
    result.temperature_region_k100_mean_c = calculateTemperature(rawK100Mean, resolvedTcam);
    result.region_mask = region.mask;
    return result;
}

TemperatureResult analyzeHighTemperature(const cv::Mat& rawImage, double tcamKelvin, double percentile) {
    return analyzeTemperatureRegion(rawImage, tcamKelvin, percentile, "high");
}

TemperatureResult analyzeLowTemperature(const cv::Mat& rawImage, double tcamKelvin, double percentile) {
    return analyzeTemperatureRegion(rawImage, tcamKelvin, percentile, "low");
}

constexpr int TEXT_MARGIN_X = 10;
constexpr int TEXT_MARGIN_Y = 10;
constexpr int TEXT_GROUP_GAP = 4;
constexpr double TEXT_SCALE = 0.4;
constexpr int TEXT_THICKNESS = 1;

int textFirstBaselineY() {
    int baseline = 0;
    const cv::Size size = cv::getTextSize("Hg", cv::FONT_HERSHEY_SIMPLEX, TEXT_SCALE, TEXT_THICKNESS, &baseline);
    return TEXT_MARGIN_Y + size.height;
}

int textLineStep() {
    int baseline = 0;
    const cv::Size size = cv::getTextSize("Hg", cv::FONT_HERSHEY_SIMPLEX, TEXT_SCALE, TEXT_THICKNESS, &baseline);
    return size.height + baseline + TEXT_GROUP_GAP;
}

int textLastBaselineY(const cv::Mat& displayImage) {
    int baseline = 0;
    cv::getTextSize("Hg", cv::FONT_HERSHEY_SIMPLEX, TEXT_SCALE, TEXT_THICKNESS, &baseline);
    return displayImage.rows - TEXT_MARGIN_Y - baseline;
}

void putText(cv::Mat& displayImage, const std::string& text, cv::Point origin, cv::Scalar color) {
    cv::putText(displayImage, text, origin, cv::FONT_HERSHEY_SIMPLEX, TEXT_SCALE, color, TEXT_THICKNESS, cv::LINE_AA);
}

std::string textLine1(const TemperatureResult& result) {
    return titleText(result.type)
        + " raw:" + fixed2(result.raw_value)
        + " mean:" + fixed2(result.raw_region_mean)
        + " k100:" + fixed2(result.raw_region_k100_mean);
}

std::string textLine2(const TemperatureResult& result) {
    return titleText(result.type)
        + " temp:" + fixed2(result.temperature_c)
        + " mean:" + fixed2(result.temperature_region_mean_c)
        + " k100:" + fixed2(result.temperature_region_k100_mean_c);
}

void drawTemperature(
    cv::Mat& displayImage,
    const TemperatureResult& result,
    cv::Scalar regionColor,
    cv::Scalar markerColor,
    int textY,
    int text2Y
) {
    if (!result.ok) {
        return;
    }

    if (!result.region_mask.empty()) {
        displayImage.setTo(regionColor, result.region_mask);
    }

    cv::rectangle(
        displayImage,
        cv::Rect(result.rect_x, result.rect_y, result.rect_width, result.rect_height),
        regionColor,
        2
    );

    const cv::Point point(result.location_x, result.location_y);
    cv::drawMarker(displayImage, point, markerColor, cv::MARKER_CROSS, 15, 2);
    cv::circle(displayImage, point, 10, markerColor, 1, cv::LINE_AA);
    putText(displayImage, textLine1(result), cv::Point(TEXT_MARGIN_X, textY), regionColor);
    putText(displayImage, textLine2(result), cv::Point(TEXT_MARGIN_X, text2Y), regionColor);
}

void drawHighTemperature(cv::Mat& displayImage, const TemperatureResult& result) {
    drawTemperature(
        displayImage,
        result,
        cv::Scalar(0, 0, 255),
        cv::Scalar(0, 125, 255),
        textFirstBaselineY(),
        textLastBaselineY(displayImage) - textLineStep()
    );
}

void drawLowTemperature(cv::Mat& displayImage, const TemperatureResult& result) {
    drawTemperature(
        displayImage,
        result,
        cv::Scalar(255, 0, 0),
        cv::Scalar(255, 255, 0),
        textFirstBaselineY() + textLineStep(),
        textLastBaselineY(displayImage)
    );
}

void drawTcam(cv::Mat& displayImage, double tcamKelvin) {
    const int x = std::max(10, displayImage.cols - 110);
    const int y = std::max(20, displayImage.rows - 20);
    putText(displayImage, "Tcam:" + fixed2(tcamKelvin - 273.15), cv::Point(x, y), cv::Scalar(0, 0, 255));
}

ProcessFrameResult processFrame(
    const cv::Mat& frame,
    bool showHigh,
    bool showLow,
    int width,
    int height,
    int cropBorder,
    int minValue,
    int maxValue,
    double percentile
) {
    const cv::Mat fullRawImage = processFrameForDisplay(frame, width, height);
    const double tcamKelvin = getTcamKelvin(fullRawImage);

    cv::Mat rawImage;
    if (cropBorder > 0) {
        if (cropBorder * 2 >= fullRawImage.cols || cropBorder * 2 >= fullRawImage.rows) {
            throw std::invalid_argument("cropBorder is too large for this frame.");
        }
        rawImage = fullRawImage(cv::Rect(
            cropBorder,
            cropBorder,
            fullRawImage.cols - cropBorder * 2,
            fullRawImage.rows - cropBorder * 2
        )).clone();
    } else {
        rawImage = fullRawImage.clone();
    }

    cv::Mat displayImage = makeDisplayImage(rawImage, minValue, maxValue);
    drawTcam(displayImage, tcamKelvin);

    TemperatureResult high = emptyTemperatureResult("high", "", "not requested", tcamKelvin);
    TemperatureResult low = emptyTemperatureResult("low", "", "not requested", tcamKelvin);

    if (showHigh) {
        high = analyzeHighTemperature(rawImage, tcamKelvin, percentile);
        drawHighTemperature(displayImage, high);
    }
    if (showLow) {
        low = analyzeLowTemperature(rawImage, tcamKelvin, percentile);
        drawLowTemperature(displayImage, low);
    }

    ProcessFrameResult result;
    result.ok = true;
    result.tcam_kelvin = tcamKelvin;
    result.tcam_c = tcamKelvin - 273.15;
    result.raw_image = rawImage;
    result.display_image = displayImage;
    result.high = high;
    result.low = low;
    return result;
}

std::pair<int, int> updateDisplayWindow(const cv::Mat& rawImage, double sigma) {
    require2DImage(rawImage);

    const int centerRow = rawImage.rows / 2;
    const int centerCol = rawImage.cols / 2;
    const int halfRows = rawImage.rows / 4;
    const int halfCols = rawImage.cols / 4;

    const int x0 = std::max(0, centerCol - halfCols);
    const int y0 = std::max(0, centerRow - halfRows);
    const int x1 = std::min(rawImage.cols, centerCol + halfCols);
    const int y1 = std::min(rawImage.rows, centerRow + halfRows);
    if (x1 <= x0 || y1 <= y0) {
        return {DEFAULT_MIN_VALUE, DEFAULT_MAX_VALUE};
    }

    cv::Scalar mean;
    cv::Scalar stddev;
    cv::meanStdDev(rawImage(cv::Rect(x0, y0, x1 - x0, y1 - y0)), mean, stddev);

    const int maxValue = static_cast<int>(mean[0] + sigma * stddev[0]);
    const int minValue = static_cast<int>(mean[0] - sigma * stddev[0]);
    return maxValue == minValue ? std::pair<int, int>{DEFAULT_MIN_VALUE, DEFAULT_MAX_VALUE}
                                : std::pair<int, int>{minValue, maxValue};
}
