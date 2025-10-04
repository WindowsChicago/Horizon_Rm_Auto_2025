#include "PowerRune.h"

#include <thread>
#ifdef Rune

namespace power_rune {

extern std::mutex MUTEX;

// #ifdef DAHENG1
// PowerRune::PowerRune() : m_param{CONFIG_PATH,DH1JB_PATH} {}
// #endif

// #ifdef DAHENG2
// PowerRune::PowerRune() : m_param{CONFIG_PATH,DH2JB_PATH} {}
// #endif
// #ifdef MIDVISION
// PowerRune::PowerRune() : m_param{CONFIG_PATH,MDJB_PATH} {}
// #endif
// #ifdef HK
// PowerRune::PowerRune() : m_param{CONFIG_PATH,HKJB_PATH} {}
// #endif

PowerRune::PowerRune() : m_param{CONFIG_PATH} {}


bool PowerRune::runOnce(const cv::Mat& image, double pitch, double yaw, double roll) {
    Frame frame{image, std::chrono::steady_clock::now(), pitch, yaw, roll};
    
#if CONSOLE_OUTPUT >= 2
    MUTEX.lock();
    std::cout << "------" << std::endl;
    std::cout
        << "current time point: "
        << std::chrono::duration_cast<std::chrono::microseconds>(frame.m_time.time_since_epoch()).count()
        << std::endl;
    std::cout << "current row: " << roll << std::endl;
    std::cout << "current pitch: " << pitch << std::endl;
    std::cout << "current yaw: " << yaw << std::endl;
    MUTEX.unlock();
#endif
    if (m_detector.detect(frame) == false) {
        cv::namedWindow("Rune Mode", 0);
        cv::imshow("Rune Mode", image);
        //m_detector.visualize();
        return false;
    }
    auto cameraPoints{m_detector.getCameraPoints()};
    bool result = m_calculator.calculate(frame, cameraPoints);
    
#if SHOW_IMAGE >= 1
    if (result == true) {
        m_detector.drawTargetPoint(m_calculator.getPredictPixel());
       // m_detector.visualize();
    }
    m_detector.visualize();
    char key = cv::waitKey(1);
    if (key == ' ') {
        cv::waitKey(0);
    } else if (key == 'q') {
        std::exit(1);
    }
#endif
    return result;
}

}  // namespace power_rune
#endif