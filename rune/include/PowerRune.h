#ifndef R_POWERRUNE_H
#define R_POWERRUNE_H

#include "Calculator.h"
#include "Detector.h"
#include "Param.h"
#include "Utility.h"

#ifdef Rune
#define CONFIG_PATH "../control/rune_config.yaml"
#define DH1JB_PATH "../drives/Distortions/camera_info_DH1.yaml"
#define DH2JB_PATH "../drives/Distortions/camera_info_DH2.yaml"
#define MDJB_PATH "../drives/Distortions/camera_info_MD.yaml"
#define HKJB_PATH "../drives/Distortions/camera_info_HK.yaml"

namespace power_rune {

class PowerRune {
   public:
    PowerRune();
    bool runOnce(const cv::Mat& image, double pitch, double yaw, double roll = 0.0);

   public:
    Param m_param;
    Detector m_detector;
    Calculator m_calculator;
};

}  // namespace power_rune

#endif
#endif
