#ifndef __CONST_HPP__
#define __CONST_HPP__

#include <opencv2/core/core.hpp>

using namespace std;
using namespace cv;

const string kLoggerName = "HOA";

const Scalar kBGRBlue = Scalar(255, 0, 0);
const Scalar kBGRGreen = Scalar(0, 255, 0);
const Scalar kBGRRed = Scalar(0, 0, 255);
const Scalar kBGRYellow = Scalar(0, 255, 255);
const Scalar kBGRPurple = Scalar(128, 0, 128);
const Scalar KBGRFuchsia = Scalar(255, 0, 255);
const Scalar KBGROliva = Scalar(0, 128, 128);
const Scalar kBGRMaroon = Scalar(0, 0, 128);

const string kOrigWinName = "Orig";
const string kHSVWinName = "HSV";
const string kLABWinName = "LAB";
const string kConvGrayWinName = "Gray";
const string kInSkinRangeWinName = "SkinRange";
const string kBGSubWinName = "BackgroundSub";
const string kDilateErodeWinName = "Dilate Erode";
const string kThresholdWinName = "Threshold";
const string kResizeWinName = "Resize";

const int kWindowPerRow = 2;
const int kResizeRatio = 2;
const int kWindowVerticalMargin = 60;
const int kWindowHorizontalMargin = 20;

const int kContourArea = 2000;
const int kROIWidth = 150;
const int kROIHeight = 150;

const int kDefectStartPointIdx = 0;
const int kDefectEndPointIdx = 1;
const int kDefectFarestPointIdx = 2;
const int kDefectFarestDistIdx = 3;



#endif /*  __HEART_OF_ASIA_OPENCV_IMPL_HPP__ */
