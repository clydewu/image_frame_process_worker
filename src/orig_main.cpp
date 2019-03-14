//
//  main.cpp
//  opencv_test
//
//  Created by Clyde Wu on 2017/4/19.
//  Copyright © 2017年 Clyde Wu. All rights reserved.
//

#include <iostream>
#include <limits>

#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/ocl/ocl.hpp>

#include <amqp.h>
#include <amqp_framing.h>

#include "HOACVAgent.hpp"
#include "hoacv.pb.h"

#include "const.hpp"

using namespace ocl;
using namespace std;
using namespace cv;


typedef vector<Point> Contour;
typedef vector<vector<Point>> ContourSet;

//void matToString(Mat& image, string& output);
bool enableOpenGL(int deviceId);
double dist(Point x, Point y);
pair<Point, double> circleFromPoints(Point p1, Point p2, Point p3);
void createWindows(int winWidth, int winHeight, int winPerRow, bool doHsv, bool doConvGray, bool doInSkinRange, bool doBGSub, bool doDilateErode, bool doThreshold);
void getNextWinPos(int* x, int* y, int* count, int winPerRow, int width, int height);
bool isHeartShape(vector<Point>& polygon, vector<int>& hull, vector<Vec4i>& defects, Mat& image);


int main( int argc, char** argv ) {
    bool doLab = true;
    bool doConvGray = true;
    bool doInSkinRange = false;
    bool doBGSub = false;
    bool doDilateErode = true;
    bool doThreshold = true;

    int origWidth;
    int origHeight;
    int resizeWidth;
    int resizeHeight;
    
    Mat origImg;
    Mat rmqImg;
    Mat resizeImg;
    Mat hsvImg;
    Mat grayImg;
    Mat dilateImg;
    Mat erodeImg;
    Mat thresholdImg;
    
    Mat* tempImg;
    
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    
    string cv_in_queue = "tp.hoa.cv_in";
    cld::HOACVAgent agent("192.168.2.30", 5672);
    agent.Connect();
    agent.CreateQueue(cv_in_queue);

    enableOpenGL(0);

    VideoCapture videoCapture;
    videoCapture.open(0);

    origWidth = videoCapture.get(CV_CAP_PROP_FRAME_WIDTH);
    origHeight = videoCapture.get(CV_CAP_PROP_FRAME_HEIGHT);
    resizeWidth = origWidth / kResizeRatio;
    resizeHeight = origHeight / kResizeRatio;

    cout << "Input, width: " << origWidth << ", height: " << origHeight << endl;
    cout << "Output, width: " << resizeWidth << ", height: " << resizeHeight << endl;

    Rect roiRect((resizeWidth - kROIWidth) / 2, (resizeHeight - kROIHeight), kROIWidth, kROIHeight);

    createWindows(resizeWidth + kWindowHorizontalMargin, resizeHeight + kWindowVerticalMargin, kWindowPerRow, doLab, doConvGray, doInSkinRange, doBGSub, doDilateErode, doThreshold);

    for (;;) {
        videoCapture >> origImg;
        tempImg = &origImg;

        //-- Resize to reduce data
        resize(*tempImg, resizeImg, Size(resizeWidth, resizeHeight), 0, 0, INTER_NEAREST);
        tempImg = &resizeImg;
        
        
        //-- Send to RabbitMQ and get back
        int tmp_type = tempImg->type();
        int tmp_height = tempImg->rows;
        int tmp_width = tempImg->cols;
        
        agent.Send(cv_in_queue, tempImg->data, tempImg->total() * tempImg->elemSize());
        
        hoa::HOACVFrame cvFrame;
        agent.Recv(cv_in_queue, cvFrame);
        rmqImg = Mat(tmp_height, tmp_width, tmp_type, (unsigned char*)(cvFrame.matrix().data()));
        cvFrame.Clear();

        tempImg = &rmqImg;
        

        //-- Lab is better for detect contour
        if (doLab) {
            cvtColor(*tempImg, hsvImg, CV_BGR2Lab);
            imshow(kHSVWinName, hsvImg);
            tempImg = &hsvImg;
        }

        if (doConvGray) {
            cvtColor(*tempImg, grayImg, CV_BGR2GRAY);
            imshow(kConvGrayWinName, grayImg);
            tempImg = &grayImg;
        }

        //-- Do dilate and erode to emphasize edges
        if (doDilateErode) {
            dilate(*tempImg, dilateImg, Mat(), Point(-1, -1), 2);
            erode(dilateImg, erodeImg, Mat(), Point(-1, -1), 3);
            imshow(kDilateErodeWinName, erodeImg);
            tempImg = &erodeImg;
        }

        if (doThreshold) {
            threshold(*tempImg, thresholdImg, 80, 255, THRESH_BINARY|THRESH_OTSU);
            imshow(kThresholdWinName, thresholdImg);
            tempImg = &thresholdImg;
        }

        findContours(*tempImg, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE, Point(0, 0));

        vector<vector<Point>> contours_poly(contours.size());
        bool touchBoundary;
        for (unsigned int i = 0; i < contours.size(); i++) {
            if (contourArea(contours[i]) < kContourArea) {
                continue;
            }
            vector<Point>& contour = contours[i];
            touchBoundary = false;
            for (unsigned int k = 0; k < contour.size(); k++) {
                if(contour[k].x == 1 || contour[k].x == resizeWidth-2 || contour[k].y == 1 || contour[k].y == resizeHeight-2) {
                    touchBoundary = true;
                    break;
                }
            }
            if(touchBoundary) {
                continue;
            }

            //-- Contour to polygon
            approxPolyDP(contour, contours_poly[i], 8, true);
            drawContours(resizeImg, contours_poly, i, kBGRBlue, 2);

            //-- Polygon to hull
            vector<vector<int>> hullsInt(1);
            convexHull(contours_poly[i], hullsInt[0], false);

            //-- This is only used for draw
            vector<vector<Point>> hullsPoint(1);
            convexHull(contours_poly[i], hullsPoint[0], false);
            drawContours(resizeImg, hullsPoint, -1, kBGRGreen, 2);

            //-- Hull, polygon to deftects
            vector<Vec4i> defects;
            if(hullsInt[0].size() > 3) {
                convexityDefects(contours_poly[i], hullsInt[0], defects);
            }
            bool isHeart = false;
            if (defects.size() > 0) {
                for(unsigned int j = 0; j < defects.size(); j++) {
                    circle(resizeImg, contours_poly[i][defects[j][2]], 6, kBGRRed, 2, CV_AA);
                }

                isHeart = isHeartShape(contours_poly[i], hullsInt[0], defects, resizeImg);
            }
            if(isHeart) {
                putText(resizeImg, "Is heart!", Point(150,20), FONT_HERSHEY_SIMPLEX, 1, kBGRGreen);
            }
        }
        
        imshow(kResizeWinName, resizeImg);
        waitKey(10);
    }

    return 0;
}


bool enableOpenGL(int deviceId) {
    cout << "1" << endl;
    DevicesInfo devicesInfo;
    cout << "2" << endl;
    ocl::getOpenCLDevices(devicesInfo);
    cout << "3" << endl;
    for (auto dev: devicesInfo)
    {
        cout << dev->deviceName << endl;
    }
    //ocl::setDevice(devicesInfo[deviceId]);

    return true;
}


double dist(Point x, Point y) {
    return ((x.x - y.x) * (x.x - y.x)) + ((x.y - y.y) * (x.y - y.y));
}


pair<Point, double> circleFromPoints(Point p1, Point p2, Point p3) {
    double offset = pow(p2.x, 2) + pow(p2.y, 2);
    double bc = (pow(p1.x, 2) + pow(p1.y, 2) - offset) / 2.0;
    double cd = (offset - pow(p3.x, 2) - pow(p3.y, 2)) / 2.0;
    double det = (p1.x - p2.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p2.y);
    double TOL = 0.0000001;
    if (abs(det) < TOL)
    {
        // cout << "POINTS TOO CLOSE" << endl;
        return make_pair(Point(0, 0), 0);
    }

    double idet = 1 / det;
    double centerx = (bc * (p2.y - p3.y) - cd * (p1.y - p2.y)) * idet;
    double centery = (cd * (p1.x - p2.x) - bc * (p2.x - p3.x)) * idet;
    double radius = sqrt(pow(p2.x - centerx, 2) + pow(p2.y - centery, 2));

    return make_pair(Point(centerx, centery), radius);
}

void createWindows(int winWidth, int winHeight, int winPerRow, bool doHsv, bool doConvGray, bool doInSkinRange, bool doBGSub, bool doDilateErode, bool doThreshold)
{
    int count = 1;
    int x = 0;
    int y = 0;

    if (doHsv) {
        namedWindow(kHSVWinName);
        moveWindow(kHSVWinName, x, y);
        getNextWinPos(&x, &y, &count, winPerRow, winWidth, winHeight);
        count++;
    }

    if (doConvGray) {
        namedWindow(kConvGrayWinName);
        moveWindow(kConvGrayWinName, x, y);
        getNextWinPos(&x, &y, &count, winPerRow, winWidth, winHeight);
        count++;
    }

    if (doInSkinRange) {
        namedWindow(kInSkinRangeWinName);
        moveWindow(kInSkinRangeWinName, x, y);
        getNextWinPos(&x, &y, &count, winPerRow, winWidth, winHeight);
        count++;
    }

    if (doBGSub) {
        namedWindow(kBGSubWinName);
        moveWindow(kBGSubWinName, x, y);
        getNextWinPos(&x, &y, &count, winPerRow, winWidth, winHeight);
        count++;
    }

    //-- Do dilate and erode to emphasize edges
    if (doDilateErode) {
        namedWindow(kDilateErodeWinName);
        moveWindow(kDilateErodeWinName, x, y);
        getNextWinPos(&x, &y, &count, winPerRow, winWidth, winHeight);
        count++;
    }

    if (doThreshold) {
        namedWindow(kThresholdWinName);
        moveWindow(kThresholdWinName, x, y);
        getNextWinPos(&x, &y, &count, winPerRow, winWidth, winHeight);
        count++;
    }

    namedWindow(kResizeWinName);
    moveWindow(kResizeWinName, x, y);
}

void getNextWinPos(int* x, int* y, int* count, int winPerRow, int width, int height) {
    if ((*count % winPerRow) == 0) {
        *x = 0;
        *y += height;
    } else {
        *x += width;
    }
}

bool isHeartShape(vector<Point>& polygon, vector<int>& hull, vector<Vec4i>& defects, Mat& image) {
    int leftBoundary = numeric_limits<int>::max();
    int rightBoundary = 0;
    int topBoundary = numeric_limits<int>::max();
    int bottomBoundary = 0;

    int topDefectY = numeric_limits<int>::max();
    int topDefectIdx = 0;

    for (unsigned int i = 0; i < hull.size(); i++) {
        if (polygon[hull[i]].x < leftBoundary) {
            leftBoundary = polygon[hull[i]].x;
        }

        if (polygon[hull[i]].x > rightBoundary) {
            rightBoundary = polygon[hull[i]].x;
        }

        if (polygon[hull[i]].y < topBoundary) {
            topBoundary = polygon[hull[i]].y;
        }

        if (polygon[hull[i]].y > bottomBoundary) {
            bottomBoundary = polygon[hull[i]].y;
        }
    }

    if((double)(rightBoundary-leftBoundary)/(bottomBoundary-topBoundary) > 2 || (double)(rightBoundary-leftBoundary)/(bottomBoundary-topBoundary) < 0.5) {
        return false;
    }

    if(contourArea(polygon) < 0.5*(rightBoundary-leftBoundary)*(bottomBoundary-topBoundary)) {
        return false;
    }

    double defectDistance1 = cv::norm(polygon[defects[topDefectIdx][kDefectFarestPointIdx]] - polygon[defects[topDefectIdx][kDefectStartPointIdx]]);
    double defectDistance2 = cv::norm(polygon[defects[topDefectIdx][kDefectFarestPointIdx]] - polygon[defects[topDefectIdx][kDefectEndPointIdx]]);
    if(defectDistance1/defectDistance2 > 2 || defectDistance1/defectDistance2 < 0.5) {
        return false;
    }

    CvMoments moment;
    moment = moments(polygon);
    if(fabs(moment.m10/moment.m00 - polygon[defects[topDefectIdx][kDefectFarestPointIdx]].x) > 0.2*(rightBoundary-leftBoundary)) {
        circle(image, Point((moment.m10/moment.m00), (moment.m01/moment.m00)), 6, kBGRPurple);
        return false;
    }

    rectangle(image, Point(leftBoundary, topBoundary), Point(rightBoundary, bottomBoundary), kBGRPurple);
    cout << "top: " << topBoundary << ", bottom: " << bottomBoundary << ", left: " << leftBoundary << ", right: " << rightBoundary << endl;

    for (unsigned int i = 0; i < defects.size(); i++) {
        if (polygon[defects[i][kDefectFarestPointIdx]].y < topDefectY) {
            topDefectY = polygon[defects[i][kDefectFarestPointIdx]].y;
            topDefectIdx = i;
        }
    }
    circle(image, polygon[defects[topDefectIdx][kDefectFarestPointIdx]], 6, KBGRFuchsia, 2, CV_AA);
    circle(image, polygon[defects[topDefectIdx][kDefectStartPointIdx]], 6, KBGROliva, 2, CV_AA);
    circle(image, polygon[defects[topDefectIdx][kDefectEndPointIdx]], 6, KBGROliva, 2, CV_AA);
    line(image, polygon[defects[topDefectIdx][kDefectFarestPointIdx]], polygon[defects[topDefectIdx][kDefectStartPointIdx]], kBGRMaroon, 3, CV_AA);
    line(image, polygon[defects[topDefectIdx][kDefectFarestPointIdx]], polygon[defects[topDefectIdx][kDefectEndPointIdx ]], kBGRMaroon, 3, CV_AA);
    cout << "Start Point Y: " << polygon[defects[topDefectIdx][kDefectStartPointIdx]].y << ", End Point Y: " << polygon[defects[topDefectIdx][kDefectEndPointIdx]].y << endl;

    if (hull.size() < 4 || hull.size() > 8) {
        return false;
    }

    if (defects.size() > 1) {
        return false;
    }

    if (polygon[defects[topDefectIdx][kDefectStartPointIdx]].y != topBoundary &&
        polygon[defects[topDefectIdx][kDefectEndPointIdx]].y != topBoundary) {
        return false;
    }

    cout << "Hull: " << hull.size() << ", Defect: " << defects.size() << endl;
    return true;
}

