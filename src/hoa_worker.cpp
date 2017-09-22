//
//  main.cpp
//  opencv_test
//
//  Created by Clyde Wu on 2017/4/19.
//  Copyright © 2017年 Clyde Wu. All rights reserved.
//

#include <iostream>
#include <string>
#include <limits>

#include "log4cxx/basicconfigurator.h"
#include "log4cxx/propertyconfigurator.h"
#include "log4cxx/helpers/exception.h"
#include "log4cxx/patternlayout.h"
#include "log4cxx/fileappender.h"
#include "log4cxx/consoleappender.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/ocl/ocl.hpp>

#include <amqp.h>
#include <amqp_framing.h>

#include "AMQPBComm.hpp"
#include "hoa_frame.pb.h"

#include "const.hpp"

using namespace std;
using namespace ocl;
using namespace log4cxx;
using namespace cv;
using namespace hoa;


void init_logger(int gpu_id);
bool enableOpenCL(int deviceId);
bool find_heart(Mat& src_mat);
void preprocess_pipe(Mat& mat);
double dist(Point x, Point y);
pair<Point, double> circleFromPoints(Point p1, Point p2, Point p3);
void createWindows(int winWidth, int winHeight, int winPerRow, bool doLab, bool doConvGray, bool doDilateErode, bool doThreshold);
void getNextWinPos(int* x, int* y, int* count, int winPerRow, int width, int height);
bool isHeartShape(vector<Point>& polygon, vector<int>& hull, vector<Vec4i>& defects, Mat& image);

bool gui = false;
bool debug_mode = false;

LoggerPtr g_logger(LoggerPtr(Logger::getLogger(kLoggerName.c_str())));

int main ( int argc, char** argv)
{
    // argv[1] hostname
    // argv[2] port
    // argv[3] in_queue
    // argv[4] gpu_id
    
    if (argc < 5)
    {
        cout << "Usage: " << argv[0] << " hostname port in_queue gpu_id" << endl;
        return -1;
    }

    const string hostname(argv[1]);
    const int port(atoi(argv[2]));
    const string in_queue(argv[3]);
    const int gpu_id(atoi(argv[4]));

    init_logger(gpu_id);
    LOG4CXX_INFO(g_logger, "========== New HOA Worker start, gpu_id: " << gpu_id << " ==========");

    hoa::AMQPBComm comm(hostname, port);
    LOG4CXX_INFO(g_logger, "Connect to RabbitMQ, hostanem: " << hostname << ", port: " << port);
    comm.Connect();
    comm.CreateQueue(in_queue);

    enableOpenCL(gpu_id);

    while (true)
    {
        Mat src_mat;
        string src_uuid;
        int width;
        int height;
        int type;


        //-- Fetch HOAFrame from RabbitMQ
        hoa::HOAFrame in_frame;
        comm.Recv(in_queue, in_frame);
        LOG4CXX_DEBUG(g_logger, "Get one frame from " << in_frame.uuid());

        src_uuid = in_frame.uuid();
        height = in_frame.height();
        width = in_frame.width();
        type = in_frame.type();
        src_mat = Mat(height, width, type, (unsigned char*)(in_frame.data().data()));
        debug_mode = in_frame.debug_mode();
        in_frame.Clear();
        //-- Fetch Done

        bool is_heart = find_heart(src_mat);
        LOG4CXX_DEBUG(g_logger, "Does frame have heart? " << is_heart);

        //-- Send back result frame
        HOAFrame out_frame;
        out_frame.set_uuid(src_uuid);
        out_frame.set_data(src_mat.data, src_mat.total() * src_mat.elemSize());
        out_frame.set_width(width);
        out_frame.set_height(height);
        out_frame.set_type(type);
        out_frame.set_is_heart(is_heart);
        comm.Send(src_uuid, out_frame);
        out_frame.Clear();
        LOG4CXX_DEBUG(g_logger, "Send one frame to " << in_frame.uuid());

    }
    
    return 0;
}


void init_logger(int gpu_id)
{
    g_logger->setLevel(log4cxx::Level::getDebug());
    PatternLayoutPtr layout = new PatternLayout();
    layout->setConversionPattern("[%d{yyyy-MM-ddTHH:mm:ss,SSS}] [%-5p]: %m%n");

    FileAppenderPtr file_appender_ptr = new FileAppender();
    file_appender_ptr->setName("File");
    file_appender_ptr->setFile("/var/log/kurento-media-server/worker/GPU_" + to_string(gpu_id) + ".log");
    file_appender_ptr->setLayout(layout);
    helpers::Pool p;
    file_appender_ptr->activateOptions(p);

    ConsoleAppenderPtr cs_appender_ptr = new ConsoleAppender(layout);
    cs_appender_ptr->setName("Console");

    g_logger->addAppender(file_appender_ptr);
    g_logger->addAppender(cs_appender_ptr);
}


bool enableOpenCL(int deviceId)
{
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


bool find_heart(Mat& src_mat)
{
    Mat ana_mat;
    vector< vector<Point> > contours;
    vector< Vec4i > hierarchy;
    int width = src_mat.cols;
    int height = src_mat.rows;
    bool is_heart = false;

    src_mat.copyTo(ana_mat);
    preprocess_pipe(ana_mat);

    findContours(ana_mat, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE, Point(0, 0));

    vector< vector<Point> > contours_poly(contours.size());
    bool touchBoundary;
    for (unsigned int i = 0; i < contours.size(); i++)
    {
        if (contourArea(contours[i]) < kContourArea)
        {
            continue;
        }
        vector<Point>& contour = contours[i];
        touchBoundary = false;
        for (unsigned int k = 0; k < contour.size(); k++)
        {
            if (contour[k].x == 1 || contour[k].x == width - 2 || contour[k].y == 1 || contour[k].y == height - 2)
            {
                touchBoundary = true;
                break;
            }
        }

        if (touchBoundary)
        {
            continue;
        }

        //-- Contour to polygon
        approxPolyDP(contour, contours_poly[i], 8, true);
        if (debug_mode)
        {
            drawContours(src_mat, contours_poly, i, kBGRBlue, 2);
        }

        //-- Polygon to hull
        vector< vector<int> > hullsInt(1);
        convexHull(contours_poly[i], hullsInt[0], false);

        //-- This is only used for draw
        vector< vector<Point> > hullsPoint(1);
        convexHull(contours_poly[i], hullsPoint[0], false);
        if (debug_mode)
        {
            drawContours(src_mat, hullsPoint, -1, kBGRGreen, 2);
        }

        //-- Hull, polygon to deftects
        vector<Vec4i> defects;
        if (hullsInt[0].size() > 3)
        {
            convexityDefects(contours_poly[i], hullsInt[0], defects);
        }

        if (defects.size() > 0)
        {
            if (debug_mode)
            {
                for (unsigned int j = 0; j < defects.size(); j++)
                {
                    circle(src_mat, contours_poly[i][defects[j][2]], 6, kBGRRed, 2, CV_AA);
                }
            }

            if ((is_heart = isHeartShape(contours_poly[i], hullsInt[0], defects, src_mat)))
            {
                if (debug_mode)
                {
                    putText(src_mat, "Is heart!", Point(150, 20), FONT_HERSHEY_SIMPLEX, 1, kBGRGreen);
                }
            }
        }

    }

    return is_heart;
}


void preprocess_pipe(Mat& mat)
{
    bool doLab = true;
    bool doConvGray = true;
    bool doDilateErode = true;
    bool doThreshold = true;

    if (doLab)
    {
        cvtColor(mat, mat, CV_BGR2Lab);
        if (gui) imshow(kLABWinName, mat);
    }

    if (doConvGray)
    {
        cvtColor(mat, mat, CV_BGR2GRAY);
        if (gui) imshow(kConvGrayWinName, mat);
    }

    if (doDilateErode)
    {
        dilate(mat, mat, Mat(), Point(-1, -1), 2);
        erode(mat, mat, Mat(), Point(-1, -1), 3);
        if(gui) imshow(kDilateErodeWinName, mat);
    }

    if (doThreshold)
    {
        threshold(mat, mat, 80, 255, THRESH_BINARY|THRESH_OTSU);
        if (gui) imshow(kThresholdWinName, mat);
    }

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

void createWindows(int winWidth, int winHeight, int winPerRow, bool doLab, bool doConvGray, bool doBGSub, bool doDilateErode, bool doThreshold)
{
    int count = 1;
    int x = 0;
    int y = 0;

    if (doLab) {
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

