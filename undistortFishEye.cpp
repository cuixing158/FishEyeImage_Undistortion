/**
* @file        :undistortImg.cpp
* @brief       :由鱼眼镜头生产厂家提供的超广角镜头进行去畸变，并估计获得相机内参
* @details     :This is the detail description.
* @date        :2022/06/21 09:26:33
* @author      :cuixingxing(cuixingxing150@gmail.com)
* @version     :1.0
*
* @copyright Copyright (c) 2022
*
*/
#include <iostream>
#include <fstream>
#include <math.h>
#include "opencv2/opencv.hpp"
// #include "CommandParser.h"

// 读取csv数据为二维矩阵
cv::Mat readCSV(std::string csvFile) {
    std::ifstream inFile(csvFile, std::ios::in);
    if (!inFile) {
        std::cout << "打开文件失败！" << std::endl;
        exit(1);
    }
    std::vector<std::vector<double> > all_data;
    std::string lineStr;
    std::getline(inFile, lineStr);
    while (std::getline(inFile, lineStr)) {
        std::vector<double> values;
        std::stringstream temp(lineStr);
        std::string single_value;
        while (std::getline(temp, single_value, ',')) {
            values.push_back(atof(single_value.c_str()));
        }
        all_data.push_back(values);
    }

    cv::Mat vect = cv::Mat::zeros((int)all_data.size(), (int)all_data[0].size(), CV_64FC1);
    for (int row = 0; row < (int)all_data.size(); row++) {
        for (int cols = 0; cols < (int)all_data[0].size(); cols++) {
            vect.at<double>(row, cols) = all_data[row][cols];
        }
    }
    return vect;
}

// 等同于matlab的interp1函数
float interp1(std::vector<float>& xData, std::vector<float>& yData, float x, bool extrapolate = true) {
    int size = xData.size();

    int i = 0;
    if (x >= xData[size - 2]) {
        i = size - 2;
    } else {
        while (x > xData[i + 1]) i++;
    }
    float xL = xData[i], yL = yData[i], xR = xData[i + 1], yR = yData[i + 1];
    if (!extrapolate) {
        if (x < xL) yR = yL;
        if (x > xR) yL = yR;
    }
    float dydx = (yR - yL) / (xR - xL);
    return yL + dydx * (x - xL);  // linear interpolation
}

/**
* @brief       由无畸变点转换为畸变点
* @details     此函数工作在原始比例大小图像上
* @param[in]   cameraData input argument description.
* @param[in]   undistortPt 无畸变图像上的理论点像素坐标.
* @param[in]   f 焦距，单位像素,其中 fx=fy=f.
* @param[in]   h 畸变图像的高（像素）.
* @param[in]   w 畸变图像的宽（像素）
* @param[in]   H 矫正后全局图像的高.
* @param[in]   W 矫正后全局图像的宽.
* @param[out]  outArgName output argument description.
* @return      对应畸变图像上像素点坐标
* @retval      返回值类型
* @par 标识符
*     保留
* @par 其它
*
* @par 修改日志
*      cuixingxing于2022/06/22创建
*/
cv::Point2d forwardPt(cv::Mat cameraData, cv::Point2d undistortPt, float f, float h, float w, float H, float W) {
    cv::Point2d relP = undistortPt - cv::Point2d(W, H) / 2;
    float dist = cv::norm(relP);
    float focalLen = f;
    float ang = atan(dist / focalLen) * 180 / CV_PI;
    float minAng = 180;
    float currScale = 1.0;
    double* curr_angle_d = cameraData.ptr<double>(0, 0);
    size_t index = 0;
    for (size_t i = 0; i < cameraData.rows; i++) {
        double absAng = cv::abs(*curr_angle_d - ang);
        if (absAng < minAng) {
            minAng = absAng;
            index = i;
        } else {
            break;
        }
        if (i != cameraData.rows - 1) {
            curr_angle_d = curr_angle_d + cameraData.cols;
        }
    }
    currScale = *(curr_angle_d + 5);  //currScale = cameraData.at<double>(index, 5);
    relP /= currScale;
    relP += cv::Point2d(w / 2, h / 2);
    return relP;
}

cv::Point2d backforwardPt(cv::Mat cameraData, cv::Point2d distortPt, float f, float h, float w, float H, float W) {
    cv::Point2d relP = distortPt - cv::Point2d(w, h) / 2;
    float dist = cv::norm(relP);
    float focalLen = f;
    float ang = atan(dist / focalLen) * 180 / CV_PI;
    float minAng = 180;
    float currScale = 1.0;
    double* curr_angle_d = cameraData.ptr<double>(0, 1);
    size_t index = 0;
    for (size_t i = 0; i < cameraData.rows; i++) {
        double absAng = cv::abs(*curr_angle_d - ang);
        if (absAng < minAng) {
            minAng = absAng;
            index = i;
        } else {
            break;
        }
        if (i != cameraData.rows - 1) {
            curr_angle_d = curr_angle_d + cameraData.cols;
        }
    }
    currScale = *(curr_angle_d + 4);  //cameraData.at<double>(index, 5);
    relP *= currScale;
    relP += cv::Point2d(W / 2, H / 2);
    return relP;
}

/**
* @brief       用于鱼眼图像去畸变
* @details     This is the detail description.
* @param[in]   fisheyeImg 鱼眼畸变图像，1920*1080大小.
* @param[in]   K 内参K,形如[fx,0,cx;0,fy,cy;0,0,1],单位为像素
* @param[in]   cameraData  鱼眼镜头厂商提供的查询表格数据,每列分别为入射角，出射角，投影长度，理论长度，畸变，尺度（ref_height/real_height）
* @param[in]   OutputView  指定为"same","valid","full"的一种
* @param[in]   scalarRatio：无畸变图像缩放因子比例，小于1为缩小输出图像，等于1没有缩放，大于1为放大图像.一般调整小于1，减少计算量
* @param[out]  outArgName output argument description.
* @return      矫正后的图像
* @retval      cv::Mat
* @par 标识符
*     保留
* @par 其它
*
* @par 修改日志
*      cuixingxing于2022/06/22创建
*/
cv::Mat undistortFishEyeImg(cv::Mat fisheyeImg, cv::Mat K, cv::Mat cameraData, cv::Mat& outputMapx, cv::Mat& outputMapy, std::string OutputView = "valid", double scalarRatio = 0.25) {
    if (fisheyeImg.channels() != 1) {
        cv::cvtColor(fisheyeImg, fisheyeImg, cv::COLOR_BGR2GRAY);
    }
    static float h, w, H, W, focalLen;
    static bool isInit = false;
    if (!isInit) {
        // 获取矫正图像的宽和高，H,W
        h = fisheyeImg.rows;
        w = fisheyeImg.cols;
        focalLen = K.at<double>(0, 0);
        double cx = K.at<double>(0, 2);
        double cy = K.at<double>(1, 2);
        double dTheta_d = atan(cv::sqrt(cx * cx + cy * cy) / focalLen) * 180 / CV_PI;
        double minAng = 180;
        double currScale = 1.0;
        for (size_t i = 0; i < cameraData.rows; i++) {
            double curr_angle_d = cameraData.at<double>(i, 1);
            if (cv::abs(curr_angle_d - dTheta_d) < minAng) {
                minAng = cv::abs(curr_angle_d - dTheta_d);
                currScale = cameraData.at<double>(i, 5);
            }
        }
        std::cout << "图像顶点最大畸变处与表格误差的角度误差为（度）：" << minAng << std::endl;
        H = currScale * h;
        W = currScale * w;
        isInit = true;
    }

    // 插值求解
    int sH = scalarRatio * H, sW = scalarRatio * W;
    cv::Point2d offsetPt = cv::Point2d(0, 0);
    if (OutputView == "same") {
        sH = scalarRatio * h;
        sW = scalarRatio * w;
        offsetPt = cv::Point2d(W - w, H - h) / 2.0;
    } else if (OutputView == "valid") {
        cv::Point2d topmiddlePt = backforwardPt(cameraData, cv::Point2d(w / 2, 0), focalLen, h, w, H, W);
        cv::Point2d leftmiddlePt = backforwardPt(cameraData, cv::Point2d(0, h / 2), focalLen, h, w, H, W);
        sW = 2 * scalarRatio * (topmiddlePt.x - leftmiddlePt.x);
        sH = 2 * scalarRatio * (leftmiddlePt.y - topmiddlePt.y);
        offsetPt = cv::Point2d(leftmiddlePt.x, topmiddlePt.y);
    } else  // "full"
    {
        sH = sH;
        sW = sW;
        offsetPt = cv::Point2d(0.0, 0.0);
    }

    cv::Mat undistortImg = cv::Mat::zeros(sH, sW, CV_8UC1);
    cv::Mat matMapx = cv::Mat::zeros(sH, sW, CV_32FC1) - 1;
    cv::Mat matMapy = cv::Mat::zeros(sH, sW, CV_32FC1) - 1;

    cv::Rect validRect = cv::Rect(0, 0, w, h);
    uchar* fisheyePtr = fisheyeImg.ptr<uchar>(0, 0);

    for (size_t i = 0; i < sH; i++) {
        uchar* d = undistortImg.ptr<uchar>(i);
        float* fMapxPtr = matMapx.ptr<float>(i);
        float* fMapyPtr = matMapy.ptr<float>(i);
        for (size_t j = 0; j < sW; j++) {
            cv::Point2d currPt = cv::Point2d(j, i) / scalarRatio + offsetPt;
            cv::Point2d projectPt = forwardPt(cameraData, currPt, focalLen, h, w, H, W);
            cv::Point proPt = cv::Point(projectPt.x, projectPt.y);
            if (validRect.contains(proPt)) {
                d[j] = *(fisheyePtr + proPt.y * (int)w + proPt.x);  //fisheyeImg(proPt.y, proPt.x);
                fMapxPtr[j] = proPt.x;
                fMapyPtr[j] = proPt.y;
            }
        }
    }
    outputMapx = matMapx;
    outputMapy = matMapy;
    return undistortImg;
}

cv::Point2d distortPt2undistortPt(cv::Point2d p1, cv::Mat realHeight, cv::Mat refHeight) {
    float dDistortD1 = cv::sqrt(p1.x * p1.x + p1.y * p1.y);
    std::vector<float> xdata{realHeight}, ydata{refHeight};
    float dundistortD1 = interp1(xdata, ydata, dDistortD1, false);
    cv::Point2d p2 = p1 * dundistortD1 / dDistortD1;
    return p2;
}

void meshgrid(const cv::Range& xgv, const cv::Range& ygv, cv::Mat& X, cv::Mat& Y) {
    std::vector<int> t_x, t_y;
    for (int i = xgv.start; i <= xgv.end; i++) t_x.push_back(i);
    for (int j = ygv.start; j <= ygv.end; j++) t_y.push_back(j);

    cv::repeat(cv::Mat(t_x).t(), t_y.size(), 1, X);
    cv::repeat(cv::Mat(t_y), 1, t_x.size(), Y);
    X.convertTo(X, CV_32FC1);
    Y.convertTo(Y, CV_32FC1);
}

cv::Mat undistortFishEyeImgFast(cv::Mat fisheyeImg, cv::Mat K, cv::Mat cameraData, cv::Mat& outputMapx,
                                cv::Mat& outputMapy, std::string OutputView = "valid", double scalarRatio = 0.25) {
    if (fisheyeImg.channels() != 1) {
        cv::cvtColor(fisheyeImg, fisheyeImg, cv::COLOR_BGR2GRAY);
    }
    // 获取矫正图像的宽和高，H,W
    int h = fisheyeImg.rows;
    int w = fisheyeImg.cols;
    double focalLen = K.at<double>(0, 0);
    double cx = K.at<double>(0, 2);
    double cy = K.at<double>(1, 2);
    cv::Mat realHeight = cv::Mat::zeros(cameraData.rows, 1, CV_64FC1);
    cv::Mat refHeight = cv::Mat::zeros(cameraData.rows, 1, CV_64FC1);

    for (size_t i = 0; i < cameraData.rows; i++) {
        realHeight.at<double>(i) = focalLen * tan(cameraData.at<double>(i, 1) * CV_PI / 180);
        refHeight.at<double>(i) = focalLen * tan(cameraData.at<double>(i, 0) * CV_PI / 180);
    }

    // mode
    int sH, sW;
    if (OutputView == "same") {
        sH = scalarRatio * h;
        sW = scalarRatio * w;
    } else if (OutputView == "valid") {
        cv::Point2d topmiddleRelPt = cv::Point2d(w / 2, 0) - cv::Point2d(cx, cy);
        cv::Point2d leftmiddleRelPt = cv::Point2d(0, h / 2) - cv::Point2d(cx, cy);
        cv::Point2d p1 = distortPt2undistortPt(topmiddleRelPt, realHeight, refHeight);
        cv::Point2d p2 = distortPt2undistortPt(leftmiddleRelPt, realHeight, refHeight);
        sW = 2 * scalarRatio * cv::abs(p1.x - p2.x);
        sH = 2 * scalarRatio * cv::abs(p1.y - p2.y);
    } else  // "full"
    {
        cv::Point2d downrightRelPt = cv::Point2d(w, h) - cv::Point2d(cx, cy);
        cv::Point2d p1 = distortPt2undistortPt(downrightRelPt, realHeight, refHeight);
        sW = 2 * scalarRatio * p1.x;
        sH = 2 * scalarRatio * p1.y;
    }

    cv::Mat undistortImg;
    cv::Mat matMapx = cv::Mat::zeros(sH, sW, CV_32FC1) - 1;
    cv::Mat matMapy = cv::Mat::zeros(sH, sW, CV_32FC1) - 1;
    cv::Mat X, Y, x, y;
    meshgrid(cv::Range(0, sW), cv::Range(0, sH), X, Y);
    X = (X - sW / 2.0) / scalarRatio;
    Y = (Y - sH / 2.0) / scalarRatio;
    cv::Mat undistortImgDist;
    cv::sqrt(X.mul(X) + Y.mul(Y), undistortImgDist);
    cv::Mat distortImgDist = cv::Mat::zeros(undistortImgDist.rows, undistortImgDist.cols, CV_32FC1);
    std::vector<float> vecXdata{refHeight}, vecYdata{realHeight};
    // interpolation
    for (size_t i = 0; i < sH; i++) {
        float* querydata = undistortImgDist.ptr<float>(i);
        float* data = distortImgDist.ptr<float>(i);
        for (size_t j = 0; j < sW; j++) {
            data[j] = interp1(vecXdata, vecYdata, querydata[j], false);
        }
    }

    x = X.mul(distortImgDist / undistortImgDist) + w / 2.0;
    y = Y.mul(distortImgDist / undistortImgDist) + h / 2.0;
    cv::remap(fisheyeImg, undistortImg, x, y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar::all(0));

    outputMapx = x;
    outputMapy = y;
    return undistortImg;
}

cv::Mat getIntrinsicMatrix(int fisheyeHeight, int fisheyeWidth, cv::Mat cameraData, float sensorratio = 0.0029) {
    // 由镜头厂商提供的cameraData和sensorratio计算焦距（像素）
    double focalLen = 0.0;
    double nums = 0;
    for (size_t i = 0; i < cameraData.rows; i++) {
        double refH = cameraData.at<double>(i, 2);
        double angle = cameraData.at<double>(i, 0);
        double temp = refH / tan(angle * CV_PI / 180);
        if (isnan(temp) || isinf(temp)) {
            continue;
        }
        nums++;
        focalLen += temp;
    }
    focalLen = focalLen / (nums * sensorratio);
    double h = static_cast<double>(fisheyeHeight);
    double w = static_cast<double>(fisheyeWidth);
    double cx = w / 2.0;
    double cy = h / 2.0;
    cv::Mat intrinsic = (cv::Mat_<double>(3, 3) << focalLen, 0, cx, 0, focalLen, cy, 0, 0, 1);
    return intrinsic;
}

int main(int argc, const char** argv) {
    // 准备你的数据
    std::string imgPath = "images/original.png";  // 鱼眼图像
    std::string fisheyeCfg = "fisheye.csv";       // 厂家畸变表格

    // 估计相机内参矩阵
    cv::Mat oriImg = cv::imread(imgPath, 0);
    cv::Mat cameraData = readCSV(fisheyeCfg);
    float sensorratio = 0.003;  // 每个像素为0.003毫米
    cv::Mat intrinsic = getIntrinsicMatrix(oriImg.rows, oriImg.cols, cameraData, sensorratio);

    // 厂家畸变表格数据cameraData适当计算
    cv::Mat outAngles = cv::Mat::zeros(cameraData.rows, 1, CV_64F);
    cv::Mat scales = cv::Mat::zeros(cameraData.rows, 1, CV_64F);
    double* p = outAngles.ptr<double>(0);
    double* s = scales.ptr<double>(0);
    for (size_t i = 0; i < cameraData.rows; i++) {
        p[i] = 180 / CV_PI * atan2(cameraData.at<double>(i, 1), sensorratio * intrinsic.at<double>(0, 0));
        s[i] = cameraData.at<double>(i, 2) / cameraData.at<double>(i, 1);
    }
    std::vector<cv::Mat> temp = {cameraData.colRange(0, 1), outAngles, cameraData.colRange(1, 4), scales};
    cv::hconcat(temp, cameraData);
    std::cout << cameraData.rowRange(cv::Range(0, 5)) << std::endl;  // 预览部分数据

    // 去畸变
    cv::Mat matMapx, matMapy;  // 方便后续视频帧直接映射操作
    cv::Mat undistortImg = undistortFishEyeImgFast(oriImg, intrinsic, cameraData, matMapx, matMapy, "same");

    cv::imshow("fisheye image", oriImg);
    cv::imshow("undistort image using table", undistortImg);
    cv::waitKey(0);

    return 0;
}
