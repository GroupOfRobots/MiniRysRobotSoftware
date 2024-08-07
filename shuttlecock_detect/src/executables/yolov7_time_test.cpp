#include "shuttlecock_detect/helpers/Yolov7.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <stdio.h>
#include <vector>
#include <chrono>

YoloV7 yolov7;
int target_size = 420;

int main(int argc, char** argv)
{
    yolov7.load(target_size, "best100ep1inp.ncnn.param", "best100ep1inp.ncnn.bin");
    cv::Mat m = cv::imread("/home/Jakub/Documents/test/image_29_1717764171-923069_jpg.rf.d3d04ad129cfd8cdbcc8a5b3e8ccea9f.jpg", 1);
    if (m.empty())
    {
        fprintf(stderr, "cv::imread failed\n");
        return -1;
    }
    std::vector<Object> objects;
    auto beg = std::chrono::high_resolution_clock::now();
    yolov7.detect(m, objects, 0.1f, 0.1f);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - beg);
    std::cout<< duration.count() <<std::endl;
    std::cout<< objects.size() << std::endl;
    yolov7.draw(m, objects);
    cv::imshow("RPi4 - 1.95 GHz - 2 GB ram",m);
//    cv::imwrite("test.jpg",m);
    cv::waitKey(0);
    return 0;
}
