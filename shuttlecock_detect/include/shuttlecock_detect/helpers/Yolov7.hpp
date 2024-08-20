#ifndef YOLO_H
#define YOLO_H
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <net.h>
#include <string>
#include "cpu.h"

struct Object
{
    cv::Rect_<float> rect;
    int label;
    float prob;
};

class YoloV7
{
private:
    ncnn::Net yolo;
    int target_size;
    float norm_vals[3];
    int image_w;
    int image_h;
    int in_w;
    int in_h;
public:
    YoloV7();
    int load(int target_size, std::string weight_param,  std::string weight_bin);
    int detect(const cv::Mat& rgb, std::vector<Object>& objects, float prob_threshold = 0.25f, float nms_threshold = 0.45f);
    int draw(cv::Mat& rgb, const std::vector<Object>& objects);
};

#endif // YOLO_H
