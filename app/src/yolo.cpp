#include "yolo.h"
#include <opencv2/dnn.hpp>
#include "opencv2/core/mat.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "ztu/cv.hpp"
#include "ztu/nn/structures.hpp"
#include "ztu/nn/yolov5_ztopt_3out_postprocess.hpp"
#include "ztu/rk/ez_rknn.hpp"
#include <string>
#include <vector>
#include <iostream>
#include <cmath>

static const std::vector<std::string> classes_main = {"zebra", "person", "red", "green", "black", "yellow"};
static const std::vector<cv::Scalar> colors_main = {
    cv::Scalar(255, 0, 0), cv::Scalar(0, 165, 255), cv::Scalar(0, 0, 255), 
    cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 0), cv::Scalar(0, 255, 255)
};

static ztu::rk::EzRknn *rknn_main = nullptr; 
static ztu::rk::EzRknn *rknn_plate = nullptr;

void yolo_init() {
    if (!rknn_main) rknn_main = new ztu::rk::EzRknn("./best_rk3568.rknn", ztu::rk::EzRknn::Layout::CHW2HWC);
    if (!rknn_plate) rknn_plate = new ztu::rk::EzRknn("./plate_detect.rknn", ztu::rk::EzRknn::Layout::CHW2HWC);
}

inline bool rectangles_overlap(const cv::Rect& rect1, const cv::Rect& rect2) {
    return !(rect1.x > rect2.x + rect2.width || rect1.x + rect1.width < rect2.x ||
             rect1.y > rect2.y + rect2.height || rect1.y + rect1.height < rect2.y);
}

inline float sigmoid_fast(float x) { 
    return 1.0f / (1.0f + std::exp(-x)); 
}

// =========================================================================
// 阶段 1：跑行人红绿灯 (恢复 CPU 等比例 LetterBox 缩放，保证精度)
// =========================================================================
void yolo_run_main(cv::Mat &frame) {
    if (!rknn_main) yolo_init();
    auto model_w_main = rknn_main->input_attrs[0].dims[2];
    auto model_h_main = rknn_main->input_attrs[0].dims[1];
    
    // 【核心修复】：退回等比例缩放，保证画面不被压扁！
    auto resize_param_main = ztu::cv::calculateResize(frame.cols, frame.rows, model_w_main, model_h_main);
    auto resized_frame_main = ztu::cv::resize(frame, resize_param_main);
    cv::cvtColor(resized_frame_main, resized_frame_main, cv::COLOR_BGR2RGB);
    
    auto res_main = rknn_main->inference(resized_frame_main.data);
    ztu::nn::object_detect_result_list detections_main;
    ztu::nn::yolov5_ztopt_3out_postprocess::postprocess(
        const_cast<const float **>(res_main.data()), 0.25, 0.4, &detections_main, 
        model_w_main, model_h_main, classes_main.size());

    cv::Rect zebra_rect;
    bool has_zebra = false, has_green = false;
    std::vector<cv::Rect> person_rects;

    for (auto &det : detections_main.results) {
        auto [left, top] = ztu::cv::reverseMapPoint(det.box.left, det.box.top, resize_param_main);
        auto [right, bottom] = ztu::cv::reverseMapPoint(det.box.right, det.box.bottom, resize_param_main);
        cv::Rect rect(left, top, right - left, bottom - top);
        
        if (det.cls_id == 0) { zebra_rect = rect; has_zebra = true; } 
        else if (det.cls_id == 3) { has_green = true; } 
        else if (det.cls_id == 1) { person_rects.push_back(rect); }

        cv::Scalar color = colors_main[det.cls_id];
        cv::rectangle(frame, rect, color, 2);
        std::string label = classes_main[det.cls_id] + " " + std::to_string(static_cast<int>(det.prop * 100)) + "%";
        int baseline = 0;
        cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
        cv::rectangle(frame, cv::Point(rect.x, rect.y - text_size.height - 5), cv::Point(rect.x + text_size.width, rect.y), color, -1);
        cv::putText(frame, label, cv::Point(rect.x, rect.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    }

    if (has_zebra && has_green) {
        for (const auto& pr : person_rects) {
            if (rectangles_overlap(pr, zebra_rect)) {
                int cx = zebra_rect.x + zebra_rect.width / 2, cy = zebra_rect.y + zebra_rect.height / 2;
                int r = std::min(zebra_rect.width, zebra_rect.height) / 3;
                cv::circle(frame, cv::Point(cx, cy), r, cv::Scalar(0, 0, 255), -1);
                cv::circle(frame, cv::Point(cx, cy), r, cv::Scalar(255, 255, 255), 5);
                cv::putText(frame, "!", cv::Point(cx - r/2.5, cy + r/1.5), cv::FONT_HERSHEY_SIMPLEX, r/12.0, cv::Scalar(255, 255, 255), r/6, cv::LINE_AA);
                break;
            }
        }
    }
}

// =========================================================================
// 阶段 2：跑车牌定位 (恢复 CPU 等比例 LetterBox 缩放)
// =========================================================================
void yolo_run_plate(cv::Mat &frame, std::vector<cv::Rect> &out_plate_rects) {
    if (!rknn_plate) yolo_init();
    out_plate_rects.clear();

    auto model_w_plate = rknn_plate->input_attrs[0].dims[2]; 
    auto model_h_plate = rknn_plate->input_attrs[0].dims[1]; 
    
    // 【核心修复】：退回等比例缩放，这是精准坐标反推的命脉！
    auto resize_param_plate = ztu::cv::calculateResize(frame.cols, frame.rows, model_w_plate, model_h_plate);
    auto resized_frame_plate = ztu::cv::resize(frame, resize_param_plate);
    cv::cvtColor(resized_frame_plate, resized_frame_plate, cv::COLOR_BGR2RGB);
    
    auto res_plate = rknn_plate->inference(resized_frame_plate.data);
    float* pdata = (float*)res_plate[0]; 
    int num_dimensions = 14;   
    
    float conf_threshold = 0.35f; 
    float nms_threshold = 0.45f; 
    
    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;

    const float anchors[3][3][2] = {
        {{10, 13}, {16, 30}, {33, 23}},     
        {{30, 61}, {62, 45}, {59, 119}},    
        {{116, 90}, {156, 198}, {373, 326}} 
    };
    const int strides[3] = {8, 16, 32};
    const int grid_sizes[3] = {80, 40, 20}; 
    
    int offset = 0;
    for (int s = 0; s < 3; ++s) {
        int grid_size = grid_sizes[s];
        int stride = strides[s];
        
        for (int a = 0; a < 3; ++a) { 
            for (int y = 0; y < grid_size; ++y) {
                for (int x = 0; x < grid_size; ++x) {
                    
                    if (boxes.size() > 50) goto NMS_STAGE; 

                    float* ptr = pdata + (offset * num_dimensions);
                    offset++;

                    float box_conf = sigmoid_fast(ptr[4]);
                    if (box_conf < conf_threshold) continue; 

                    float cls_conf = sigmoid_fast(ptr[5]);
                    float score = box_conf * cls_conf;

                    if (score > conf_threshold) {
                        float cx = (sigmoid_fast(ptr[0]) * 2.0f - 0.5f + x) * stride;
                        float cy = (sigmoid_fast(ptr[1]) * 2.0f - 0.5f + y) * stride;
                        
                        float sw = sigmoid_fast(ptr[2]) * 2.0f;
                        float w = sw * sw * anchors[s][a][0];
                        
                        float sh = sigmoid_fast(ptr[3]) * 2.0f;
                        float h = sh * sh * anchors[s][a][1];

                        int left = static_cast<int>(cx - w / 2.0f);
                        int top = static_cast<int>(cy - h / 2.0f);
                        
                        if (w < 10 || h < 10) continue;

                        boxes.push_back(cv::Rect(left, top, static_cast<int>(w), static_cast<int>(h)));
                        confidences.push_back(score);
                    }
                }
            }
        }
    }
    
NMS_STAGE:
    if (boxes.empty()) return;

    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, conf_threshold, nms_threshold, indices);
    
    for (int idx : indices) {
        cv::Rect box = boxes[idx];
        auto [left, top] = ztu::cv::reverseMapPoint(box.x, box.y, resize_param_plate);
        auto [right, bottom] = ztu::cv::reverseMapPoint(box.x + box.width, box.y + box.height, resize_param_plate);
        out_plate_rects.push_back(cv::Rect(left, top, right - left, bottom - top));
    }
}
