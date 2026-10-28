#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

void yolo_init();

//看全景，找行人（传 720p 大图）
void yolo_run_main(cv::Mat &frame);

//专门找车牌（传 360p 小图）
void yolo_run_plate(cv::Mat &frame, std::vector<cv::Rect> &out_plate_rects);
