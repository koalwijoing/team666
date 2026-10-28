#pragma once

#include <cstdint>
#include <math.h>
#include <stdlib.h>
#include <sys/time.h>
#include <vector>

#include <set>
#include <vector>

#include "structures.hpp"

namespace ztu {
namespace nn {
namespace yolov5_ztopt_3out_postprocess {
// Anchors extracted from best.pt (scaled by strides 8/16/32).
// stride 8:  (5.64,12.39),(7.86,18.50),(10.72,28.14)
// stride 16: (63.50,9.08),(15.92,41.31),(25.48,69.94)
// stride 32: (218.38,16.33),(49.91,100.31),(612.50,90.00)
const float anchor[3][6] = {
    {5.64062f, 12.39062f, 7.85547f, 18.5f, 10.71875f, 28.14062f},
    {63.5f, 9.07812f, 15.92188f, 41.3125f, 25.48438f, 69.9375f},
    {218.375f, 16.32812f, 49.90625f, 100.3125f, 612.5f, 90.0f}};

namespace {

// Common strides for YOLOv5
const int YOLOv5_STRIDES[] = {8, 16, 32};
const int NUM_STRIDES = sizeof(YOLOv5_STRIDES) / sizeof(YOLOv5_STRIDES[0]);

inline static int clamp(float val, int min, int max) {
  return val > min ? (val < max ? val : max) : min;
}

static float CalculateOverlap(float xmin0, float ymin0, float xmax0,
                              float ymax0, float xmin1, float ymin1,
                              float xmax1, float ymax1) {
  float w = fmax(0.f, fmin(xmax0, xmax1) - fmax(xmin0, xmin1) + 1.0);
  float h = fmax(0.f, fmin(ymax0, ymax1) - fmax(ymin0, ymin1) + 1.0);
  float i = w * h;
  float u = (xmax0 - xmin0 + 1.0) * (ymax0 - ymin0 + 1.0) +
            (xmax1 - xmin1 + 1.0) * (ymax1 - ymin1 + 1.0) - i;
  return u <= 0.f ? 0.f : (i / u);
}

static int nms(int validCount, std::vector<float> &outputLocations,
               std::vector<int> classIds, std::vector<int> &order, int filterId,
               float threshold) {
  for (int i = 0; i < validCount; ++i) {
    int n = order[i];
    if (n == -1 || classIds[n] != filterId) {
      continue;
    }
    for (int j = i + 1; j < validCount; ++j) {
      int m = order[j];
      if (m == -1 || classIds[m] != filterId) {
        continue;
      }
      float xmin0 = outputLocations[n * 4 + 0];
      float ymin0 = outputLocations[n * 4 + 1];
      float xmax0 = outputLocations[n * 4 + 0] + outputLocations[n * 4 + 2];
      float ymax0 = outputLocations[n * 4 + 1] + outputLocations[n * 4 + 3];

      float xmin1 = outputLocations[m * 4 + 0];
      float ymin1 = outputLocations[m * 4 + 1];
      float xmax1 = outputLocations[m * 4 + 0] + outputLocations[m * 4 + 2];
      float ymax1 = outputLocations[m * 4 + 1] + outputLocations[m * 4 + 3];

      float iou = CalculateOverlap(xmin0, ymin0, xmax0, ymax0, xmin1, ymin1,
                                   xmax1, ymax1);

      if (iou > threshold) {
        order[j] = -1;
      }
    }
  }
  return 0;
}

static int quick_sort_indice_inverse(std::vector<float> &input, int left,
                                     int right, std::vector<int> &indices) {
  float key;
  int key_index;
  int low = left;
  int high = right;
  if (left < right) {
    key_index = indices[left];
    key = input[left];
    while (low < high) {
      while (low < high && input[high] <= key) {
        high--;
      }
      input[low] = input[high];
      indices[low] = indices[high];
      while (low < high && input[low] >= key) {
        low++;
      }
      input[high] = input[low];
      indices[high] = indices[low];
    }
    input[low] = key;
    indices[low] = key_index;
    quick_sort_indice_inverse(input, left, low - 1, indices);
    quick_sort_indice_inverse(input, low + 1, right, indices);
  }
  return low;
}

static float sigmoid(float x) { return 1.0 / (1.0 + expf(-x)); }

static float unsigmoid(float y) { return -1.0 * logf((1.0 / y) - 1.0); }

inline static int32_t __clip(float val, float min, float max) {
  float f = val <= min ? min : (val >= max ? max : val);
  return f;
}

static int8_t qnt_f32_to_affine(float f32, int32_t zp, float scale) {
  float dst_val = (f32 / scale) + zp;
  int8_t res = (int8_t)__clip(dst_val, -128, 127);
  return res;
}

static uint8_t qnt_f32_to_affine_u8(float f32, int32_t zp, float scale) {
  float dst_val = (f32 / scale) + zp;
  uint8_t res = (uint8_t)__clip(dst_val, 0, 255);
  return res;
}

static float deqnt_affine_to_f32(int8_t qnt, int32_t zp, float scale) {
  return ((float)qnt - (float)zp) * scale;
}

static float deqnt_affine_u8_to_f32(uint8_t qnt, int32_t zp, float scale) {
  return ((float)qnt - (float)zp) * scale;
}

static int process_fp32(float *input, const float *anchor, int grid_h,
                        int grid_w, int height, int width, int stride,
                        std::vector<float> &boxes, std::vector<float> &objProbs,
                        std::vector<int> &classId, float threshold,
                        const int prop_box_size, const int class_nums) {
  int validCount = 0;
  int grid_len = grid_h * grid_w;

  for (int a = 0; a < 3; a++) {
    for (int i = 0; i < grid_h; i++) {
      for (int j = 0; j < grid_w; j++) {
        float box_confidence =
            sigmoid(input[(prop_box_size * a + 4) * grid_len + i * grid_w + j]);
        if (box_confidence >= threshold) {
          int offset = (prop_box_size * a) * grid_len + i * grid_w + j;
          float *in_ptr = input + offset;
          float box_x = sigmoid(*in_ptr) * 2.0 - 0.5;
          float box_y = sigmoid(in_ptr[grid_len]) * 2.0 - 0.5;
          float box_w = sigmoid(in_ptr[2 * grid_len]) * 2.0;
          float box_h = sigmoid(in_ptr[3 * grid_len]) * 2.0;
          box_x = (box_x + j) * (float)stride;
          box_y = (box_y + i) * (float)stride;
          box_w = box_w * box_w * (float)anchor[a * 2];
          box_h = box_h * box_h * (float)anchor[a * 2 + 1];
          box_x -= (box_w / 2.0);
          box_y -= (box_h / 2.0);

          float maxClassProbs = sigmoid(in_ptr[5 * grid_len]);
          int maxClassId = 0;
          for (int k = 1; k < class_nums; ++k) {
            float prob = sigmoid(in_ptr[(5 + k) * grid_len]);
            if (prob > maxClassProbs) {
              maxClassId = k;
              maxClassProbs = prob;
            }
          }
          if (maxClassProbs > threshold) {
            objProbs.push_back(maxClassProbs * box_confidence);
            classId.push_back(maxClassId);
            validCount++;
            boxes.push_back(box_x);
            boxes.push_back(box_y);
            boxes.push_back(box_w);
            boxes.push_back(box_h);
          }
        }
      }
    }
  }
  return validCount;
}

} // anonymous namespace

/**
 * @brief Post-processes the FP32 output tensors of a standard YOLOv5 model (3
 * outputs).
 *
 * @param outputs Array of pointers to the raw FP32 output tensor data.
 * @param conf_threshold Confidence threshold for filtering detections.
 * @param nms_threshold Non-Maximum Suppression threshold (IoU based).
 * @param od_results Pointer to the output structure to store final detection
 * results.
 * @param model_in_w Width of the model input tensor.
 * @param model_in_h Height of the model input tensor.
 * @param num_classes Number of object classes the model predicts.
 * @return int 0 if successful, non-zero otherwise.
 */
static int postprocess(const float *outputs[], float conf_threshold,
                       float nms_threshold,
                       object_detect_result_list *od_results,
                       int model_in_w = 640, int model_in_h = 640,
                       int num_classes = 80) {
  const int prop_box_size = 5 + num_classes;
  std::vector<float> filterBoxes;
  std::vector<float> objProbs;
  std::vector<int> classId;
  int validCount = 0;
  od_results->results.clear();

  // Calculate grid sizes based on input dimensions and strides
  std::vector<int> grid_hs(NUM_STRIDES);
  std::vector<int> grid_ws(NUM_STRIDES);
  for (int i = 0; i < NUM_STRIDES; ++i) {
    grid_hs[i] = model_in_h / YOLOv5_STRIDES[i];
    grid_ws[i] = model_in_w / YOLOv5_STRIDES[i];
  }

  for (int i = 0; i < NUM_STRIDES; i++) {
    int grid_h = grid_hs[i];
    int grid_w = grid_ws[i];
    int stride = YOLOv5_STRIDES[i];
    validCount +=
        process_fp32((float *)outputs[i], anchor[i], grid_h, grid_w, model_in_h,
                     model_in_w, stride, filterBoxes, objProbs, classId,
                     conf_threshold, prop_box_size, num_classes);
  }

  // no object detect
  if (validCount <= 0) {
    return 0;
  }
  std::vector<int> indexArray;
  for (int i = 0; i < validCount; ++i) {
    indexArray.push_back(i);
  }
  quick_sort_indice_inverse(objProbs, 0, validCount - 1, indexArray);

  std::set<int> class_set(std::begin(classId), std::end(classId));

  for (auto c : class_set) {
    nms(validCount, filterBoxes, classId, indexArray, c, nms_threshold);
  }

  od_results->results.reserve(validCount);

  /* box valid detect target */
  for (int i = 0; i < validCount; ++i) {
    if (indexArray[i] == -1) {
      continue;
    }
    int n = indexArray[i];

    float x1 = filterBoxes[n * 4 + 0];
    float y1 = filterBoxes[n * 4 + 1];
    float x2 = x1 + filterBoxes[n * 4 + 2];
    float y2 = y1 + filterBoxes[n * 4 + 3];
    int id = classId[n];
    float obj_conf = objProbs[i];

    object_detect_result result;
    result.box.left = (int)(clamp(x1, 0, model_in_w));
    result.box.top = (int)(clamp(y1, 0, model_in_h));
    result.box.right = (int)(clamp(x2, 0, model_in_w));
    result.box.bottom = (int)(clamp(y2, 0, model_in_h));
    result.prop = obj_conf;
    result.cls_id = id;
    od_results->results.push_back(result);
  }
  od_results->count = od_results->results.size();
  return 0;
}
} // namespace yolov5_ztopt_3out_postprocess
} // namespace nn
} // namespace ztu
