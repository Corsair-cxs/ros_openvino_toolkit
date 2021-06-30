/*
 * Copyright (c) 2018 Intel Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/**
 * @brief A header file with declaration for FaceDetectionModel Class
 * @file face_detection_model.h
 */
#ifndef DYNAMIC_VINO_LIB_MODELS_FACE_DETECTION_MODEL_H
#define DYNAMIC_VINO_LIB_MODELS_FACE_DETECTION_MODEL_H
#include <string>
#include "dynamic_vino_lib/models/base_model.h"
// #include "dynamic_vino_lib/inferences/face_detection.h"
#include "dynamic_vino_lib/results/face_detection_result.h"
#include "dynamic_vino_lib/engines/engine_manager.h"

namespace Models
{
/**
 * @class FaceDetectionModel
 * @brief This class generates the face detection model.
 */
class FaceDetectionModel : public BaseModel
{
  // using Result = dynamic_vino_lib::FaceDetectionResult;
public:
  FaceDetectionModel();
  FaceDetectionModel(const std::string& model_loc, int batch_size = 1);

  bool fetchResults(const std::shared_ptr<Engines::Engine>& engine,
                    std::vector<dynamic_vino_lib::FaceDetectionResult>& results, const float& confidence_thresh = 0.3,
                    const bool& enable_roi_constraint = false);

  bool enqueue(const std::shared_ptr<Engines::Engine>& engine, const cv::Mat& frame,
               const cv::Rect& input_frame_loc);

  bool matToBlob(const cv::Mat& orig_image, const cv::Rect&, float scale_factor, int batch_index,
                 const std::shared_ptr<Engines::Engine>& engine);
  void adjustBoundingBox(cv::Rect& boundingBox);

  /**
   * @brief Get the name of this detection model.
   * @return Name of the model.
   */
  const std::string getModelCategory() const override;

  bool updateLayerProperty(InferenceEngine::CNNNetReader::Ptr) override;
};
}  // namespace Models
#endif  // DYNAMIC_VINO_LIB_MODELS_FACE_DETECTION_MODEL_H
