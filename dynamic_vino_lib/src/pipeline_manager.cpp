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
 * @brief a header file with declaration of Pipeline Manager class
 * @file pipeline_manager.cpp
 */

#include <memory>
#include <string>
#include <utility>
#include <vino_param_lib/param_manager.h>
#include "dynamic_vino_lib/inferences/age_gender_detection.h"
#include "dynamic_vino_lib/inferences/emotions_detection.h"
// #include "dynamic_vino_lib/inferences/object_detection.h"
#include "dynamic_vino_lib/inferences/face_detection.h"
#include "dynamic_vino_lib/inferences/head_pose_detection.h"
#include "dynamic_vino_lib/inferences/face_reidentification.h"
#include "dynamic_vino_lib/inferences/person_attribs_detection.h"
#include "dynamic_vino_lib/inferences/vehicle_attribs_detection.h"
#include "dynamic_vino_lib/inferences/license_plate_detection.h"
#include "dynamic_vino_lib/inferences/landmarks_detection.h"
#include "dynamic_vino_lib/inferences/gaze_estimation.h"
#include "dynamic_vino_lib/inferences/head_pose_detection.h"
#include "dynamic_vino_lib/inferences/peak.h"
#include "dynamic_vino_lib/inferences/human_pose_estimation.h"
#include "dynamic_vino_lib/inputs/image_input.h"
#include "dynamic_vino_lib/inputs/realsense_camera.h"
#include "dynamic_vino_lib/inputs/realsense_camera_topic.h"
#include "dynamic_vino_lib/inputs/standard_camera.h"
#include "dynamic_vino_lib/inputs/ip_camera.h"
#include "dynamic_vino_lib/inputs/video_input.h"
#include "dynamic_vino_lib/models/age_gender_detection_model.h"
#include "dynamic_vino_lib/models/emotion_detection_model.h"
#include "dynamic_vino_lib/models/face_detection_model.h"
#include "dynamic_vino_lib/models/head_pose_detection_model.h"
#include "dynamic_vino_lib/models/object_detection_ssd_model.h"
#include "dynamic_vino_lib/models/object_detection_yolov2voc_model.h"
#include "dynamic_vino_lib/models/face_reidentification_model.h"
#include "dynamic_vino_lib/models/person_attribs_detection_model.h"
#include "dynamic_vino_lib/models/vehicle_attribs_detection_model.h"
#include "dynamic_vino_lib/models/license_plate_detection_model.h"
#include "dynamic_vino_lib/models/landmarks_detection_model.h"
#include "dynamic_vino_lib/models/gaze_estimation_model.h"
#include "dynamic_vino_lib/models/head_pose_detection_model.h"
#include "dynamic_vino_lib/models/human_pose_estimation_model.h"
#include "dynamic_vino_lib/outputs/image_window_output.h"
#include "dynamic_vino_lib/outputs/ros_topic_output.h"
#include "dynamic_vino_lib/outputs/rviz_output.h"
#include "dynamic_vino_lib/outputs/ros_service_output.h"
#include "dynamic_vino_lib/pipeline.h"
#include "dynamic_vino_lib/pipeline_manager.h"
#include "dynamic_vino_lib/pipeline_params.h"
#include "dynamic_vino_lib/services/pipeline_processing_server.h"
#include "dynamic_vino_lib/engines/engine_manager.h"

std::shared_ptr<Pipeline> PipelineManager::createPipeline(const Params::ParamManager::PipelineRawData& params)
{
  if (params.name == "")
  {
    throw std::logic_error("The name of pipeline won't be empty!");
  }

  std::shared_ptr<Pipeline> pipeline = std::make_shared<Pipeline>(params.name);
  pipeline->getParameters()->update(params);

  PipelineData data;
  data.pipeline = pipeline;
  data.params = params;
  data.state = PipelineState_ThreadNotCreated;

  auto inputs = parseInputDevice(data);
  if (inputs.size() != 1)
  {
    slog::err << "currently one pipeline only supports ONE input." << slog::endl;
    return nullptr;
  }
  for (auto it = inputs.begin(); it != inputs.end(); ++it)
  {
    pipeline->add(it->first, it->second);
    auto node = it->second->getHandler();
    if (node != nullptr)
    {
      data.spin_nodes.emplace_back(node);
    }
  }

  auto outputs = parseOutput(data);
  for (auto it = outputs.begin(); it != outputs.end(); ++it)
  {
    pipeline->add(it->first, it->second);
  }

  auto infers = parseInference(params);
  for (auto it = infers.begin(); it != infers.end(); ++it)
  {
    pipeline->add(it->first, it->second);
  }

  slog::info << "Updating connections ..." << slog::endl;
  for (auto it = params.connects.begin(); it != params.connects.end(); ++it)
  {
    pipeline->add(it->first, it->second);
  }

  pipelines_.insert({ params.name, data });

  pipeline->setCallback();
  slog::info << "One Pipeline Created!" << slog::endl;
  pipeline->printPipeline();
  return pipeline;
}

std::map<std::string, std::shared_ptr<Input::BaseInputDevice>>
PipelineManager::parseInputDevice(const PipelineData& pdata)
{
  std::map<std::string, std::shared_ptr<Input::BaseInputDevice>> inputs;
  for (auto& name : pdata.params.inputs)
  {
    slog::info << "Parsing InputDvice: " << name << slog::endl;
    std::shared_ptr<Input::BaseInputDevice> device = nullptr;
    if (name == kInputType_RealSenseCamera)
    {
      device = std::make_shared<Input::RealSenseCamera>();
    }
    else if (name == kInputType_StandardCamera)
    {
      device = std::make_shared<Input::StandardCamera>();
    }
    else if (name == kInputType_IpCamera)
    {
      if (pdata.params.input_meta != "")
      {
        device = std::make_shared<Input::IpCamera>(pdata.params.input_meta);
      }
    }
    else if (name == kInputType_CameraTopic || name == kInputType_ImageTopic)
    {
      device = std::make_shared<Input::ImageTopic>();
    }
    else if (name == kInputType_Video)
    {
      if (pdata.params.input_meta != "")
      {
        device = std::make_shared<Input::Video>(pdata.params.input_meta);
      }
    }
    else if (name == kInputType_Image)
    {
      if (pdata.params.input_meta != "")
      {
        device = std::make_shared<Input::Image>(pdata.params.input_meta);
      }
    }
    else
    {
      slog::err << "Invalid input device name: " << name << slog::endl;
    }

    if (device != nullptr)
    {
      device->initialize();
      inputs.insert({ name, device });
      slog::info << " ... Adding one Input device: " << name << slog::endl;
    }
  }

  return inputs;
}

std::map<std::string, std::shared_ptr<Outputs::BaseOutput>> PipelineManager::parseOutput(const PipelineData& pdata)
{
  std::map<std::string, std::shared_ptr<Outputs::BaseOutput>> outputs;
  for (auto& name : pdata.params.outputs)
  {
    slog::info << "Parsing Output: " << name << slog::endl;
    std::shared_ptr<Outputs::BaseOutput> object = nullptr;
    if (name == kOutputTpye_RosTopic)
    {
      object = std::make_shared<Outputs::RosTopicOutput>(pdata.params.name);
    }
    else if (name == kOutputTpye_ImageWindow)
    {
      object = std::make_shared<Outputs::ImageWindowOutput>(pdata.params.name);
    }
    else if (name == kOutputTpye_RViz)
    {
      object = std::make_shared<Outputs::RvizOutput>(pdata.params.name);
    }
    else if (name == kOutputTpye_RosService)
    {
      object = std::make_shared<Outputs::RosServiceOutput>(pdata.params.name);
    }
    else
    {
      slog::err << "Invalid output name: " << name << slog::endl;
    }
    if (object != nullptr)
    {
      outputs.insert({ name, object });
      slog::info << " ... Adding one Output: " << name << slog::endl;
    }
  }

  return outputs;
}

std::map<std::string, std::shared_ptr<dynamic_vino_lib::BaseInference>>
PipelineManager::parseInference(const Params::ParamManager::PipelineRawData& params)
{
  std::map<std::string, std::shared_ptr<dynamic_vino_lib::BaseInference>> inferences;
  for (auto& infer : params.infers)
  {
    if (infer.name.empty() || infer.model.empty())
    {
      continue;
    }
    slog::info << "Parsing Inference: " << infer.name << slog::endl;
    std::shared_ptr<dynamic_vino_lib::BaseInference> object = nullptr;

    if (infer.name == kInferTpye_FaceDetection)
    {
      object = createFaceDetection(infer);
    }
    else if (infer.name == kInferTpye_AgeGenderRecognition)
    {
      object = createAgeGenderRecognition(infer);
    }
    else if (infer.name == kInferTpye_EmotionRecognition)
    {
      object = createEmotionRecognition(infer);
    }
    else if (infer.name == kInferTpye_HeadPoseEstimation)
    {
      object = createHeadPoseEstimation(infer);
    }
    else if (infer.name == kInferTpye_ObjectDetection)
    {
      object = createObjectDetection(infer);
    }
    else if (infer.name == kInferTpye_ObjectSegmentation)
    {
      object = createObjectSegmentation(infer);
    }
    else if (infer.name == kInferTpye_PersonReidentification)
    {
      object = createPersonReidentification(infer);
    }
    else if (infer.name == kInferTpye_FaceReidentification)
    {
      object = createFaceReidentification(infer);
    }
    else if (infer.name == kInferTpye_PersonAttribsDetection)
    {
      object = createPersonAttribsDetection(infer);
    }
    else if (infer.name == kInferTpye_LandmarksDetection)
    {
      object = createLandmarksDetection(infer);
    }
    else if (infer.name == kInferTpye_GazeEstimation)
    {
      object = createGazeEstimation(infer);
    }
    else if (infer.name == kInferTpye_VehicleAttribsDetection)
    {
      object = createVehicleAttribsDetection(infer);
    }
    else if (infer.name == kInferTpye_LicensePlateDetection)
    {
      object = createLicensePlateDetection(infer);
    }
    else if (infer.name == kInferTpye_HumanPoseEstimation) 
    {
      object = createHumanPoseEstimation(infer);
    }
    else
    {
      slog::err << "Invalid inference name: " << infer.name << slog::endl;
    }

    if (object != nullptr)
    {
      inferences.insert({ infer.name, object });
      slog::info << " ... Adding one Inference: " << infer.name << slog::endl;
    }
  }

  return inferences;
}

std::shared_ptr<dynamic_vino_lib::BaseInference>
PipelineManager::createAgeGenderRecognition(const Params::ParamManager::InferenceRawData& infer_param)
{
  auto model = std::make_shared<Models::AgeGenderDetectionModel>();
  model->modelInit(infer_param.model, infer_param.batch);
  auto engine = engine_manager_.createEngine(infer_param.engine, model);
  auto infer = std::make_shared<dynamic_vino_lib::AgeGenderDetection>();
  infer->loadNetwork(model);
  infer->loadEngine(engine);

  return infer;
}

std::shared_ptr<dynamic_vino_lib::BaseInference>
PipelineManager::createEmotionRecognition(const Params::ParamManager::InferenceRawData& infer_param)
{
  auto model = std::make_shared<Models::EmotionDetectionModel>();
  model->modelInit(infer_param.model, infer_param.batch);
  auto engine = engine_manager_.createEngine(infer_param.engine, model);
  auto infer = std::make_shared<dynamic_vino_lib::EmotionsDetection>();
  infer->loadNetwork(model);
  infer->loadEngine(engine);

  return infer;
}

std::shared_ptr<dynamic_vino_lib::BaseInference>
PipelineManager::createHeadPoseEstimation(const Params::ParamManager::InferenceRawData& infer_param)
{
  auto model = std::make_shared<Models::HeadPoseDetectionModel>();
  model->modelInit(infer_param.model, infer_param.batch);
  auto engine = engine_manager_.createEngine(infer_param.engine, model);
  auto infer = std::make_shared<dynamic_vino_lib::HeadPoseDetection>();
  infer->loadNetwork(model);
  infer->loadEngine(engine);

  return infer;
}

std::shared_ptr<dynamic_vino_lib::BaseInference>
PipelineManager::createFaceDetection(const Params::ParamManager::InferenceRawData& infer_param)
{
  auto model = std::make_shared<Models::FaceDetectionModel>();
  model->modelInit(infer_param.model, infer_param.batch);
  auto engine = engine_manager_.createEngine(infer_param.engine, model);
  auto infer = std::make_shared<dynamic_vino_lib::FaceDetection>(infer_param.enable_roi_constraint, infer_param.confidence_threshold);
  infer->loadNetwork(model);
  infer->loadEngine(engine);

  return infer;
}

std::shared_ptr<dynamic_vino_lib::BaseInference>
PipelineManager::createObjectDetection(const Params::ParamManager::InferenceRawData& infer_param)
{
  auto model = std::make_shared<Models::ObjectDetectionSSDModel>();
  model->modelInit(infer_param.model, infer_param.batch);
  auto engine = engine_manager_.createEngine(infer_param.engine, model);
  auto infer = std::make_shared<dynamic_vino_lib::ObjectDetection>(infer_param.enable_roi_constraint, infer_param.confidence_threshold);
  infer->loadNetwork(model);
  infer->loadEngine(engine);

  return infer;
}

std::shared_ptr<dynamic_vino_lib::BaseInference>
PipelineManager::createObjectSegmentation(const Params::ParamManager::InferenceRawData& infer_param)
{
  auto model = std::make_shared<Models::ObjectSegmentationModel>();
  model->modelInit(infer_param.model, infer_param.batch);
  auto engine = engine_manager_.createEngine(infer_param.engine, model);
  auto infer = std::make_shared<dynamic_vino_lib::ObjectSegmentation>(infer_param.confidence_threshold);
  infer->loadNetwork(model);
  infer->loadEngine(engine);

  return infer;
}

std::shared_ptr<dynamic_vino_lib::BaseInference>
PipelineManager::createPersonReidentification(const Params::ParamManager::InferenceRawData& infer_param)
{
  auto model = std::make_shared<Models::PersonReidentificationModel>();
  model->modelInit(infer_param.model, infer_param.batch);
  auto engine = engine_manager_.createEngine(infer_param.engine, model);
  auto infer = std::make_shared<dynamic_vino_lib::PersonReidentification>(infer_param.confidence_threshold);
  infer->loadNetwork(model);
  infer->loadEngine(engine);

  return infer;
}

std::shared_ptr<dynamic_vino_lib::BaseInference>
PipelineManager::createVehicleAttribsDetection(const Params::ParamManager::InferenceRawData& infer_param)
{
  auto model = std::make_shared<Models::VehicleAttribsDetectionModel>();
  model->modelInit(infer_param.model, infer_param.batch);
  auto engine = engine_manager_.createEngine(infer_param.engine, model);
  auto infer = std::make_shared<dynamic_vino_lib::VehicleAttribsDetection>();
  infer->loadNetwork(model);
  infer->loadEngine(engine);

  return infer;
}

std::shared_ptr<dynamic_vino_lib::BaseInference>
PipelineManager::createLicensePlateDetection(const Params::ParamManager::InferenceRawData& infer_param)
{
  auto model = std::make_shared<Models::LicensePlateDetectionModel>();
  model->modelInit(infer_param.model, infer_param.batch);
  auto engine = engine_manager_.createEngine(infer_param.engine, model);
  auto infer = std::make_shared<dynamic_vino_lib::LicensePlateDetection>();
  infer->loadNetwork(model);
  infer->loadEngine(engine);

  return infer;
}

std::shared_ptr<dynamic_vino_lib::BaseInference>
PipelineManager::createPersonAttribsDetection(const Params::ParamManager::InferenceRawData& infer_param)
{
  auto model = std::make_shared<Models::PersonAttribsDetectionModel>();
  model->modelInit(infer_param.model, infer_param.batch);
  auto engine = engine_manager_.createEngine(infer_param.engine, model);
  auto infer = std::make_shared<dynamic_vino_lib::PersonAttribsDetection>(infer_param.confidence_threshold);
  infer->loadNetwork(model);
  infer->loadEngine(engine);

  return infer;
}

std::shared_ptr<dynamic_vino_lib::BaseInference>
PipelineManager::createFaceReidentification(const Params::ParamManager::InferenceRawData& infer_param)
{
  auto model = std::make_shared<Models::FaceReidentificationModel>();
  slog::debug << "for test in createFaceReidentification()" << slog::endl;
  model->modelInit(infer_param.model, infer_param.batch);
  auto engine = engine_manager_.createEngine(infer_param.engine, model);
  auto infer = std::make_shared<dynamic_vino_lib::FaceReidentification>(infer_param.confidence_threshold);
  infer->loadNetwork(model);
  infer->loadEngine(engine);

  return infer;
}

std::shared_ptr<dynamic_vino_lib::BaseInference>
PipelineManager::createLandmarksDetection(
  const Params::ParamManager::InferenceRawData & infer_param)
{
  auto model = std::make_shared<Models::LandmarksDetectionModel>();
  model->modelInit(infer_param.model, infer_param.batch);
  auto engine = engine_manager_.createEngine(infer_param.engine, model);
  auto infer =  std::make_shared<dynamic_vino_lib::LandmarksDetection>();
  infer->loadNetwork(model);
  infer->loadEngine(engine);

  return infer;
}

std::shared_ptr<dynamic_vino_lib::BaseInference>
PipelineManager::createGazeEstimation(
  const Params::ParamManager::InferenceRawData & infer_param)
{
  auto model = std::make_shared<Models::GazeEstimationModel>();
  model->modelInit(infer_param.model, infer_param.batch);
  auto engine = engine_manager_.createEngine(infer_param.engine, model);
  auto infer =  std::make_shared<dynamic_vino_lib::GazeEstimation>();
  infer->loadNetwork(model);
  infer->loadEngine(engine);

  return infer;
}

std::shared_ptr<dynamic_vino_lib::BaseInference>
PipelineManager::createHumanPoseEstimation(
  const Params::ParamManager::InferenceRawData & infer_param)
{
  auto model = std::make_shared<Models::HumanPoseEstimationModel>();
  model->modelInit(infer_param.model, infer_param.batch);
  auto engine = engine_manager_.createEngine(infer_param.engine, model);
  auto infer = std::make_shared<dynamic_vino_lib::HumanPoseEstimation>();
  infer->loadNetwork(model);
  infer->loadEngine(engine);

  // auto infer =  std::make_shared<dynamic_vino_lib::GazeEstimation>();
  return infer;

}

void PipelineManager::threadPipeline(const char* name)
{
  PipelineData& p = pipelines_[name];
  while (p.state != PipelineState_ThreadStopped && p.pipeline != nullptr && ros::ok())
  {
    if (p.state != PipelineState_ThreadPasued)
    {
      ros::spinOnce();
      p.pipeline->runOnce();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

void PipelineManager::runAll()
{
  for (auto it = pipelines_.begin(); it != pipelines_.end(); ++it)
  {
    if (it->second.state != PipelineState_ThreadRunning)
    {
      it->second.state = PipelineState_ThreadRunning;
    }
    if (it->second.thread == nullptr)
    {
      it->second.thread =
          std::make_shared<std::thread>(&PipelineManager::threadPipeline, this, it->second.params.name.c_str());
    }
  }
}

void PipelineManager::stopAll()
{
  for (auto it = pipelines_.begin(); it != pipelines_.end(); ++it)
  {
    if (it->second.state == PipelineState_ThreadRunning)
    {
      it->second.state = PipelineState_ThreadStopped;
    }
  }
}

void PipelineManager::runService()
{
  auto node =
      std::make_shared<vino_service::PipelineProcessingServer<pipeline_srv_msgs::PipelineSrv>>("pipeline_service");
  ros::spin();  // hold the thread waiting for pipeline service
}

void PipelineManager::joinAll()
{
  auto service_thread = std::make_shared<std::thread>(&PipelineManager::runService, this);
  service_thread->join();  // pipeline service

  for (auto it = pipelines_.begin(); it != pipelines_.end(); ++it)
  {
    if (it->second.thread != nullptr && it->second.state == PipelineState_ThreadRunning)
    {
      it->second.thread->join();
    }
  }
}
