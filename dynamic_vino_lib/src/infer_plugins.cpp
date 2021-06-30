//包含pluginlib的头文件，使用pluginlib的宏来注册插件
#include <pluginlib/class_list_macros.hpp>

#include "dynamic_vino_lib/inferences/age_gender_detection.h"
#include "dynamic_vino_lib/inferences/emotions_detection.h"
#include "dynamic_vino_lib/inferences/object_detection.h"
#include "dynamic_vino_lib/inferences/face_detection.h"
#include "dynamic_vino_lib/inferences/head_pose_detection.h"
#include "dynamic_vino_lib/inferences/face_reidentification.h"
#include "dynamic_vino_lib/inferences/person_attribs_detection.h"
#include "dynamic_vino_lib/inferences/vehicle_attribs_detection.h"
#include "dynamic_vino_lib/inferences/license_plate_detection.h"
#include "dynamic_vino_lib/inferences/landmarks_detection.h"
#include "dynamic_vino_lib/inferences/gaze_estimation.h"
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

//注册插件，宏参数：plugin的实现类，plugin的基类
PLUGINLIB_EXPORT_CLASS(dynamic_vino_lib::AgeGenderDetection, dynamic_vino_lib::BaseInference);
PLUGINLIB_EXPORT_CLASS(dynamic_vino_lib::EmotionsDetection, dynamic_vino_lib::BaseInference);
PLUGINLIB_EXPORT_CLASS(dynamic_vino_lib::FaceDetection, dynamic_vino_lib::BaseInference);
PLUGINLIB_EXPORT_CLASS(dynamic_vino_lib::FaceReidentification, dynamic_vino_lib::BaseInference);
PLUGINLIB_EXPORT_CLASS(dynamic_vino_lib::GazeEstimation, dynamic_vino_lib::BaseInference);
PLUGINLIB_EXPORT_CLASS(dynamic_vino_lib::HeadPoseDetection, dynamic_vino_lib::BaseInference);
PLUGINLIB_EXPORT_CLASS(dynamic_vino_lib::LandmarksDetection, dynamic_vino_lib::BaseInference);
PLUGINLIB_EXPORT_CLASS(dynamic_vino_lib::LicensePlateDetection, dynamic_vino_lib::BaseInference);
PLUGINLIB_EXPORT_CLASS(dynamic_vino_lib::ObjectDetection, dynamic_vino_lib::BaseInference);
PLUGINLIB_EXPORT_CLASS(dynamic_vino_lib::ObjectSegmentation, dynamic_vino_lib::BaseInference);
PLUGINLIB_EXPORT_CLASS(dynamic_vino_lib::PersonAttribsDetection, dynamic_vino_lib::BaseInference);
PLUGINLIB_EXPORT_CLASS(dynamic_vino_lib::PersonReidentification, dynamic_vino_lib::BaseInference);
PLUGINLIB_EXPORT_CLASS(dynamic_vino_lib::VehicleAttribsDetection, dynamic_vino_lib::BaseInference);

//注册插件，宏参数：plugin的实现类，plugin的基类
PLUGINLIB_EXPORT_CLASS(Models::AgeGenderDetectionModel, Models::BaseModel);
PLUGINLIB_EXPORT_CLASS(Models::EmotionDetectionModel, Models::BaseModel);
PLUGINLIB_EXPORT_CLASS(Models::FaceDetectionModel, Models::BaseModel);
PLUGINLIB_EXPORT_CLASS(Models::FaceReidentificationModel, Models::BaseModel);
PLUGINLIB_EXPORT_CLASS(Models::GazeEstimationModel, Models::BaseModel);
PLUGINLIB_EXPORT_CLASS(Models::HeadPoseDetectionModel, Models::BaseModel);
PLUGINLIB_EXPORT_CLASS(Models::LandmarksDetectionModel, Models::BaseModel);
PLUGINLIB_EXPORT_CLASS(Models::LicensePlateDetectionModel, Models::BaseModel);
PLUGINLIB_EXPORT_CLASS(Models::ObjectDetectionSSDModel, Models::BaseModel);
PLUGINLIB_EXPORT_CLASS(Models::ObjectDetectionYolov2Model, Models::BaseModel);
PLUGINLIB_EXPORT_CLASS(Models::ObjectSegmentationModel, Models::BaseModel);
PLUGINLIB_EXPORT_CLASS(Models::PersonAttribsDetectionModel, Models::BaseModel);
PLUGINLIB_EXPORT_CLASS(Models::PersonReidentificationModel, Models::BaseModel);
PLUGINLIB_EXPORT_CLASS(Models::VehicleAttribsDetectionModel, Models::BaseModel);
