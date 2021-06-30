#include <boost/shared_ptr.hpp>

#include <pluginlib/class_loader.hpp>

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

#include "dynamic_vino_lib/pipeline_manager.h"

std::shared_ptr<dynamic_vino_lib::BaseInference> PipelineManager::createInference(const Params::ParamManager::InferenceRawData& infer_param)
{
  // 创建一个ClassLoader，用来加载plugin
  pluginlib::ClassLoader<Models::BaseModel> model_loader("dynamic_vino_lib", "Models::BaseModel");
  pluginlib::ClassLoader<dynamic_vino_lib::BaseInference> inference_loader("dynamic_vino_lib", "dynamic_vino_lib::BaseInference");

  string pkg_name = "dynamic_vino_lib"

  try
  {
    // model_tutorials/infer_param.model_type
    string model_name = pkg_name + "/" + infer_param.model_type;
    boost::shared_ptr<Models::BaseModel> model = model_loader.createInstance(model_name);
	

    ROS_INFO("Triangle area: %.2f", triangle->area());
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
  }

  try
  {
    boost::shared_ptr<polygon_base::RegularPolygon> square = poly_loader.createInstance("pluginlib_tutorials/regular_square");
    square->initialize(10.0);

    ROS_INFO("Square area: %.2f", square->area());
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
  }

  tryparseInference
  {
    boost::shared_ptr<polygon_base::RegularPolygon> square2 = poly_loader.createInstance("pluginlib_tutorials/regular_triangle2");
    square2->initialize(10.0);

    ROS_INFO("Square area: %.2f", square2->area());
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
  }

  return 0;
}
