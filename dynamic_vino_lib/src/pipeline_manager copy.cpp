
std::shared_ptr<dynamic_vino_lib::BaseInference>
PipelineManager::createFaceDetection(const Params::ParamManager::InferenceRawData& infer)
{
  std::shared_ptr<Models::FaceDetectionModel> object_detection_model;
  std::shared_ptr<dynamic_vino_lib::FaceDetection> object_inference_ptr;
  slog::debug << "for test in createFaceDetection(), model_path =" << infer.model << slog::endl;
  object_detection_model = std::make_shared<Models::FaceDetectionGazeModel>(infer.model, infer.batch);

  slog::debug << "for test in createFaceDetection(), Created SSDModel" << slog::endl;
  object_inference_ptr = std::make_shared<dynamic_vino_lib::FaceDetection>(
      infer.enable_roi_constraint, infer.confidence_threshold);  // To-do theshold configuration
  slog::debug << "for test in createFaceDetection(), before modelInit()" << slog::endl;
  object_detection_model->modelInit();
  auto object_detection_engine = engine_manager_.createEngine(infer.engine, object_detection_model);
  object_inference_ptr->loadNetwork(object_detection_model);
  object_inference_ptr->loadEngine(object_detection_engine);

  return object_inference_ptr;
}