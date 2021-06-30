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
 * @brief a header file with declaration of BaseModel class
 * @file base_model.cpp
 */

#include <algorithm>
#include <fstream>
#include <memory>
#include <string>

#include "dynamic_vino_lib/models/base_model.h"
#include "dynamic_vino_lib/slog.h"

// Validated Base Network
Models::BaseModel::BaseModel()
{
}

void Models::BaseModel::modelInit(const std::string& model_loc, int max_batch_size)
{
  setModelName(model_loc);
  setMaxBatchSize(max_batch_size);
  net_reader_ = std::make_shared<InferenceEngine::CNNNetReader>();
  slog::info << "Loading network files: " << model_loc << slog::endl;

  // Read network model
  net_reader_->ReadNetwork(model_loc);
  // Extract model name and load it's weights
  // remove extension
  size_t last_index = model_loc.find_last_of(".");
  std::string raw_name = model_loc.substr(0, last_index);
  std::string bin_file_name = raw_name + ".bin";
  net_reader_->ReadWeights(bin_file_name);
  // Read labels (if any)
  std::string label_file_name = raw_name + ".labels";
  loadLabelsFromFile(label_file_name);

  // Set batch size to given max_batch_size
  slog::info << "Batch size is set to  " << max_batch_size << slog::endl;
  net_reader_->getNetwork().setBatchSize(max_batch_size);

  updateLayerProperty(net_reader_);
}

#if 0
bool Models::BaseModel::updateLayerProperty(
  InferenceEngine::CNNNetReader::Ptr net_reader)
{
#if 0
  if (!updateLayerProperty(net_reader)){
    slog::warn << "The model(name: " << getModelName() << ") failed to update Layer Property!"
      << slog::endl;
    return false;
  }
#endif
  if(!isVerified()){
    slog::warn << "The model(name: " << getModelName() << ") does NOT pass Attribute Check!"
      << slog::endl;
    return false;
  }

  return true;
}
#endif

