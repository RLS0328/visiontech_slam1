/*
 * Copyright (c) 2018 Smart Mapping Tech Co., Ltd. All rights reserved.
 *
 * Created by Shuang Song on 4/22/18.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef VINS_LIDAR_CONFIG_H
#define VINS_LIDAR_CONFIG_H

#include <iostream>
#include <memory>
#include <yaml-cpp/yaml.h>

class Config {
 private:
  static std::shared_ptr<Config> config_;

  YAML::Node node_;
  std::string filepath_;
  Config() {}

 public:
  ~Config() {
  }

  static void setParameterFile(const std::string &filename) {
    if (config_ == nullptr)
      config_ = std::shared_ptr<Config>(new Config);

    config_->node_ = YAML::LoadFile(filename);
    config_->filepath_ = filename;
    if (config_->node_.IsNull()) {
      std::cerr << "parameter file " << filename << " does not exist." << std::endl;
      return;
    }
  }

  template<typename T>
  static T get(const std::string &key) {
    auto result = config_->node_[key];
    if (result.IsDefined()) {
      return result.as<T>();
    } else {
      return T();
    }
  }

  template<typename T>
  static T get(const std::string &key, const T defaultVal) {
    auto result = config_->node_[key];
    if (result.IsDefined()) {
      return result.as<T>();
    } else {
      return defaultVal;
    }
  }

  static std::string getConfigFilePath() {
    return Config::config_->filepath_;
  }

};

#endif //VINS_LIDAR_CONFIG_H
