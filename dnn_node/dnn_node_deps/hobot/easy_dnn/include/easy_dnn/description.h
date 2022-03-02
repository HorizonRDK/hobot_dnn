// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef _EASY_DNN_DESCRIPTION_H_
#define _EASY_DNN_DESCRIPTION_H_

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rapidjson/document.h"

/**
 * Register InputDescriptionParser Class, and aliases are supported
 */
#define REGISTER_INPUT_DESCRIPTION_PARSER(class, types...)                 \
  hobot::easy_dnn::InputDescriptionParser *class##_creator() {             \
    return new class;                                                      \
  }                                                                        \
  static hobot::easy_dnn::InputDescriptionParserRegistry class##_registry( \
      class##_creator, {types});

/**
 * Register OutputDescriptionParser Class, and aliases are supported
 */
#define REGISTER_OUTPUT_DESCRIPTION_PARSER(class, types...)                 \
  hobot::easy_dnn::OutputDescriptionParser *class##_creator() {             \
    return new class;                                                       \
  }                                                                         \
  static hobot::easy_dnn::OutputDescriptionParserRegistry class##_registry( \
      class##_creator, {types});

namespace hobot {
namespace easy_dnn {

class Model;
class InputProcessor;
class OutputParser;

/**
 * Base description
 */
class Description {
 public:
  Description(Model *mode, int index, std::string type)
      : model_(mode), index_(index), type_(std::move(type)) {}
  /**
   * Get model, kept for flexibility
   * @return model
   */
  inline Model *GetModel() { return model_; }

  /**
   * Set model
   * @param[in] model
   */
  inline void SetModel(Model *model) { model_ = model; }

  /**
   * Get input or output branch index, kept for flexibility
   * @return index
   */
  inline int32_t GetIndex() const { return index_; }

  /**
   * Set index, kept for flexibility
   * @param[in] index
   */
  inline void SetIndex(int32_t index) { index_ = index; }

  /**
   * Get description type, kept for flexibility
   * @return type
   */
  inline std::string &GetType() { return type_; }

  /**
   * Set type, kept for flexibility
   * @param[in] type
   */
  inline void SetType(const std::string &type) { type_ = type; }

  virtual ~Description() = default;

 protected:
  // members just for flexibility
  Model *model_;
  int32_t index_;
  std::string type_;
};

/**
 * Base input description
 */
class InputDescription : public Description {
 public:
  InputDescription(Model *mode, int index, std::string type = "")
      : Description(mode, index, std::move(type)) {}
  ~InputDescription() override = default;
};

/**
 * Base output description
 */
class OutputDescription : public Description {
 public:
  OutputDescription(Model *mode, int index, std::string type = "")
      : Description(mode, index, std::move(type)) {}
  /**
   * Get dependencies output branch
   * @return dependencies
   */
  std::vector<int> &GetDependencies() { return dependencies_; }

  ~OutputDescription() override = default;

 protected:
  std::vector<int> dependencies_;
};

/**
 * Input description parser
 */
class InputDescriptionParser {
 public:
  /**
   * Parse input description
   * @param[in] desc_doc description json document
   * @param[in] model redundant parameter
   * @param[in] input_index redundant parameter
   * @return <InputDescription,InputProcessor>
   */
  virtual std::pair<std::shared_ptr<InputDescription>,
                    std::shared_ptr<InputProcessor>>
  Parse(rapidjson::Document &desc_doc, Model *model, int32_t input_index) = 0;
};

/**
 * Output description parser
 */
class OutputDescriptionParser {
 public:
  /**
   * Parse output description
   * @param[in] desc_doc description json document
   * @param[in] model redundant parameter
   * @param[in] output_index redundant parameter
   * @return <OutputDescription,OutputParser>
   */
  virtual std::pair<std::shared_ptr<OutputDescription>,
                    std::shared_ptr<OutputParser>>
  Parse(rapidjson::Document &desc_doc, Model *model, int32_t output_index) = 0;
};

class InputDescriptionParserFactory {
 public:
  /**
   * Get input description parser factory
   * @return singleton instance
   */
  static InputDescriptionParserFactory *GetInstance();

  virtual void RegisterParser(
      const std::string &type,
      std::shared_ptr<InputDescriptionParser> &desc_parser) = 0;
};

class OutputDescriptionParserFactory {
 public:
  static OutputDescriptionParserFactory *GetInstance();

  virtual void RegisterParser(
      const std::string &type,
      std::shared_ptr<OutputDescriptionParser> &desc_parser) = 0;
};

typedef hobot::easy_dnn::InputDescriptionParser *(
    *InputDescriptionParserCreator)();

class InputDescriptionParserRegistry {
 public:
  explicit InputDescriptionParserRegistry(
      InputDescriptionParserCreator creator,
      std::initializer_list<std::string> types) noexcept {
    auto *parser = creator();
    auto shared_parser = std::shared_ptr<InputDescriptionParser>(parser);
    for (const std::string &type : types) {
      InputDescriptionParserFactory::GetInstance()->RegisterParser(
          type, shared_parser);
    }
  }
};

typedef hobot::easy_dnn::OutputDescriptionParser *(
    *OutputDescriptionParserCreator)();

class OutputDescriptionParserRegistry {
 public:
  explicit OutputDescriptionParserRegistry(
      OutputDescriptionParserCreator creator,
      std::initializer_list<std::string> types) noexcept {
    auto *parser = creator();
    auto shared_parser = std::shared_ptr<OutputDescriptionParser>(parser);
    for (const std::string &type : types) {
      OutputDescriptionParserFactory::GetInstance()->RegisterParser(
          type, shared_parser);
    }
  }
};

}  // namespace easy_dnn
}  // namespace hobot

#endif  // _EASY_DNN_DESCRIPTION_H_
