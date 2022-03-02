// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef DNN_NODE_H_
#define DNN_NODE_H_

#include <string>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"

#include "easy_dnn/model.h"
#include "easy_dnn/data_structure.h"
#include "easy_dnn/model_manager.h"
#include "easy_dnn/input_processor.h"
#include "easy_dnn/input_process/crop.h"
#include "easy_dnn/task.h"
#include "easy_dnn/task_manager.h"
#include "easy_dnn/output_parser.h"
#include "easy_dnn/output_parser/parsing/parsing_output_parser.h"
#include "easy_dnn/output_parser/detection/facehand_detect_output_parser.h"

#include "dnn/hb_dnn.h"

namespace hobot {
namespace dnn_node {

using rclcpp::NodeOptions;
using hobot::easy_dnn::Model;
using hobot::easy_dnn::DNNInput;
using hobot::easy_dnn::DNNResult;
using hobot::easy_dnn::Task;
using hobot::easy_dnn::ModelTask;
using hobot::easy_dnn::ModelInferTask;
using hobot::easy_dnn::ModelRoiInferTask;
using hobot::easy_dnn::MultiModelTask;
using hobot::easy_dnn::ModelManager;
using hobot::easy_dnn::NV12PyramidInput;
using hobot::easy_dnn::TaskManager;
using hobot::easy_dnn::ParsingResult;
using hobot::easy_dnn::InputProcessor;
using hobot::easy_dnn::InputDescription;
using hobot::easy_dnn::OutputParser;
using hobot::easy_dnn::CropProcessor;
using hobot::easy_dnn::CropDescription;

using TaskId = int;

struct DnnNodeRunTimePara;
struct ThreadPool;

// 任务实体类型，包括以下几种类型
// - ModelInferTask: Pyramid或DDR模型任务
// - ModelRoiInferTask: Resizer模型任务，
//   有且只有一个输入源为resizer (pyramid和roi)，剩余的为pyramid或DDR
enum class ModelTaskType {
  InvalidType = 0,
  ModelInferType = 1,
  ModelRoiInferType = 2
};

struct DnnNodePara {
  // 模型文件名称
  std::string model_file;

  // 模型名，dnn node根据model_name解析出需要管理和推理使用的模型
  std::string model_name;

  // 模型的task类型，dnn node根据模类型创建task
  // 例如多任务检测或者扣图检测，对应的ModelTask分别是ModelInferTask和ModelRoiInferTask
  ModelTaskType model_task_type;

  // 创建task超时时间，单位ms
  int timeout_ms = 100;

  // 创建的task数量，一个model支持由多个task执行
  int task_num = 1;
};

// 用户可以继承DnnNodeOutput来扩展输出内容
// 例如增加输入信息：图片数据、图片名、时间戳、ID等
struct DnnNodeOutput {
  DnnNodeOutput() {
    outputs.clear();
  }
  virtual ~DnnNodeOutput() {}
  // 输出数据智能指针列表
  std::vector<std::shared_ptr<DNNResult>> outputs;
};

class DnnNode : public rclcpp::Node {
 public:
  // node_name为创建的节点名，options为选项，用法和ROS Node相同
  DnnNode(
    const std::string & node_name,
    const NodeOptions & options = NodeOptions());

  DnnNode(
    const std::string & node_name,
    const std::string & namespace_,
    const NodeOptions & options = NodeOptions());

  virtual ~DnnNode();

  // 执行初始化流程，只做pipeline的串联，具体的每个初始化步骤由用户（子类中）实现。
  int Init();

  // 1. 批量配置预测任务的输入数据。
  // 2. 更新模型输入描述InputDescription/SetInputProcessor
  //    （如果需要，例如需要为crop检测模型指定抠图方法，全图检测不需要更新）。
  // 3. 如果是抠图检测模型，还需要指定抠图区域roi数据（hbDNNRoi类型）。
  // 4. DNNInput是easy dnn中定义的模型输入数据类型，
  //    用户必须根据实际使用的模型数据输入，继承DNNInput并定义模型输入数据类型。
  // 5. easy dnn中定义了一些常用模型的输入数据类型，
  //    如pym输入类型NV12PyramidInput。
  // - 参数
  //   - [in] inputs 输入数据智能指针列表。
  //   - [in] task_id 预测任务ID。
  //   - [in] rois 抠图roi数据，只对抠图检测模型有效。
  virtual int PreProcess(std::vector<std::shared_ptr<DNNInput>> &inputs,
            const TaskId& task_id,
            const std::shared_ptr<std::vector<hbDNNRoi>> rois = nullptr) = 0;
  // 执行推理任务
  // - 参数
  //   - [out] outputs 输出数据智能指针列表，同步模式有效。
  //   - [in] task_id 预测任务ID。
  //   - [in] is_sync_mode 预测模式，true为同步模式，false为异步模式。
  //   - [in] timeout_ms 预测推理超时时间。
  int RunInferTask(std::shared_ptr<DnnNodeOutput> &sync_output,
                   const TaskId& task_id,
                   const bool is_sync_mode = true,
                   const int timeout_ms = 1000);

  // 处理解析后的模型输出数据，例如将输出封装成msg发布到总线。
  // DNNResult是easy dnn中定义的模型输出数据类型，
  // 用户必须根据实际使用的模型输出数据类型，继承DNNResult并定义模型输出数据类型。
  // 例如对于检测模型，继承DNNResult的子类中需要定义检测框、置信度等输出数据类型。
  // - 参数
  //   - [out] outputs 输出数据智能指针列表。
  // 解析模型输出数据DNNResult，如必要，将输出封装成msg发布到总线
  // DNNResult是easy dnn中定义的模型输出数据类型，
  // 用户必须根据实际使用的模型输出数据类型，继承DNNResult并定义模型输出数据类型
  // 例如对于检测模型，继承DNNResult的子类中需要定义检测框、置信度等输出数据类型
  virtual int PostProcess(
    const std::shared_ptr<DnnNodeOutput> &output) = 0;

 protected:
  // easy_dnn_node_para_ptr为模型管理和推理参数，用户配置模型文件名和模型类型后传入
  std::shared_ptr<DnnNodePara> dnn_node_para_ptr_ = nullptr;

  // 获取dnn node管理和推理使用的模型。
  Model* GetModel();

  // 设置DnnNodePara类型的node para。
  virtual int SetNodePara() = 0;

  // 配置模型输出的解析方式。
  virtual int SetOutputParser() = 0;

  // 申请模型预测任务。
  // - 参数
  //   - [in] timeout_ms 申请超时时间。
  // - 返回值
  //   - 返回申请到的task id，小于0为无效id。
  TaskId AllocTask(int timeout_ms = -1);

  // 释放模型预测任务。
  // - 参数
  //   - [in] task_id 需要释放的task id。
  int ReleaseTask(const TaskId&);

  // 根据预测任务ID获取任务task。
  // - 参数
  //   - [in] task_id 预测任务ID。
  std::shared_ptr<Task> GetTask(const TaskId&);

 private:
  std::shared_ptr<DnnNodeRunTimePara> dnn_rt_para_ = nullptr;
  std::shared_ptr<ThreadPool> thread_pool_ = nullptr;

  void NodeInit();
  int ModelInit();
  int TaskInit();

  // 使用通过SetInputs输入给模型的数据进行推理
  // outputs为模型输出，timeout_ms为推理超时时间
  int RunInfer(std::vector<std::shared_ptr<DNNResult>> &outputs,
                   const std::shared_ptr<Task>& task,
                   const int timeout_ms = 100);
};

}  // namespace dnn_node
}  // namespace hobot
#endif  // DNN_NODE_H_
