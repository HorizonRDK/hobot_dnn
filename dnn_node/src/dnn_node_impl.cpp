// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include <string>
#include <memory>
#include <vector>
#include <queue>
#include <unordered_map>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "dnn_node/dnn_node_impl.h"

#include "hobotlog/hobotlog.hpp"

namespace hobot {
namespace dnn_node {

DnnNodeImpl::DnnNodeImpl(std::shared_ptr<DnnNodePara>& dnn_node_para_ptr) {
  dnn_node_para_ptr_ = dnn_node_para_ptr;
  dnn_rt_para_ = std::make_shared<DnnNodeRunTimePara>();
  thread_pool_ = std::make_shared<ThreadPool>();
}

DnnNodeImpl::~DnnNodeImpl() {
  if (!dnn_rt_para_) {
    ModelManager::GetInstance()->OffLoad(dnn_rt_para_->models_load);
  }
}

int DnnNodeImpl::ModelInit() {
  RCLCPP_INFO(rclcpp::get_logger("dnn"), "Model init.");
  if (!dnn_node_para_ptr_ || !dnn_rt_para_) {
    RCLCPP_ERROR(rclcpp::get_logger("dnn"), "Invalid node para!");
    return -1;
  }

  // 1. 加载模型hbm文件，一个hbm中可能包含多个模型
  int ret = 0;
  ret = ModelManager::GetInstance()->Load(dnn_rt_para_->models_load,
  dnn_node_para_ptr_->model_file);
  if (0 != ret) {
    RCLCPP_ERROR(rclcpp::get_logger("dnn"), "Load model: %s fail, ret: %d",
    dnn_node_para_ptr_->model_file.c_str(), ret);
    return ret;
  }

  // 2. 根据模型名，加载实际需要管理的模型
  const auto& model_name = dnn_node_para_ptr_->model_name;
  for (auto model : dnn_rt_para_->models_load) {
    if (model->GetName() == model_name) {
      dnn_rt_para_->model_manage = model;
      break;
    }
  }
  if (!dnn_rt_para_->model_manage) {
    RCLCPP_ERROR(rclcpp::get_logger("dnn"),
      "Find model: %s fail!", model_name.c_str());
    return -1;
  }

  // 3. 查询模型的输入信息
  for (int idx = 0; idx < dnn_rt_para_->model_manage->GetInputCount(); idx++) {
    hbDNNTensorProperties properties;
    dnn_rt_para_->model_manage->GetInputTensorProperties(properties, idx);
    int in_w = properties.validShape.dimensionSize[3];
    int in_h = properties.validShape.dimensionSize[2];
    RCLCPP_INFO(rclcpp::get_logger("dnn"),
    "The model input %d width is %d and height is %d", idx, in_w, in_h);
  }

  return 0;
}

int DnnNodeImpl::TaskInit() {
  RCLCPP_INFO(rclcpp::get_logger("dnn"), "Task init.");
  if (!dnn_node_para_ptr_ || !dnn_rt_para_ ||
    dnn_node_para_ptr_->task_num < 1) {
    RCLCPP_ERROR(rclcpp::get_logger("dnn"), "Invalid node para!");
    return -1;
  }

  // 1. 为每个模型创建task
  // 此处只创建空的task，AllocTask才真正创建task，ReleaseTask时释放task
  // 原因是目前一个task不支持多次预测，即每次预测都要申请一个新的task
  // todo 20220305 需要easy dnn支持task复用
  dnn_rt_para_->tasks.resize(dnn_node_para_ptr_->task_num);

  // 2. 创建idle running task
  {
    int task_num = dnn_rt_para_->tasks.size();
    std::unique_lock<std::mutex> lg(dnn_rt_para_->task_mtx);
    for (int idx = 0; idx < task_num; idx++) {
      auto node_task = std::make_shared<DnnNodeTask>(idx);
      dnn_rt_para_->idle_tasks[node_task->task_id] = node_task;
    }
  }

  thread_pool_->msg_handle_.CreatThread(dnn_node_para_ptr_->task_num);
  RCLCPP_INFO(rclcpp::get_logger("dnn"), "Set task_num [%d]",
    dnn_node_para_ptr_->task_num);

  // set log level of easy dnn
  SetLogLevel(HOBOT_LOG_ERROR);
  return 0;
}

int DnnNodeImpl::PreProcess(
  std::vector<std::shared_ptr<DNNInput>> &inputs,
  const TaskId& task_id,
  const std::shared_ptr<std::vector<hbDNNRoi>> rois) {
  if (task_id < 0 || !dnn_node_para_ptr_) {
    return -1;
  }
  uint32_t ret = 0;
  if (ModelTaskType::ModelRoiInferType == dnn_node_para_ptr_->model_task_type) {
    if (!rois) {
      RCLCPP_ERROR(rclcpp::get_logger("dnn"),
      "Invalid input rois for roi infer task");
      return -1;
    }

    std::shared_ptr<ModelRoiInferTask> infer_task =
      std::dynamic_pointer_cast<ModelRoiInferTask>(GetTask(task_id));
    if (!infer_task) {
      RCLCPP_ERROR(rclcpp::get_logger("dnn"), "Invalid infer task");
      return -1;
    }

    // set roi
    ret = infer_task->SetInputRois(*rois);
    if (ret != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("dnn"), "Failed to set roi inputs");
      return ret;
    }

    ret = infer_task->SetInputs(inputs);
    if (ret != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("dnn"), "Failed to set inputs");
      return ret;
    }
  } else if (ModelTaskType::ModelInferType ==
    dnn_node_para_ptr_->model_task_type) {
    std::shared_ptr<ModelInferTask> infer_task =
      std::dynamic_pointer_cast<ModelInferTask>(GetTask(task_id));
    if (!infer_task) {
      RCLCPP_ERROR(rclcpp::get_logger("dnn"), "Invalid infer task");
      return -1;
    }
    ret = infer_task->SetInputs(inputs);
    if (ret != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("dnn"), "Failed to set inputs");
      return ret;
    }
  }
  return ret;
}

int DnnNodeImpl::RunInferTask(
  std::shared_ptr<DnnNodeOutput> &sync_outputs,
  const TaskId& task_id,
  std::function<int(const std::shared_ptr<DnnNodeOutput> &)> post_process,
  const bool is_sync_mode,
  const int timeout_ms) {
  if (!dnn_rt_para_) {
    return -1;
  }
  if (is_sync_mode) {
    return RunInfer(sync_outputs->outputs, GetTask(task_id), timeout_ms);
  } else {
    std::lock_guard<std::mutex> lock(thread_pool_->msg_mutex_);
    if (thread_pool_->msg_handle_.GetTaskNum() >=
      thread_pool_->msg_limit_count_) {
      RCLCPP_ERROR(rclcpp::get_logger("dnn"),
        "Task Size: %d exceeds limit: %d",
        thread_pool_->msg_handle_.GetTaskNum(),
        thread_pool_->msg_limit_count_);
      return -1;
    }
    auto infer_task = [this, task_id, timeout_ms, sync_outputs, post_process](){
      std::shared_ptr<DnnNodeOutput> aync_outputs = sync_outputs;
      if (RunInfer(aync_outputs->outputs, GetTask(task_id), timeout_ms) != 0) {
        ReleaseTask(task_id);
        RCLCPP_ERROR(rclcpp::get_logger("dnn"), "Run infer fail\n");
      } else {
        ReleaseTask(task_id);
        if (post_process) {
          post_process(aync_outputs);
        }
      }
    };

    thread_pool_->msg_handle_.PostTask(infer_task);
  }
  return 0;
}

int DnnNodeImpl::RunInfer(std::vector<std::shared_ptr<DNNResult>> &outputs,
                    const std::shared_ptr<Task>& node_task,
                    const int timeout_ms) {
  if (!dnn_node_para_ptr_ || !node_task) {
    RCLCPP_ERROR(rclcpp::get_logger("dnn"), "Invalid node task\n");
    return -1;
  }

  auto& task = node_task;
  uint32_t ret = 0;
  ret = task->ProcessInput();
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("dnn"),
      "Failed to run infer task, ret[%d]", ret);
    return ret;
  }

  ret = task->RunInfer();
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("dnn"),
      "Failed to run infer task, ret[%d]", ret);
    return ret;
  }

  ret = task->WaitInferDone(timeout_ms);
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("dnn"),
      "Failed to run infer task, ret[%d]", ret);
    return ret;
  }

  if (ModelTaskType::ModelInferType == dnn_node_para_ptr_->model_task_type) {
    auto model_task = std::dynamic_pointer_cast<ModelInferTask>(task);
    ret = model_task->ParseOutput();
    if (ret != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("dnn"),
        "Failed to parse outputs, ret[%d]", ret);
      return ret;
    }
    ret = model_task->GetOutputs(outputs);
  } else if (ModelTaskType::ModelRoiInferType ==
    dnn_node_para_ptr_->model_task_type) {
    auto model_task = std::dynamic_pointer_cast<ModelRoiInferTask>(task);
    ret = model_task->ParseOutput();
    if (ret != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("dnn"),
        "Failed to parse outputs, ret[%d]", ret);
      return ret;
    }
    ret = model_task->GetOutputs(outputs);
  }

  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("dnn"),
      "Failed to get outputs, ret[%d]", ret);
  }

  return ret;
}

TaskId DnnNodeImpl::AllocTask(int timeout_ms) {
  RCLCPP_DEBUG(rclcpp::get_logger("dnn"), "Alloc task");
  TaskId task_id = -1;
  if (!dnn_rt_para_) {
    return task_id;
  }

  // 真正创建task
  std::shared_ptr<Task> task = nullptr;
  // 根据模型类型选择接口创建task,为task添加model
  if (ModelTaskType::ModelInferType == dnn_node_para_ptr_->model_task_type) {
    task =
      TaskManager::GetInstance()->GetModelInferTask(
      dnn_node_para_ptr_->timeout_ms);
    if (!task) {
      RCLCPP_ERROR(rclcpp::get_logger("dnn"), "GetModelInferTask fail");
      return task_id;
    }
    std::dynamic_pointer_cast<ModelInferTask>(task)->SetModel(
      dnn_rt_para_->model_manage);
  } else if (ModelTaskType::ModelRoiInferType ==
    dnn_node_para_ptr_->model_task_type) {
    task =
    TaskManager::GetInstance()->GetModelRoiInferTask(
      dnn_node_para_ptr_->timeout_ms);
    if (!task) {
      RCLCPP_ERROR(rclcpp::get_logger("dnn"), "GetModelRoiInferTask fail");
      return task_id;
    }
    std::dynamic_pointer_cast<ModelRoiInferTask>(task)->SetModel(
      dnn_rt_para_->model_manage);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("dnn"), "Invalid model task type [%d]",
      static_cast<int>(dnn_node_para_ptr_->model_task_type));
    return task_id;
  }

  auto alloc_task = [this, &task_id](){
    auto idle_task = dnn_rt_para_->idle_tasks.begin();
    idle_task->second->alloc_tp = std::chrono::system_clock::now();
    task_id = idle_task->first;
    dnn_rt_para_->running_tasks[task_id] = idle_task->second;
    dnn_rt_para_->idle_tasks.erase(task_id);
  };
  std::unique_lock<std::mutex> lg(dnn_rt_para_->task_mtx);
  if (!dnn_rt_para_->idle_tasks.empty()) {
    alloc_task();
  } else {
    // wait for idle task
    if (timeout_ms > 0) {
      dnn_rt_para_->task_cv.wait_for(lg, std::chrono::milliseconds(timeout_ms),
      [&](){
        return !dnn_rt_para_->idle_tasks.empty() || !rclcpp::ok();
      });
    } else {
      dnn_rt_para_->task_cv.wait(lg, [&](){
        return !dnn_rt_para_->idle_tasks.empty() || !rclcpp::ok();
      });
    }
    if (!rclcpp::ok()) {
      return task_id;
    }
    if (!dnn_rt_para_->idle_tasks.empty()) {
      alloc_task();
    }
  }

  RCLCPP_DEBUG(rclcpp::get_logger("dnn"), "Alloc task id: %d", task_id);
  if (task_id < 0 || task_id >= static_cast<int>(dnn_rt_para_->tasks.size())) {
    RCLCPP_ERROR(rclcpp::get_logger("dnn"), "Invalid task id: %d", task_id);
    return -1;
  }

  dnn_rt_para_->tasks[task_id] = std::move(task);

  return task_id;
}

int DnnNodeImpl::ReleaseTask(const TaskId& task_id) {
  RCLCPP_DEBUG(rclcpp::get_logger("dnn"), "Release task id: %d", task_id);

  if (!dnn_rt_para_ || task_id < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("dnn"), "Invalid task_id: %d", task_id);
    return -1;
  }
  auto node_task = std::make_shared<DnnNodeTask>(task_id);
  std::unique_lock<std::mutex> lg(dnn_rt_para_->task_mtx);
  if (dnn_rt_para_->running_tasks.find(task_id) ==
    dnn_rt_para_->running_tasks.end()) {
    RCLCPP_ERROR(rclcpp::get_logger("dnn"),
      "Task id: %d is not running", task_id);
    return -1;
  }
  dnn_rt_para_->idle_tasks[node_task->task_id] = node_task;
  dnn_rt_para_->running_tasks.erase(task_id);
  dnn_rt_para_->tasks[task_id] = nullptr;
  dnn_rt_para_->task_cv.notify_one();
  lg.unlock();
  RCLCPP_DEBUG(rclcpp::get_logger("dnn"),
    "idle_tasks size: %d, running_tasks size: %d",
    dnn_rt_para_->idle_tasks.size(), dnn_rt_para_->running_tasks.size());
  return -1;
}

std::shared_ptr<Task> DnnNodeImpl::GetTask(const TaskId& task_id) {
  std::shared_ptr<Task> task = nullptr;
  if (!dnn_rt_para_ || task_id < 0 ||
      task_id >= static_cast<int>(dnn_rt_para_->tasks.size())) {
    RCLCPP_ERROR(rclcpp::get_logger("dnn"), "Invalid task_id: %d", task_id);
    return task;
  }
  task = dnn_rt_para_->tasks.at(task_id);
  return task;
}

Model* DnnNodeImpl::GetModel() {
  if (dnn_rt_para_) {
    return dnn_rt_para_->model_manage;
  }
  return nullptr;
}

int DnnNodeImpl::GetModelInputSize(int32_t input_index, int& w, int& h) {
  if (!dnn_rt_para_ || !dnn_rt_para_->model_manage) {
    RCLCPP_ERROR(rclcpp::get_logger("dnn"), "Invalid input model");
    return -1;
  }
  if (input_index >= dnn_rt_para_->model_manage->GetInputCount()) {
    RCLCPP_ERROR(rclcpp::get_logger("dnn"),
    "Invalid input index: %d", input_index);
    return -1;
  }

  hbDNNTensorProperties properties;
  dnn_rt_para_->model_manage->GetInputTensorProperties(properties, input_index);
  w = properties.validShape.dimensionSize[3];
  h = properties.validShape.dimensionSize[2];

  return 0;
}

}  // namespace dnn_node
}  // namespace hobot
