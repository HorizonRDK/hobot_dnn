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

#include "rclcpp/rclcpp.hpp"

#include "easy_dnn/model.h"
#include "dnn_node/dnn_node.h"
#include "util/threads/threadpool.h"
#include "hobotlog/hobotlog.hpp"

namespace hobot {
namespace dnn_node {

struct DnnNodeTask {
  explicit DnnNodeTask(TaskId id) {
    task_id = id;
    alloc_tp = std::chrono::system_clock::now();
  }
  TaskId task_id = -1;
  std::chrono::high_resolution_clock::time_point alloc_tp;
};

struct DnnNodeRunTimePara {
  // 使用模型文件加载后的模型列表
  std::vector<Model *> models_load;

  // 根据model_name解析出的需要管理和推理使用的模型
  Model* model_manage = nullptr;

  // 一个DNNNode实例只支持一种ModelTask类型
  std::vector<std::shared_ptr<Task>> tasks{};

  // todo 20220228
  // Add task release strategy according to task alloc_tp to
  // avoid task abnormal leakage
  std::unordered_map<TaskId, std::shared_ptr<DnnNodeTask>> idle_tasks{};
  std::unordered_map<TaskId, std::shared_ptr<DnnNodeTask>> running_tasks{};
  std::mutex task_mtx;
  std::condition_variable task_cv;
};

struct ThreadPool {
  hobot::CThreadPool msg_handle_;
  std::mutex msg_mutex_;
  int msg_limit_count_ = 10;
};

DnnNode::DnnNode(
  const std::string & node_name,
  const NodeOptions & options) :
  rclcpp::Node(node_name, options) {
  NodeInit();
}

DnnNode::DnnNode(
  const std::string & node_name,
  const std::string & namespace_,
  const NodeOptions & options) :
  rclcpp::Node(node_name, namespace_, options) {
  NodeInit();
}

DnnNode::~DnnNode() {
  if (!dnn_rt_para_) {
    ModelManager::GetInstance()->OffLoad(dnn_rt_para_->models_load);
  }
}

void DnnNode::NodeInit() {
  dnn_node_para_ptr_ = std::make_shared<DnnNodePara>();
  dnn_rt_para_ = std::make_shared<DnnNodeRunTimePara>();
  thread_pool_ = std::make_shared<ThreadPool>();
}

int DnnNode::ModelInit() {
  RCLCPP_INFO(rclcpp::get_logger("dnn"), "Model init.");
  if (!dnn_node_para_ptr_ || !dnn_rt_para_) {
    RCLCPP_ERROR(rclcpp::get_logger("dnn"), "Invalid node para!");
    return -1;
  }

  // 1. 加载模型hbm文件，一个hbm中可能包含多个模型
  ModelManager::GetInstance()->Load(dnn_rt_para_->models_load,
  dnn_node_para_ptr_->model_file);

  // 2. 根据模型名，加载实际需要管理的模型
  const auto& model_name = dnn_node_para_ptr_->model_name;
  for (auto model : dnn_rt_para_->models_load) {
    if (model->GetName().find(model_name) == std::string::npos) {
      continue;
    }
    dnn_rt_para_->model_manage = model;
    break;
  }
  if (!dnn_rt_para_->model_manage) {
    RCLCPP_ERROR(rclcpp::get_logger("dnn"),
      "Find model: %s fail!", model_name.c_str());
    return -1;
  }

  return 0;
}

int DnnNode::TaskInit() {
  RCLCPP_INFO(rclcpp::get_logger("dnn"), "Task init.");
  if (!dnn_node_para_ptr_ || !dnn_rt_para_ ||
    dnn_node_para_ptr_->task_num < 1) {
    RCLCPP_ERROR(rclcpp::get_logger("dnn"), "Invalid node para!");
    return -1;
  }

  // 1. 为每个模型创建task
  const auto& model_name = dnn_node_para_ptr_->model_name;
  // 根据模型类型选择接口创建task
  // 为task添加model
  if (ModelTaskType::ModelInferType == dnn_node_para_ptr_->model_task_type) {
    for (int idx = 0; idx < dnn_node_para_ptr_->task_num; idx++) {
      auto task =
        TaskManager::GetInstance()->GetModelInferTask(
        dnn_node_para_ptr_->timeout_ms);
      task->SetModel(dnn_rt_para_->model_manage);
      dnn_rt_para_->tasks.emplace_back(task);
    }
  } else if (ModelTaskType::ModelRoiInferType ==
    dnn_node_para_ptr_->model_task_type) {
    for (int idx = 0; idx < dnn_node_para_ptr_->task_num; idx++) {
      auto task =
      TaskManager::GetInstance()->GetModelRoiInferTask(
        dnn_node_para_ptr_->timeout_ms);
      task->SetModel(dnn_rt_para_->model_manage);
      dnn_rt_para_->tasks.emplace_back(task);
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("dnn"), "Invalid model task type [%d]",
      static_cast<int>(dnn_node_para_ptr_->model_task_type));
  }
  if (dnn_rt_para_->tasks.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("dnn"), "Failed to get infer task");
    return -1;
  }

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

  return 0;
}

int DnnNode::Init() {
  RCLCPP_INFO(rclcpp::get_logger("dnn"), "Node init.");

  int ret = 0;
  // 1. set model info in node para
  ret = SetNodePara();
  if (ret != 0) {
    return ret;
  }

  // 2. model init
  ret = ModelInit();
  if (ret != 0) {
    return ret;
  }

  // 3. set output parser
  ret = SetOutputParser();
  if (ret != 0) {
    return ret;
  }

  // 4. task init
  ret = TaskInit();
  if (ret != 0) {
    return ret;
  }

  // set log level of easy dnn
  SetLogLevel(HOBOT_LOG_ERROR);
  return ret;
}

int DnnNode::RunInferTask(
  std::vector<std::shared_ptr<DNNResult>> &sync_outputs,
  const TaskId& task_id,
  const bool is_sync_mode,
  const int timeout_ms) {
  if (!dnn_rt_para_) {
    return -1;
  }
  if (is_sync_mode) {
    return RunInfer(sync_outputs, GetTask(task_id), timeout_ms);
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
    auto infer_task = [this, task_id, timeout_ms](){
      std::vector<std::shared_ptr<DNNResult>> aync_outputs;
      if (RunInfer(aync_outputs, GetTask(task_id), timeout_ms) != 0) {
        ReleaseTask(task_id);
        RCLCPP_ERROR(rclcpp::get_logger("dnn"), "Run infer fail\n");
      } else {
        ReleaseTask(task_id);
        PostProcess(aync_outputs);
      }
    };
    thread_pool_->msg_handle_.PostTask(infer_task);
  }
  return 0;
}

int DnnNode::RunInfer(std::vector<std::shared_ptr<DNNResult>> &outputs,
                    const std::shared_ptr<Task>& node_task,
                    const int timeout_ms) {
  if (!dnn_node_para_ptr_ || !node_task) {
    RCLCPP_ERROR(rclcpp::get_logger("dnn"), "Invalid infer task\n");
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

TaskId DnnNode::AllocTask(int timeout_ms) {
  RCLCPP_INFO(rclcpp::get_logger("dnn"), "Alloc task");
  TaskId task_id = -1;
  if (!dnn_rt_para_) {
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

  RCLCPP_INFO(rclcpp::get_logger("dnn"), "Alloc task id: %d", task_id);
  return task_id;
}

int DnnNode::ReleaseTask(const TaskId& task_id) {
  RCLCPP_INFO(rclcpp::get_logger("dnn"), "Release task id: %d", task_id);

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
  dnn_rt_para_->task_cv.notify_one();
  lg.unlock();
  RCLCPP_INFO(rclcpp::get_logger("dnn"),
    "idle_tasks size: %d, running_tasks size: %d",
    dnn_rt_para_->idle_tasks.size(), dnn_rt_para_->running_tasks.size());
  return -1;
}

std::shared_ptr<Task> DnnNode::GetTask(const TaskId& task_id) {
  std::shared_ptr<Task> task = nullptr;
  if (!dnn_rt_para_ || task_id < 0 || task_id >= dnn_rt_para_->tasks.size()) {
    RCLCPP_ERROR(rclcpp::get_logger("dnn"), "Invalid task_id: %d", task_id);
    return task;
  }
  task = dnn_rt_para_->tasks.at(task_id);
  return task;
}

Model* DnnNode::GetModel() {
  if (dnn_rt_para_) {
    return dnn_rt_para_->model_manage;
  }
  return nullptr;
}

}  // namespace dnn_node
}  // namespace hobot
