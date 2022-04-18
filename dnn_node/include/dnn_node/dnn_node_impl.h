// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef DNN_NODE_IMPL_H_
#define DNN_NODE_IMPL_H_

#include <vector>
#include <memory>
#include <unordered_map>

#include "dnn_node/dnn_node_data.h"
#include "util/threads/threadpool.h"

namespace hobot {
namespace dnn_node {

struct DnnNodeRunTimePara;
struct ThreadPool;

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

class DnnNodeImpl {
 public:
  explicit DnnNodeImpl(std::shared_ptr<DnnNodePara>& dnn_node_para_ptr);

  ~DnnNodeImpl();

  int ModelInit();

 public:
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

  int TaskInit();

  // 配置预测任务的输入数据
  // - 参数
  //   - [in] inputs 输入数据智能指针列表。
  //   - [in] task_id 预测任务ID。
  //   - [in] rois 抠图roi数据，只对抠图检测模型有效。
  int PreProcess(std::vector<std::shared_ptr<DNNInput>> &inputs,
            const TaskId& task_id,
            const std::shared_ptr<std::vector<hbDNNRoi>> rois = nullptr);

  // 执行推理任务
  // - 参数
  //   - [out] outputs 输出数据智能指针列表，同步模式有效。
  //   - [in] task_id 预测任务ID。
  //   - [in] is_sync_mode 预测模式，true为同步模式，false为异步模式。
  //   - [in] timeout_ms 预测推理超时时间。
  int RunInferTask(std::shared_ptr<DnnNodeOutput> &sync_output,
      const TaskId& task_id,
      std::function<int(const std::shared_ptr<DnnNodeOutput> &)> post_process,
      const bool is_sync_mode = true,
      const int timeout_ms = 1000);

  // 使用通过SetInputs输入给模型的数据进行推理
  // outputs为模型输出，timeout_ms为推理超时时间
  int RunInfer(std::vector<std::shared_ptr<DNNResult>> &outputs,
                   const std::shared_ptr<Task>& task,
                   const int timeout_ms);

  // 获取dnn node管理和推理使用的模型。
  Model* GetModel();

  // 获取模型的输入size
  // - 参数
  //   - [in] input_index 获取的输入索引，一个模型可能有多个输入。
  //   - [out] w 模型输入的宽度。
  //   - [out] h 模型输入的高度。
  int GetModelInputSize(int32_t input_index, int& w, int& h);

 private:
  std::shared_ptr<DnnNodePara> dnn_node_para_ptr_ = nullptr;
  std::shared_ptr<DnnNodeRunTimePara> dnn_rt_para_ = nullptr;
  std::shared_ptr<ThreadPool> thread_pool_ = nullptr;
};

}  // namespace dnn_node
}  // namespace hobot
#endif  // DNN_NODE_IMPL_H_
