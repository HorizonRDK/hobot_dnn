// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include <string>
#include <memory>
#include <fstream>
#include <vector>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "dnn_node/dnn_node.h"
#include "include/fasterrcnn_body_det_node.h"
#include "include/image_utils.h"
#include "include/fasterrcnn_kps_output_parser.h"
#include "include/image_subscriber.h"
#include <cv_bridge/cv_bridge.h>

FasterRcnnBodyDetNode::FasterRcnnBodyDetNode(
    const std::string & node_name,
    const NodeOptions & options) :
    DnnNode(node_name, options) {
  this->declare_parameter<int>("feed_type", feed_type_);
  this->declare_parameter<std::string>("image", image_);
  this->declare_parameter<int>("image_type", image_type_);
  this->declare_parameter<int>("dump_render_img", dump_render_img_);
  this->declare_parameter<int>("is_sync_mode", is_sync_mode_);

  this->get_parameter<int>("feed_type", feed_type_);
  this->get_parameter<std::string>("image", image_);
  this->get_parameter<int>("image_type", image_type_);
  this->get_parameter<int>("dump_render_img", dump_render_img_);
  this->get_parameter<int>("is_sync_mode", is_sync_mode_);

  std::stringstream ss;
  ss << "Parameter:"
  << "\n feed_type(0:local, 1:sub): " << feed_type_
  << "\n image: " << image_
  << "\n image_type: " << image_type_
  << "\n dump_render_img: " << dump_render_img_
  << "\n is_sync_mode_: " << is_sync_mode_;
  RCLCPP_WARN(rclcpp::get_logger("example"), "%s", ss.str().c_str());
}

FasterRcnnBodyDetNode::~FasterRcnnBodyDetNode() {
}

int FasterRcnnBodyDetNode::SetNodePara() {
  RCLCPP_INFO(rclcpp::get_logger("example"), "Set node para.");
  if (!dnn_node_para_ptr_) {
    return -1;
  }
  dnn_node_para_ptr_->model_file = model_file_name_;
  dnn_node_para_ptr_->model_name = model_name_;
  dnn_node_para_ptr_->model_task_type = model_task_type_;
  return 0;
}

int FasterRcnnBodyDetNode::SetOutputParser() {
  RCLCPP_INFO(rclcpp::get_logger("example"), "Set output parser.");
  // set output parser
  auto model_manage = GetModel();
  if (!model_manage || !dnn_node_para_ptr_) {
    RCLCPP_ERROR(rclcpp::get_logger("example"), "Invalid model");
    return -1;
  }

  if (model_manage->GetOutputCount() < model_output_count_) {
    RCLCPP_ERROR(rclcpp::get_logger("example"),
      "Error! Model %s output count is %d, unmatch with count %d",
      dnn_node_para_ptr_->model_name.c_str(),
      model_manage->GetOutputCount(),
      model_output_count_);
    return -1;
  }

  // set box paser
  // 使用easy dnn中定义的FaceHandDetectionOutputParser后处理进行更新
  std::shared_ptr<OutputParser> box_out_parser =
      std::make_shared<hobot::easy_dnn::FaceHandDetectionOutputParser>();
  model_manage->SetOutputParser(box_output_index_, box_out_parser);

  // set kps paser
  auto output_desc =
      std::make_shared<OutputDescription>(model_manage, kps_output_index_,
      "body_kps_branch");
  output_desc->GetDependencies().push_back(box_output_index_);
  output_desc->SetType("body_kps");
  model_manage->SetOutputDescription(output_desc);
  auto parser_para = std::make_shared<FasterRcnnKpsParserPara>();
  // get kps parser para from model
  hbDNNTensorProperties tensor_properties;
  model_manage->GetOutputTensorProperties(tensor_properties,
    kps_output_index_);
  parser_para->aligned_kps_dim.clear();
  parser_para->kps_shifts_.clear();
  for (int i = 0; i < tensor_properties.alignedShape.numDimensions; i++) {
    parser_para->aligned_kps_dim.push_back(
      tensor_properties.alignedShape.dimensionSize[i]);
  }
  for (int i = 0; i < tensor_properties.shift.shiftLen; i++) {
    parser_para->kps_shifts_.push_back(static_cast<uint8_t>(
      tensor_properties.shift.shiftData[i]));
  }

  std::stringstream ss;
  ss << "aligned_kps_dim:";
  for (const auto& val : parser_para->aligned_kps_dim) {
    ss << " " << val;
  }
  ss << "\nkps_shifts: ";
  for (const auto& val : parser_para->kps_shifts_) {
    ss << " " << val;
  }
  ss << "\n";
  RCLCPP_INFO(rclcpp::get_logger("example"), "%s", ss.str().c_str());

  std::shared_ptr<OutputParser> kps_out_parser =
      std::make_shared<FasterRcnnKpsOutputParser>(parser_para);
  model_manage->SetOutputParser(kps_output_index_, kps_out_parser);

  return 0;
}

int FasterRcnnBodyDetNode::PreProcess(
  std::vector<std::shared_ptr<DNNInput>> &inputs,
  const TaskId& task_id,
  const std::shared_ptr<std::vector<hbDNNRoi>> rois) {
  uint32_t ret = 0;
  std::shared_ptr<ModelInferTask> infer_task =
    std::dynamic_pointer_cast<ModelInferTask>(GetTask(task_id));
  if (!infer_task || (ret = infer_task->SetInputs(inputs)) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("example"), "Failed to set inputs");
    return ret;
  }
  if (rois) {
    // set roi
  }
  return ret;
}

int FasterRcnnBodyDetNode::PostProcess(
  const std::vector<std::shared_ptr<DNNResult>> &outputs) {
  RCLCPP_INFO(rclcpp::get_logger("example"),
    "outputs size: %d", outputs.size());
  if (outputs.empty() ||
    static_cast<int32_t>(outputs.size()) < model_output_count_) {
    RCLCPP_ERROR(rclcpp::get_logger("example"), "Invalid outputs");
    return -1;
  }

  {
    auto tp_now = std::chrono::system_clock::now();
    std::unique_lock<std::mutex> lk(frame_stat_mtx_);
    output_frameCount_++;
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                        tp_now - output_tp_).count();
    if (interval >= 1000) {
      RCLCPP_WARN(rclcpp::get_logger("example"),
        "Smart fps = %d", output_frameCount_);
      output_frameCount_ = 0;
      output_tp_ = std::chrono::system_clock::now();
    }
  }

  // outputs.front().size() is 3
  // 0: invalid, 1: box, 2: kps
  // box
  {
    auto *filter2d_result =
      dynamic_cast<Filter2DResult *>(outputs[box_output_index_].get());
    if (!filter2d_result) {
      RCLCPP_INFO(rclcpp::get_logger("example"), "invalid cast");
    } else {
      RCLCPP_INFO(rclcpp::get_logger("example"), "out box size: %d",
        filter2d_result->boxes.size());
      for (auto &rect : filter2d_result->boxes) {
        if (rect.left < 0) rect.left = 0;
        if (rect.top < 0) rect.top = 0;
        if (rect.right > model_input_width_) {
          rect.right = model_input_width_;
        }
        if (rect.bottom > model_input_height_) {
          rect.bottom = model_input_height_;
        }
            std::stringstream ss;
        ss << "rect: " << rect.left << " " << rect.top
            << " " << rect.right << " " << rect.bottom;
        RCLCPP_INFO(rclcpp::get_logger("example"), "%s", ss.str().c_str());
      }
    }
  }
  // kps
  {
    auto *lmk_result =
      dynamic_cast<LandmarksResult *>(outputs[kps_output_index_].get());
    if (!lmk_result) {
      RCLCPP_INFO(rclcpp::get_logger("example"), "invalid cast");
    } else {
      RCLCPP_INFO(rclcpp::get_logger("example"), "out kps size: %d",
        lmk_result->values.size());
      std::stringstream ss;
      for (const auto& value : lmk_result->values) {
        ss << "kps point: ";
        for (const auto &point : value) {
          ss << "\n" << point.x << "," << point.y << "," << point.score;
        }
        ss << "\n";
        RCLCPP_DEBUG(rclcpp::get_logger("example"),
          "FasterRcnnKpsOutputParser parse kps: %s", ss.str().c_str());
      }
    }
  }
  return 0;
}

int FasterRcnnBodyDetNode::Predict(
  std::vector<std::shared_ptr<DNNInput>> &inputs,
  const std::shared_ptr<std::vector<hbDNNRoi>> rois,
  std::vector<std::shared_ptr<DNNResult>> &outputs) {
  // 1. 申请预测task
  auto task_id = AllocTask();
  uint32_t ret = 0;
  // 2. 将准备好的数据通过前处理接口输入给模型
  ret = PreProcess(inputs, task_id, rois);
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("example"), "Run PreProcess failed!");
    return ret;
  }

  // 3. 模型推理
  ret = RunInferTask(outputs, task_id, is_sync_mode_);
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("example"), "Run RunInferTask failed!");
    ReleaseTask(task_id);
    return ret;
  }
  if (is_sync_mode_) {
    ReleaseTask(task_id);
    // 4. 通过后处理接口处理模型输出
    ret = PostProcess(outputs);
    if (ret != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("example"), "Run PostProcess failed!");
    }
  }
  return ret;
}

int FasterRcnnBodyDetNode::FeedFromLocal() {
  if (access(image_.c_str(), R_OK) == -1) {
    RCLCPP_ERROR(rclcpp::get_logger("example"),
      "Image: %s not exist!", image_.c_str());
    return -1;
  }

  RCLCPP_INFO(rclcpp::get_logger("example"),
    "Dnn node feed with local image: %s", image_.c_str());

  // 1. 将图片处理成模型输入数据类型DNNInput
  // 使用图片生成pym，NV12PyramidInput为DNNInput的子类
  std::shared_ptr<hobot::easy_dnn::NV12PyramidInput> pyramid = nullptr;
  if (static_cast<int>(ImageType::BGR) == image_type_) {
    // bgr img，支持将图片resize到模型输入size
    pyramid = ImageUtils::GetNV12Pyramid(image_, ImageType::BGR,
      model_input_height_, model_input_width_);
    if (!pyramid) {
      RCLCPP_ERROR(rclcpp::get_logger("example"),
        "Get Nv12 pym fail with image: %s", image_.c_str());
      return -1;
    }
  } else if (static_cast<int>(ImageType::NV12) == image_type_) {
    // nv12 img，不支持resize，图片size必须和模型输入size一致
    pyramid = ImageUtils::GetNV12Pyramid(image_, ImageType::NV12,
      model_input_height_, model_input_width_);
    if (!pyramid) {
      RCLCPP_ERROR(rclcpp::get_logger("example"),
        "Get Nv12 pym fail with image: %s", image_.c_str());
      return -1;
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("example"),
      "Invalid image type: %d", image_type_);
    return -1;
  }

  // 2. 使用pyramid创建DNNInput对象inputs
  // inputs将会作为模型的输入通过RunInferTask接口传入
  auto inputs = std::vector<std::shared_ptr<DNNInput>>{pyramid};
  std::vector<std::shared_ptr<DNNResult>> outputs;
  uint32_t ret = 0;
  // 3. 开始预测
  ret = Predict(inputs, nullptr, outputs);
  // 4. 处理预测结果，如渲染到图片或者发布预测结果
  if (ret != 0 || outputs.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("example"), "Run predict failed!");
    return ret;
  } else if (dump_render_img_ &&
    static_cast<int>(ImageType::BGR) == image_type_) {
    // 只支持对jpg/png等格式图片做渲染
    std::string result_image = "render.jpg";
    cv::Mat mat = cv::imread(image_, cv::IMREAD_COLOR);
    Render(mat,
          dynamic_cast<Filter2DResult *>(outputs[box_output_index_].get()),
          dynamic_cast<LandmarksResult *>(outputs[kps_output_index_].get()),
          model_input_height_, model_input_width_);
    RCLCPP_INFO(rclcpp::get_logger("example"),
      "Draw result to file: %s", result_image.c_str());
    cv::imwrite(result_image, mat);
  }
  return 0;
}

int FasterRcnnBodyDetNode::FeedFromSubscriber() {
  RCLCPP_INFO(rclcpp::get_logger("example"),
    "Dnn node feed with img subscriber");
  if (!image_subscriber_) {
    RCLCPP_ERROR(rclcpp::get_logger("example"), "Invalid img subscriber");
    return -1;
  }

  auto topics = this->get_topic_names_and_types();
  std::stringstream ss;
  ss << "\n";
  for (const auto& topic : topics) {
    ss << "topic name: " << topic.first
    << " has sub count: " << count_subscribers(topic.first)
    << " has pub count: " << count_publishers(topic.first)
    << ", type:";
    for (const auto& type : topic.second) {
      ss << " " << type;
    }
    ss << "\n";
  }
  RCLCPP_INFO(rclcpp::get_logger("example"), "%s", ss.str().c_str());

  while (rclcpp::ok()) {
    RCLCPP_INFO(rclcpp::get_logger("example"), "Get img start");
    // 1. 订阅图片消息，如果无publisher，阻塞在GetImg调用
    auto img_msg = image_subscriber_->GetImg();
    if (!img_msg) {
      continue;
    }
    if (!rclcpp::ok()) {
      return 0;
    }

    std::stringstream ss;
    ss << "Recved img encoding: " << img_msg->encoding
    << ", h: " << img_msg->height
    << ", w: " << img_msg->width
    << ", step: " << img_msg->step
    << ", frame_id: " << img_msg->header.frame_id
    << ", stamp: " << img_msg->header.stamp.sec
    << "." << img_msg->header.stamp.nanosec
    << ", data size: " << img_msg->data.size();
    RCLCPP_INFO(rclcpp::get_logger("example"), "%s", ss.str().c_str());

    auto tp_start = std::chrono::system_clock::now();

    // dump recved img msg
    // std::ofstream ofs("img." + img_msg->encoding);
    // ofs.write(reinterpret_cast<const char*>(img_msg->data.data()),
    //   img_msg->data.size());
    // auto cv_img = cv_bridge::toCvCopy(img_msg, img_msg->encoding);
    auto cv_img = cv_bridge::cvtColorForDisplay(
      cv_bridge::toCvShare(img_msg),
      "bgr8");
    // dump recved img msg after convert
    // cv::imwrite("dump_raw_" +
    //     std::to_string(img_msg->header.stamp.sec) + "." +
    //     std::to_string(img_msg->header.stamp.nanosec) + ".jpg",
    //     cv_img->image);

    {
      auto tp_now = std::chrono::system_clock::now();
      auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                          tp_now - tp_start).count();
      RCLCPP_DEBUG(rclcpp::get_logger("example"),
        "after cvtColorForDisplay cost ms: %d", interval);
    }

    // 1. 将图片处理成模型输入数据类型DNNInput
    // 使用图片生成pym，NV12PyramidInput为DNNInput的子类
    std::shared_ptr<hobot::easy_dnn::NV12PyramidInput> pyramid =
    ImageUtils::GetNV12Pyramid(cv_img->image,
      model_input_height_, model_input_width_);
    if (!pyramid) {
      RCLCPP_ERROR(rclcpp::get_logger("example"),
        "Get Nv12 pym fail with image: %s", image_.c_str());
      return -1;
    }

    {
      auto tp_now = std::chrono::system_clock::now();
      auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                          tp_now - tp_start).count();
      RCLCPP_DEBUG(rclcpp::get_logger("example"),
        "after GetNV12Pyramid cost ms: %d", interval);
    }

    // 2. 使用pyramid创建DNNInput对象inputs
    // inputs将会作为模型的输入通过RunInferTask接口传入
    auto inputs = std::vector<std::shared_ptr<DNNInput>>{pyramid};
    std::vector<std::shared_ptr<DNNResult>> outputs;
    outputs.clear();
    uint32_t ret = 0;
    // 3. 开始预测
    ret = Predict(inputs, nullptr, outputs);

    {
      auto tp_now = std::chrono::system_clock::now();
      auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                          tp_now - tp_start).count();
      RCLCPP_DEBUG(rclcpp::get_logger("example"),
        "after Predict cost ms: %d", interval);
    }

    // 4. 处理预测结果，如渲染到图片或者发布预测结果
    if (ret != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("example"), "Run predict failed!");
      return ret;
    } else if (dump_render_img_ && is_sync_mode_) {
      std::string result_image = "render_" +
        std::to_string(img_msg->header.stamp.sec) + "." +
        std::to_string(img_msg->header.stamp.nanosec) + ".jpg";
      auto mat = cv_img->image;
      Render(mat,
            dynamic_cast<Filter2DResult *>(outputs[box_output_index_].get()),
            dynamic_cast<LandmarksResult *>(outputs[kps_output_index_].get()),
            model_input_height_, model_input_width_);
      RCLCPP_INFO(rclcpp::get_logger("example"),
        "Draw result to file: %s", result_image.c_str());
      cv::imwrite(result_image, mat);
    }
  }
  RCLCPP_WARN(rclcpp::get_logger("example"), "FeedFromSubscriber done");
  return 0;
}

int FasterRcnnBodyDetNode::Render(
  cv::Mat& mat, const Filter2DResult *filter2d_result,
  const LandmarksResult *lmk_result,
  const float& model_input_h, const float& model_input_w) {
  float height_scale =
    !model_input_height_ ? 1.0f : mat.rows * 1.0f / model_input_h;
  float width_scale =
    !model_input_width_ ? 1.0f : mat.cols * 1.0f / model_input_w;
  // outputs.size() is 3
  // 0: invalid, 1: box, 2: kps
  // render box
  if (filter2d_result) {
    for (auto &rect : filter2d_result->boxes) {
      auto &color = colors[rect.perception_type % 4];
      cv::rectangle(
          mat,
          cv::Point(rect.left * width_scale, rect.top * height_scale),
          cv::Point(rect.right * width_scale, rect.bottom * height_scale),
          color, 3);
    }
  }

  // render kps
  if (lmk_result) {
    size_t lmk_num = lmk_result->values.size();
    for (size_t idx = 0; idx < lmk_num; idx++) {
      const auto& lmk = lmk_result->values.at(idx);
      auto &color = colors[idx % 4];
      for (const auto& point : lmk) {
        cv::circle(
            mat,
            cv::Point(point.x * width_scale, point.y * height_scale),
            3, color, 3);
      }
    }
  }
  return 0;
}

int FasterRcnnBodyDetNode::Run() {
  if (static_cast<int>(DnnFeedType::FROM_LOCAL) == feed_type_) {
    RCLCPP_INFO(rclcpp::get_logger("example"),
      "Dnn node feed with local image: %s", image_.c_str());
    return FeedFromLocal();
  } else if (static_cast<int>(DnnFeedType::FROM_SUB) == feed_type_) {
    RCLCPP_INFO(rclcpp::get_logger("example"),
      "Dnn node feed with subscription");
    image_subscriber_ = std::make_shared<ImageSubscriber>();
    auto predict_task = std::make_shared<std::thread>(
      std::bind(&FasterRcnnBodyDetNode::FeedFromSubscriber, this));
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(image_subscriber_);
    exec.spin();
    if (predict_task && predict_task->joinable()) {
      predict_task.reset();
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("example"),
      "Invalid feed_type:%d", feed_type_);
    return -1;
  }

  return 0;
}
