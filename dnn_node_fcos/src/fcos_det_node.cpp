// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include "include/fcos_det_node.h"

#include <unistd.h>

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "dnn_node/dnn_node.h"
#include "include/image_utils.h"
#include "rclcpp/rclcpp.hpp"
#ifdef CV_BRIDGE_PKG_ENABLED
#include <cv_bridge/cv_bridge.h>
#endif
#include "include/fcos_output_parser.h"

using hobot::easy_dnn::OutputDescription;
using hobot::easy_dnn::OutputParser;

FcosDetNode::FcosDetNode(const std::string &node_name,
                         const NodeOptions &options)
    : DnnNode(node_name, options) {
  this->declare_parameter<std::string>("config_file_path", config_file_path_);
  this->declare_parameter<int>("shared_men", shared_mem_);
  this->declare_parameter<int>("is_sync_mode", is_sync_mode_);
  this->declare_parameter<int>("feed_type", feed_type_);
  this->declare_parameter<std::string>("image", image_);
  this->declare_parameter<int>("image_type", image_type_);
  this->declare_parameter<int>("dump_render_img", dump_render_img_);

  this->get_parameter<std::string>("config_file_path", config_file_path_);
  this->get_parameter<int>("shared_men", shared_mem_);
  this->get_parameter<int>("is_sync_mode", is_sync_mode_);
  this->get_parameter<int>("feed_type", feed_type_);
  this->get_parameter<std::string>("image", image_);
  this->get_parameter<int>("image_type", image_type_);
  this->get_parameter<int>("dump_render_img", dump_render_img_);
  model_file_name_ = config_file_path_ + "/fcos_512x512_nv12.bin";

  std::stringstream ss;
  ss << "Parameter:"
     << "\nconfig_file_path_:" << config_file_path_
     << "\nshared_men:" << shared_mem_ << "\nis_sync_mode_: " << is_sync_mode_
     << "\nfeed_type_: " << feed_type_ << "\nimage_: " << image_
     << "\nimage_type_: " << image_type_
     << "\ndump_render_img_: " << dump_render_img_
     << "\nmodel_file_name_: " << model_file_name_;
  RCLCPP_WARN(rclcpp::get_logger("fcos_example"), "%s", ss.str().c_str());
  if (Start()) {
    RCLCPP_WARN(rclcpp::get_logger("fcos_example"), "start success!!!");
  } else {
    RCLCPP_WARN(rclcpp::get_logger("fcos_example"), "start fail!!!");
  }
}

FcosDetNode::~FcosDetNode() {}

void FcosDetNode::GetConfig() {
  std::string fcos_config_ = config_file_path_ + "/fcos_config.json";
  std::ifstream fs_config;
  fs_config.open(fcos_config_, std::ios::in);
  if (!fs_config.is_open()) {
    RCLCPP_ERROR(rclcpp::get_logger("fcos_example"),
                 "open config file %s fail!", fcos_config_.c_str());
    return;
  }

  std::string line;
  std::string detlist_filepath;
  while (std::getline(fs_config, line)) {
    if (line.find("score_threshold") != std::string::npos) {
      size_t nEndPos = line.find(":");
      if (nEndPos == std::string::npos) continue;
      line = line.substr(nEndPos + 1);
      score_threshold_ = atof(line.c_str());
      RCLCPP_WARN(rclcpp::get_logger("fcos_example"), "score_threshold_: %f",
                  score_threshold_);
    }
    std::string tmp = "\"det_type_list\":";
    if (line.find(tmp) != std::string::npos) {
      size_t nEndPos = line.find(tmp);
      line = line.substr(nEndPos + tmp.length());
      nEndPos = line.find("\"");
      if (nEndPos == std::string::npos) continue;
      line = line.substr(nEndPos + 1);
      nEndPos = line.find("\"");
      if (nEndPos == std::string::npos) continue;
      line = line.substr(0, nEndPos);
      detlist_filepath = config_file_path_ + "/" + line;
      RCLCPP_WARN(rclcpp::get_logger("fcos_example"), "detlist file %s",
                  detlist_filepath.c_str());
    }
  }
  fs_config.close();

  std::ifstream fs_det;
  fs_det.open(detlist_filepath, std::ios::in);
  if (!fs_det.is_open()) {
    RCLCPP_ERROR(rclcpp::get_logger("fcos_example"),
                 "open config file %s fail!", detlist_filepath.c_str());
    return;
  }
  read_detlist_success_ = true;
  dettype_list_.clear();
  while (std::getline(fs_det, line)) {
    dettype_list_[line] = 1;
  }
  fs_det.close();

  RCLCPP_INFO(rclcpp::get_logger("fcos_example"), "need to detect type");
  for (auto &item : dettype_list_) {
    RCLCPP_INFO(rclcpp::get_logger("fcos_example"), "%s", item.first.c_str());
  }
}

int FcosDetNode::Start() {
  int ret = Init();
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("fcos_example"), "Init failed!");
    return ret;
  }

  ret = GetModelInputSize(0, model_input_width_, model_input_height_);
  if (ret < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("fcos_example"),
                 "Get model input size fail!");
    return ret;
  }
  RCLCPP_INFO(rclcpp::get_logger("fcos_example"),
              "The model input width is %d and height is %d",
              model_input_width_, model_input_height_);

  if (static_cast<int>(DnnFeedType::FROM_LOCAL) != feed_type_ &&
      static_cast<int>(DnnFeedType::FROM_SUB) != feed_type_) {
    RCLCPP_ERROR(rclcpp::get_logger("fcos_example"), "Invalid feed_type:%d",
                 feed_type_);
    return -1;
  }
  if (static_cast<int>(DnnFeedType::FROM_LOCAL) == feed_type_) {
    FeedFromLocal();
    return 0;
  }

  // DnnFeedType::FROM_SUB  == feed_type_
  RCLCPP_INFO(rclcpp::get_logger("fcos_example"),
              "Dnn node feed with subscription");
  if (shared_mem_) {
#ifdef SHARED_MEM_ENABLED
    RCLCPP_WARN(rclcpp::get_logger("fcos_example"),
                "Create hbmem_subscription with topic_name: %s",
                sharedmem_img_topic_name_.c_str());
    sharedmem_img_subscription_ =
        this->create_subscription_hbmem<hbm_img_msgs::msg::HbmMsg1080P>(
            sharedmem_img_topic_name_, 10,
            std::bind(&FcosDetNode::SharedMemImgProcess, this,
                      std::placeholders::_1));
#else
    RCLCPP_ERROR(rclcpp::get_logger("fcos_example"), "Unsupport shared mem");
#endif
  } else {
    RCLCPP_WARN(rclcpp::get_logger("fcos_example"),
                "Create subscription with topic_name: %s",
                ros_img_topic_name_.c_str());
    ros_img_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        ros_img_topic_name_, 10,
        std::bind(&FcosDetNode::RosImgProcess, this, std::placeholders::_1));
  }

  return 0;
}

int FcosDetNode::SetNodePara() {
  RCLCPP_INFO(rclcpp::get_logger("fcos_example"), "Set node para.");
  if (!dnn_node_para_ptr_) {
    return -1;
  }
  dnn_node_para_ptr_->model_file = model_file_name_;
  dnn_node_para_ptr_->model_name = model_name_;
  dnn_node_para_ptr_->model_task_type = model_task_type_;
  dnn_node_para_ptr_->task_num = 2;
  return 0;
}

int FcosDetNode::SetOutputParser() {
  // set output parser
  auto model_manage = GetModel();
  if (!model_manage || !dnn_node_para_ptr_) {
    RCLCPP_ERROR(rclcpp::get_logger("fcos_example"), "Invalid model");
    return -1;
  }

  if (model_manage->GetOutputCount() < model_output_count_) {
    RCLCPP_ERROR(rclcpp::get_logger("fcos_example"),
                 "Error! Model %s output count is %d, unmatch with count %d",
                 dnn_node_para_ptr_->model_name.c_str(),
                 model_manage->GetOutputCount(), model_output_count_);
    return -1;
  }

  for (int i = 0; i < fcos_output_index_; ++i) {
    std::shared_ptr<OutputParser> assist_parser =
        std::make_shared<FcosDetectionAssistParser>();
    model_manage->SetOutputParser(i, assist_parser);
  }
  // set fcos paser
  auto output_desc = std::make_shared<OutputDescription>(
      model_manage, fcos_output_index_, "fcos_branch");
  for (int i = 0; i < fcos_output_index_; ++i) {
    output_desc->GetDependencies().push_back(i);
  }
  output_desc->SetType("fcos");
  model_manage->SetOutputDescription(output_desc);
  std::shared_ptr<OutputParser> fcos_out_parser =
      std::make_shared<FcosDetectionOutputParser>();
  model_manage->SetOutputParser(fcos_output_index_, fcos_out_parser);

  return 0;
}

int FcosDetNode::PostProcess(
    const std::shared_ptr<DnnNodeOutput> &node_output) {
  auto fcos_output = std::dynamic_pointer_cast<FcosOutput>(node_output);
  if (fcos_output) {
    std::stringstream ss;
    ss << "Output from image_name: " << fcos_output->image_name;
    if (fcos_output->image_msg_header) {
      ss << ", frame_id: " << fcos_output->image_msg_header->frame_id
         << ", stamp: " << fcos_output->image_msg_header->stamp.sec << "."
         << fcos_output->image_msg_header->stamp.nanosec;
    }
    RCLCPP_INFO(rclcpp::get_logger("fcos_example"), "%s", ss.str().c_str());
  }

  const auto &outputs = node_output->outputs;
  RCLCPP_INFO(rclcpp::get_logger("fcos_example"), "outputs size: %d",
              outputs.size());
  if (outputs.empty() ||
      static_cast<int32_t>(outputs.size()) < model_output_count_) {
    RCLCPP_ERROR(rclcpp::get_logger("fcos_example"), "Invalid outputs");
    return -1;
  }

  {
    auto tp_now = std::chrono::system_clock::now();
    std::unique_lock<std::mutex> lk(frame_stat_mtx_);
    output_frameCount_++;
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                        tp_now - output_tp_)
                        .count();
    if (interval >= 1000) {
      RCLCPP_WARN(rclcpp::get_logger("fcos_example"), "Smart fps = %d",
                  output_frameCount_);
      output_frameCount_ = 0;
      output_tp_ = std::chrono::system_clock::now();
    }
  }

  // box
  {
    auto *det_result =
        dynamic_cast<FcosDetResult *>(outputs[fcos_output_index_].get());
    if (!det_result) {
      RCLCPP_INFO(rclcpp::get_logger("fcos_example"), "invalid cast");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("fcos_example"), "out box size: %d",
                det_result->boxes.size());
    if (dump_render_img_) return 0;
    for (auto &rect : det_result->boxes) {
      if (rect.x1 < 0) rect.x1 = 0;
      if (rect.y1 < 0) rect.y1 = 0;
      if (rect.x2 > model_input_width_) {
        rect.x2 = model_input_width_;
      }
      if (rect.y2 > model_input_height_) {
        rect.y2 = model_input_height_;
      }
      std::stringstream ss;
      ss << "det rect: " << rect.x1 << " " << rect.y1 << " " << rect.x2 << " "
         << rect.y2 << ", det type：" << rect.category_name
         << ", score:" << rect.conf;
      RCLCPP_INFO(rclcpp::get_logger("fcos_example"), "%s", ss.str().c_str());
    }
  }

  return 0;
}

int FcosDetNode::Predict(std::vector<std::shared_ptr<DNNInput>> &inputs,
                         const std::shared_ptr<std::vector<hbDNNRoi>> rois,
                         std::shared_ptr<DnnNodeOutput> dnn_output) {
  return Run(inputs, dnn_output, rois, is_sync_mode_ == 1 ? true : false);
}

int FcosDetNode::FeedFromLocal() {
  if (access(image_.c_str(), R_OK) == -1) {
    RCLCPP_ERROR(rclcpp::get_logger("fcos_example"), "Image: %s not exist!",
                 image_.c_str());
    return -1;
  }

  RCLCPP_INFO(rclcpp::get_logger("fcos_example"),
              "Dnn node feed with local image: %s", image_.c_str());

  // 1. 将图片处理成模型输入数据类型DNNInput
  // 使用图片生成pym，NV12PyramidInput为DNNInput的子类
  std::shared_ptr<hobot::easy_dnn::NV12PyramidInput> pyramid = nullptr;
  if (static_cast<int>(ImageType::BGR) == image_type_) {
    // bgr img，支持将图片resize到模型输入size
    pyramid = ImageUtils::GetNV12Pyramid(
        image_, ImageType::BGR, model_input_height_, model_input_width_);
    if (!pyramid) {
      RCLCPP_ERROR(rclcpp::get_logger("fcos_example"),
                   "Get Nv12 pym fail with image: %s", image_.c_str());
      return -1;
    }
  } else if (static_cast<int>(ImageType::NV12) == image_type_) {
    // nv12 img，不支持resize，图片size必须和模型输入size一致
    pyramid = ImageUtils::GetNV12Pyramid(
        image_, ImageType::NV12, model_input_height_, model_input_width_);
    if (!pyramid) {
      RCLCPP_ERROR(rclcpp::get_logger("fcos_example"),
                   "Get Nv12 pym fail with image: %s", image_.c_str());
      return -1;
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("fcos_example"), "Invalid image type: %d",
                 image_type_);
    return -1;
  }

  // 2. 使用pyramid创建DNNInput对象inputs
  // inputs将会作为模型的输入通过RunInferTask接口传入
  auto inputs = std::vector<std::shared_ptr<DNNInput>>{pyramid};
  auto dnn_output = std::make_shared<FcosOutput>();
  dnn_output->image_name = image_;
  uint32_t ret = 0;
  // 3. 开始预测
  ret = Predict(inputs, nullptr, dnn_output);
  // 4. 处理预测结果，如渲染到图片或者发布预测结果
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("fcos_example"), "Run predict failed!");
    return ret;
  }
  if (dump_render_img_ && static_cast<int>(ImageType::BGR) == image_type_) {
    // 只支持对jpg/png等格式图片做渲染
    std::string result_image = "render.jpg";
    cv::Mat mat = cv::imread(image_, cv::IMREAD_COLOR);
    Render(mat, dynamic_cast<FcosDetResult *>(
                    dnn_output->outputs[fcos_output_index_].get()));
    RCLCPP_INFO(rclcpp::get_logger("fcos_example"), "Draw result to file: %s",
                result_image.c_str());
    cv::imwrite(result_image, mat);
  }
  return 0;
}

void FcosDetNode::RosImgProcess(
    const sensor_msgs::msg::Image::ConstSharedPtr img_msg) {
  if (!img_msg) {
    RCLCPP_DEBUG(rclcpp::get_logger("fcos_example"), "Get img failed");
    return;
  }

  if (!rclcpp::ok()) {
    return;
  }

  {
    auto tp_now = std::chrono::system_clock::now();
    std::unique_lock<std::mutex> lk(sub_frame_stat_mtx_);
    sub_img_frameCount_++;
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                        tp_now - sub_img_tp_)
                        .count();
    if (interval >= 1000) {
      RCLCPP_WARN(rclcpp::get_logger("img_sub"), "Sub img fps = %d",
                  sub_img_frameCount_);
      sub_img_frameCount_ = 0;
      sub_img_tp_ = std::chrono::system_clock::now();
    }
  }

  std::stringstream ss;
  ss << "Recved img encoding: " << img_msg->encoding
     << ", h: " << img_msg->height << ", w: " << img_msg->width
     << ", step: " << img_msg->step
     << ", frame_id: " << img_msg->header.frame_id
     << ", stamp: " << img_msg->header.stamp.sec << "."
     << img_msg->header.stamp.nanosec
     << ", data size: " << img_msg->data.size();
  RCLCPP_INFO(rclcpp::get_logger("fcos_example"), "%s", ss.str().c_str());

  // dump recved img msg
  // std::ofstream ofs("img." + img_msg->encoding);
  // ofs.write(reinterpret_cast<const char*>(img_msg->data.data()),
  //   img_msg->data.size());

  auto tp_start = std::chrono::system_clock::now();

  // 1. 将图片处理成模型输入数据类型DNNInput
  // 使用图片生成pym，NV12PyramidInput为DNNInput的子类
  std::shared_ptr<hobot::easy_dnn::NV12PyramidInput> pyramid = nullptr;
  if ("rgb8" == img_msg->encoding || "bgr8" == img_msg->encoding) {
#ifdef CV_BRIDGE_PKG_ENABLED
    auto cv_img =
        cv_bridge::cvtColorForDisplay(cv_bridge::toCvShare(img_msg), "bgr8");
    {
      auto tp_now = std::chrono::system_clock::now();
      auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                          tp_now - tp_start)
                          .count();
      RCLCPP_DEBUG(rclcpp::get_logger("fcos_example"),
                   "after cvtColorForDisplay cost ms: %d", interval);
    }

    pyramid = ImageUtils::GetNV12Pyramid(cv_img->image, model_input_height_,
                                         model_input_width_);
#else
    RCLCPP_ERROR(rclcpp::get_logger("fcos_example"), "Unsupport cv bridge");
#endif
  } else if ("nv12" == img_msg->encoding) {
    pyramid = ImageUtils::GetNV12PyramidFromNV12Img(
        reinterpret_cast<const char *>(img_msg->data.data()), img_msg->height,
        img_msg->width, model_input_height_, model_input_width_);
  }

  if (!pyramid) {
    RCLCPP_ERROR(rclcpp::get_logger("fcos_example"), "Get Nv12 pym fail");
    return;
  }

  {
    auto tp_now = std::chrono::system_clock::now();
    auto interval =
        std::chrono::duration_cast<std::chrono::milliseconds>(tp_now - tp_start)
            .count();
    RCLCPP_DEBUG(rclcpp::get_logger("fcos_example"),
                 "after GetNV12Pyramid cost ms: %d", interval);
  }

  // 2. 使用pyramid创建DNNInput对象inputs
  // inputs将会作为模型的输入通过RunInferTask接口传入
  auto inputs = std::vector<std::shared_ptr<DNNInput>>{pyramid};
  auto dnn_output = std::make_shared<FcosOutput>();
  dnn_output->image_msg_header = std::make_shared<std_msgs::msg::Header>();
  // dnn_output->image_msg_header->set__frame_id(std::to_string(img_msg->index));
  // dnn_output->image_msg_header->set__stamp(img_msg->time_stamp);
  dnn_output->image_msg_header->set__frame_id(img_msg->header.frame_id);
  dnn_output->image_msg_header->set__stamp(img_msg->header.stamp);

  // 3. 开始预测
  uint32_t ret = Predict(inputs, nullptr, dnn_output);
  {
    auto tp_now = std::chrono::system_clock::now();
    auto interval =
        std::chrono::duration_cast<std::chrono::milliseconds>(tp_now - tp_start)
            .count();
    RCLCPP_DEBUG(rclcpp::get_logger("fcos_example"),
                 "after Predict cost ms: %d", interval);
  }

  // 4. 处理预测结果，如渲染到图片或者发布预测结果
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("fcos_example"), "Run predict failed!");
    return;
  }
  if (dump_render_img_ && is_sync_mode_) {
    std::string result_image =
        "render_" + std::to_string(img_msg->header.stamp.sec) + "." +
        std::to_string(img_msg->header.stamp.nanosec) + ".jpg";

    if ("rgb8" == img_msg->encoding || "bgr8" == img_msg->encoding) {
#ifdef CV_BRIDGE_PKG_ENABLED
      auto cv_img =
          cv_bridge::cvtColorForDisplay(cv_bridge::toCvShare(img_msg), "bgr8");
      auto mat = cv_img->image;
      Render(mat, dynamic_cast<FcosDetResult *>(
                      dnn_output->outputs[fcos_output_index_].get()));
      RCLCPP_DEBUG(rclcpp::get_logger("fcos_example"),
                   "Draw result to file: %s", result_image.c_str());
      cv::imwrite(result_image, mat);
#endif
    } else if ("nv12" == img_msg->encoding) {
      RCLCPP_INFO(rclcpp::get_logger("fcos_example"),
                  "Dnn node handle predict result nv12");
      auto img_y_size = img_msg->height * img_msg->width;
      auto img_uv_size = img_y_size / 2;
      cv::Mat nv12(img_msg->height * 3 / 2, img_msg->width, CV_8UC1);
      memcpy(nv12.ptr<uint8_t>(), img_msg->data.data(), img_y_size);
      memcpy(nv12.ptr<uint8_t>() + img_y_size,
             img_msg->data.data() + img_y_size, img_uv_size);
      cv::Mat bgr;
      cv::cvtColor(nv12, bgr, CV_YUV2BGR_NV12);
      auto &mat = bgr;
      Render(mat, dynamic_cast<FcosDetResult *>(
                      dnn_output->outputs[fcos_output_index_].get()));
      RCLCPP_DEBUG(rclcpp::get_logger("fcos_example"),
                   "Draw result to file: %s", result_image.c_str());
      cv::imwrite(result_image, mat);
    }
  }
}

#ifdef SHARED_MEM_ENABLED
void FcosDetNode::SharedMemImgProcess(
    const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr img_msg) {
  if (!img_msg) {
    return;
  }

  if (!rclcpp::ok()) {
    return;
  }
  {
    auto tp_now = std::chrono::system_clock::now();
    std::unique_lock<std::mutex> lk(sub_frame_stat_mtx_);
    sub_img_frameCount_++;
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                        tp_now - sub_img_tp_)
                        .count();
    if (interval >= 1000) {
      RCLCPP_WARN(rclcpp::get_logger("img_sub"), "Sub img fps = %d",
                  sub_img_frameCount_);
      sub_img_frameCount_ = 0;
      sub_img_tp_ = std::chrono::system_clock::now();
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("fcos_example"), "go into shared mem");
  auto tp_start = std::chrono::system_clock::now();

  // 1. 将图片处理成模型输入数据类型DNNInput
  // 使用图片生成pym，NV12PyramidInput为DNNInput的子类
  std::shared_ptr<hobot::easy_dnn::NV12PyramidInput> pyramid = nullptr;
  if ("nv12" ==
      std::string(reinterpret_cast<const char *>(img_msg->encoding.data()))) {
    pyramid = ImageUtils::GetNV12PyramidFromNV12Img(
        reinterpret_cast<const char *>(img_msg->data.data()), img_msg->height,
        img_msg->width, model_input_height_, model_input_width_);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("fcos_example"),
                 "shared mem only support nv12 img, Unsupported: %s",
                 img_msg->encoding.data());
    return;
  }

  if (!pyramid) {
    RCLCPP_ERROR(rclcpp::get_logger("fcos_example"), "Get Nv12 pym fail!");
    return;
  }

  {
    auto tp_now = std::chrono::system_clock::now();
    auto interval =
        std::chrono::duration_cast<std::chrono::milliseconds>(tp_now - tp_start)
            .count();
    RCLCPP_DEBUG(rclcpp::get_logger("fcos_example"),
                 "after GetNV12Pyramid cost ms: %d", interval);
  }

  // 2. 使用pyramid创建DNNInput对象inputs
  // inputs将会作为模型的输入通过RunInferTask接口传入
  auto inputs = std::vector<std::shared_ptr<DNNInput>>{pyramid};
  auto dnn_output = std::make_shared<FcosOutput>();
  dnn_output->image_msg_header = std::make_shared<std_msgs::msg::Header>();
  dnn_output->image_msg_header->set__frame_id(std::to_string(img_msg->index));
  dnn_output->image_msg_header->set__stamp(img_msg->time_stamp);

  // 3. 开始预测
  int ret = Predict(inputs, nullptr, dnn_output);
  {
    auto tp_now = std::chrono::system_clock::now();
    auto interval =
        std::chrono::duration_cast<std::chrono::milliseconds>(tp_now - tp_start)
            .count();
    RCLCPP_DEBUG(rclcpp::get_logger("fcos_example"),
                 "after Predict cost ms: %d", interval);
  }

  // 4. 处理预测结果，如渲染到图片或者发布预测结果
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("fcos_example"), "Run predict failed!");
    return;
  }
  if (dump_render_img_ && is_sync_mode_) {
    std::string result_image =
        "render_" + std::to_string(img_msg->index) + ".jpg";
    if ("nv12" ==
        std::string(reinterpret_cast<const char *>(img_msg->encoding.data()))) {
      auto img_y_size = img_msg->height * img_msg->width;
      auto img_uv_size = img_y_size / 2;
      cv::Mat nv12(img_msg->height * 3 / 2, img_msg->width, CV_8UC1);
      memcpy(nv12.ptr<uint8_t>(), img_msg->data.data(), img_y_size);
      memcpy(nv12.ptr<uint8_t>() + img_y_size,
             img_msg->data.data() + img_y_size, img_uv_size);
      cv::Mat bgr;
      cv::cvtColor(nv12, bgr, CV_YUV2BGR_NV12);
      auto &mat = bgr;
      Render(mat, dynamic_cast<FcosDetResult *>(
                      dnn_output->outputs[fcos_output_index_].get()));
      RCLCPP_INFO(rclcpp::get_logger("fcos_example"), "Draw result to file: %s",
                  result_image.c_str());
      cv::imwrite(result_image, mat);
    }
  }
}
#endif

int FcosDetNode::Render(cv::Mat &mat, const FcosDetResult *det_result) {
  // render box
  if (!det_result) {
    return -1;
  }
  int image_width = mat.cols;
  int image_height = mat.rows;
  float factor = image_width * 1.0 / model_input_width_;
  for (auto &rect : det_result->boxes) {
    // if (rect.category_name.find("person") == std::string::npos) continue;
    if (read_detlist_success_ &&
        dettype_list_.find(rect.category_name) == dettype_list_.end()) {
      RCLCPP_INFO(rclcpp::get_logger("fcos_example"), "no draw %s",
                  rect.category_name.c_str());
      continue;
    }
    if (rect.conf <= score_threshold_) {
      RCLCPP_INFO(rclcpp::get_logger("fcos_example"), "box score is: %f",
                  rect.conf);
      continue;
    }

    // 预处理是图像padding到width*width后,再缩放到512x512
    // 将坐标从模型输入大小(512x512)，映射到(width*width)
    double xmin = rect.x1 * factor;
    double xmax = rect.x2 * factor;
    double ymin = rect.y1 * factor;
    double ymax = rect.y2 * factor;

    // 将坐标压缩在图像范围内(width*height)
    ymin = std::min(ymin, static_cast<double>(image_height) - 1);
    ymax = std::min(ymax, static_cast<double>(image_height) - 1);
    if (xmin > xmax || ymin > ymax) {
      continue;
    }

    xmin = std::max(xmin, 0.0);
    xmax = std::min(xmax, image_width - 1.0);
    ymin = std::max(ymin, 0.0);
    ymax = std::min(ymax, image_height - 1.0);
    auto &color = colors[rect.type % 4];
    cv::rectangle(mat, cv::Point(xmin, ymin), cv::Point(xmax, ymax), color, 3);
  }
  return 0;
}
