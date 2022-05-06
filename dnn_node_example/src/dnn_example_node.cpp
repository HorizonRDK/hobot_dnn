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
#include "include/dnn_example_node.h"
#include "include/image_utils.h"
#include "include/fasterrcnn_kps_output_parser.h"
#include "rapidjson/document.h"
#include "rapidjson/istreamwrapper.h"
#include "rapidjson/writer.h"

#ifdef CV_BRIDGE_PKG_ENABLED
#include <cv_bridge/cv_bridge.h>
#endif

DnnExampleNode::DnnExampleNode(
    const std::string & node_name,
    const NodeOptions & options) :
    DnnNode(node_name, options) {
  this->declare_parameter<int>("feed_type", feed_type_);
  this->declare_parameter<std::string>("image", image_);
  this->declare_parameter<int>("image_type", image_type_);
  this->declare_parameter<int>("dump_render_img", dump_render_img_);
  this->declare_parameter<int>("is_sync_mode", is_sync_mode_);
  this->declare_parameter<int>("is_shared_mem_sub", is_shared_mem_sub_);
  this->declare_parameter<std::string>("config_file", config_file);
  this->declare_parameter<std::string>("msg_pub_topic_name",
  msg_pub_topic_name_);

  this->get_parameter<int>("feed_type", feed_type_);
  this->get_parameter<std::string>("image", image_);
  this->get_parameter<int>("image_type", image_type_);
  this->get_parameter<int>("dump_render_img", dump_render_img_);
  this->get_parameter<int>("is_sync_mode", is_sync_mode_);
  this->get_parameter<int>("is_shared_mem_sub", is_shared_mem_sub_);
  this->get_parameter<std::string>("config_file", config_file);
  this->get_parameter<std::string>("msg_pub_topic_name", msg_pub_topic_name_);

  auto ret = DnnParserInit();
  if (0 != ret) {
    RCLCPP_ERROR(rclcpp::get_logger("example"), "dnn parse init failed!!!");
    return;
  }

  std::stringstream ss;
  ss << "Parameter:"
  << "\n feed_type(0:local, 1:sub): " << feed_type_
  << "\n image: " << image_
  << "\n image_type: " << image_type_
  << "\n dump_render_img: " << dump_render_img_
  << "\n is_sync_mode_: " << is_sync_mode_
  << "\n is_shared_mem_sub: " << is_shared_mem_sub_
  << "\n model_file_name: " << model_file_name_
  << "\n model_name: " << model_name_;
  RCLCPP_WARN(rclcpp::get_logger("example"), "%s", ss.str().c_str());

  msg_publisher_ =
      this->create_publisher<ai_msgs::msg::PerceptionTargets>(
        msg_pub_topic_name_, 10);

  if (Init() != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("example"),
      "Init failed!");
  }

  if (GetModelInputSize(0, model_input_width_, model_input_height_) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("example"),
      "Get model input size fail!");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("example"),
    "The model input width is %d and height is %d",
    model_input_width_, model_input_height_);
  }

  if (static_cast<int>(DnnFeedType::FROM_LOCAL) == feed_type_) {
    RCLCPP_INFO(rclcpp::get_logger("example"),
      "Dnn node feed with local image: %s", image_.c_str());
    FeedFromLocal();
  } else if (static_cast<int>(DnnFeedType::FROM_SUB) == feed_type_) {
    RCLCPP_INFO(rclcpp::get_logger("example"),
      "Dnn node feed with subscription");

    if (is_shared_mem_sub_) {
  #ifdef SHARED_MEM_ENABLED
      RCLCPP_WARN(rclcpp::get_logger("example"),
        "Create hbmem_subscription with topic_name: %s",
        sharedmem_img_topic_name_.c_str());
      sharedmem_img_subscription_ =
          this->create_subscription_hbmem<hbm_img_msgs::msg::HbmMsg1080P>(
              sharedmem_img_topic_name_, 10,
              std::bind(&DnnExampleNode::SharedMemImgProcess, this,
                        std::placeholders::_1));
  #else
        RCLCPP_ERROR(rclcpp::get_logger("example"),
          "Unsupport shared mem");
  #endif
    } else {
      RCLCPP_WARN(rclcpp::get_logger("example"),
        "Create subscription with topic_name: %s", ros_img_topic_name_.c_str());
      ros_img_subscription_ =
      this->create_subscription<sensor_msgs::msg::Image>(
          ros_img_topic_name_, 10,
          std::bind(&DnnExampleNode::RosImgProcess, this,
                    std::placeholders::_1));
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("example"),
      "Invalid feed_type:%d", feed_type_);
  }
}

DnnExampleNode::~DnnExampleNode() {
}

int DnnExampleNode::SetNodePara() {
  RCLCPP_INFO(rclcpp::get_logger("example"), "Set node para.");
  if (!dnn_node_para_ptr_) {
    return -1;
  }
  dnn_node_para_ptr_->model_file = model_file_name_;
  dnn_node_para_ptr_->model_name = model_name_;
  dnn_node_para_ptr_->model_task_type = model_task_type_;
  dnn_node_para_ptr_->task_num = 2;
  return 0;
}

int DnnExampleNode::SetOutputParser() {
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

  if (parser == dnnParsers::YOLOV2_PARSER)
  {
    SetYolov2OutParser(model_manage);

  } else if (parser == dnnParsers::YOLOV3_PARSER) {
    SetYolov3OutParser(model_manage);

  } else if (parser == dnnParsers::YOLOV5_PARSER) {
    SetYolov5OutParser(model_manage);

  } else if (parser == dnnParsers::KPS_PARSER) {
    SetFasterRcnnOutParser(model_manage);

  } else {
    return -1;
  }

  return 0;
}

int DnnExampleNode::PostProcess(
  const std::shared_ptr<DnnNodeOutput> &node_output)
{
  if (!msg_publisher_)
  {
    RCLCPP_ERROR(rclcpp::get_logger("example"),
    "Invalid msg_publisher_");
    return -1;
  }
  struct timespec time_start = {0, 0};
  clock_gettime(CLOCK_REALTIME, &time_start);
  ai_msgs::msg::Perf perf;
  perf.set__type("PostProcess");
  perf.stamp_start.sec = time_start.tv_sec;
  perf.stamp_start.nanosec = time_start.tv_nsec;

  auto parser_output =
    std::dynamic_pointer_cast<DnnExampleOutput>(node_output);
  if (parser_output) {
    std::stringstream ss;
    ss << "Output from image_name: " << parser_output->image_name;
    if (parser_output->image_msg_header) {
      ss << ", frame_id: " << parser_output->image_msg_header->frame_id
         << ", stamp: " << parser_output->image_msg_header->stamp.sec
         << "." << parser_output->image_msg_header->stamp.nanosec;
    }
    RCLCPP_INFO(rclcpp::get_logger("example"), "%s", ss.str().c_str());
  }

  const auto& outputs = node_output->outputs;
  RCLCPP_INFO(rclcpp::get_logger("example"),
    "outputs size: %d", outputs.size());
  if (outputs.empty() ||
    static_cast<int32_t>(outputs.size()) < model_output_count_) {
    RCLCPP_ERROR(rclcpp::get_logger("example"), "Invalid outputs");
    return -1;
  }

  int smart_fps = 0;
  {
    auto tp_now = std::chrono::system_clock::now();
    std::unique_lock<std::mutex> lk(frame_stat_mtx_);
    output_frameCount_++;
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                        tp_now - output_tp_).count();
    if (interval >= 1000) {
      RCLCPP_WARN(rclcpp::get_logger("example"),
        "Smart fps = %d", output_frameCount_);
      smart_fps = output_frameCount_;
      output_frameCount_ = 0;
      output_tp_ = std::chrono::system_clock::now();
    }
  }

  ai_msgs::msg::PerceptionTargets::UniquePtr pub_data(
      new ai_msgs::msg::PerceptionTargets());
  if (parser_output->image_msg_header)
  {
    pub_data->header.set__stamp(parser_output->image_msg_header->stamp);
    pub_data->header.set__frame_id(
      parser_output->image_msg_header->frame_id);
  }
  pub_data->set__fps(smart_fps);

  // box
  ai_msgs::msg::Target target;
  if (parser == dnnParsers::KPS_PARSER)
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

        ai_msgs::msg::Roi roi;
        roi.rect.set__x_offset(rect.left);
        roi.rect.set__y_offset(rect.top);
        roi.rect.set__width(rect.right - rect.left);
        roi.rect.set__height(rect.bottom - rect.top);

        target.set__type("person");
        target.rois.emplace_back(roi);
      }
    }
  } else {
    auto *det_result =
      dynamic_cast<Dnn_Parser_Result *>(outputs[output_index_].get());
    if (!det_result)
    {
      RCLCPP_INFO(rclcpp::get_logger("example"), "invalid cast");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("example"), "out box size: %d",
                det_result->perception.det.size());
    for (auto &rect : det_result->perception.det)
    {
      if (rect.bbox.xmin < 0) rect.bbox.xmin = 0;
      if (rect.bbox.ymin < 0) rect.bbox.ymin = 0;
      if (rect.bbox.xmax > model_input_width_)
      {
        rect.bbox.xmax = model_input_width_;
      }
      if (rect.bbox.ymax > model_input_height_)
      {
        rect.bbox.ymax = model_input_height_;
      }
      std::stringstream ss;
      ss << "det rect: " << rect.bbox.xmin << " "
         << rect.bbox.ymin << " " << rect.bbox.xmax << " "
         << rect.bbox.ymax << ", det type: " << rect.class_name
         << ", score:" << rect.score;
      RCLCPP_INFO(rclcpp::get_logger("example"), "%s", ss.str().c_str());

      ai_msgs::msg::Roi roi;
      roi.rect.set__x_offset(rect.bbox.xmin);
      roi.rect.set__y_offset(rect.bbox.ymin);
      roi.rect.set__width(rect.bbox.xmax - rect.bbox.xmin);
      roi.rect.set__height(rect.bbox.ymax - rect.bbox.ymin);

      target.set__type(rect.class_name);
      target.rois.emplace_back(roi);
    }
  }
  if (parser == dnnParsers::KPS_PARSER)
  {
    auto *lmk_result =
      dynamic_cast<LandmarksResult *>(outputs[kps_output_index_].get());
    if (lmk_result) {
      RCLCPP_INFO(rclcpp::get_logger("example"), "out kps size: %d",
        lmk_result->values.size());
      std::stringstream ss;
      for (const auto& value : lmk_result->values) {
        ai_msgs::msg::Point target_point;
        target_point.set__type("body_kps");
        ss << "kps point: ";
        for (const auto &point : value) {
          ss << "\n" << point.x << "," << point.y << "," << point.score;
          geometry_msgs::msg::Point32 pt;
          pt.set__x(point.x);
          pt.set__y(point.y);
          target_point.point.emplace_back(pt);
        }
        ss << "\n";
        RCLCPP_DEBUG(rclcpp::get_logger("example"),
          "FasterRcnnKpsOutputParser parse kps: %s", ss.str().c_str());
        target.points.emplace_back(target_point);
      }
    }
  }

  pub_data->targets.emplace_back(std::move(target));

  clock_gettime(CLOCK_REALTIME, &time_start);
  perf.stamp_end.sec = time_start.tv_sec;
  perf.stamp_end.nanosec = time_start.tv_nsec;
  pub_data->perfs.emplace_back(perf);
  msg_publisher_->publish(std::move(pub_data));

  return 0;
}

int DnnExampleNode::Predict(
  std::vector<std::shared_ptr<DNNInput>> &inputs,
  const std::shared_ptr<std::vector<hbDNNRoi>> rois,
  std::shared_ptr<DnnNodeOutput> dnn_output) {
  RCLCPP_INFO(rclcpp::get_logger("example"), "task_num: %d",
  dnn_node_para_ptr_->task_num);

  return Run(inputs, dnn_output, rois, is_sync_mode_ == 1 ? true : false);
}

int DnnExampleNode::FeedFromLocal() {
  if (access(image_.c_str(), R_OK) == -1) {
    RCLCPP_ERROR(rclcpp::get_logger("example"),
      "Image: %s not exist!", image_.c_str());
    return -1;
  }

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
  auto dnn_output = std::make_shared<DnnExampleOutput>();
  dnn_output->image_name = image_;
  uint32_t ret = 0;
  // 3. 开始预测
  ret = Predict(inputs, nullptr, dnn_output);
  // 4. 处理预测结果，如渲染到图片或者发布预测结果
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("example"), "Run predict failed!");
    return ret;
  } else if (dump_render_img_ &&
    static_cast<int>(ImageType::BGR) == image_type_) {
    // 只支持对jpg/png等格式图片做渲染
    std::string result_image = "render.jpg";
    cv::Mat mat = cv::imread(image_, cv::IMREAD_COLOR);
    if (parser == dnnParsers::KPS_PARSER) {
      Render(mat,
      dynamic_cast<Filter2DResult *>(
        dnn_output->outputs[box_output_index_].get()),
      dynamic_cast<LandmarksResult *>(
        dnn_output->outputs[kps_output_index_].get()),
      model_input_height_, model_input_width_);
    } else {
      Render(mat, dynamic_cast<Dnn_Parser_Result *>(
                    dnn_output->outputs[output_index_].get())->perception);
    }

    RCLCPP_INFO(rclcpp::get_logger("example"),
      "Draw result to file: %s", result_image.c_str());
    cv::imwrite(result_image, mat);
  }
  return 0;
}

void DnnExampleNode::RosImgProcess(
  const sensor_msgs::msg::Image::ConstSharedPtr img_msg) {
  if (!img_msg) {
    RCLCPP_DEBUG(rclcpp::get_logger("example"), "Get img failed");
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
                        tp_now - sub_img_tp_).count();
    if (interval >= 1000) {
      RCLCPP_WARN(rclcpp::get_logger("img_sub"),
      "Sub img fps = %d", sub_img_frameCount_);
      sub_img_frameCount_ = 0;
      sub_img_tp_ = std::chrono::system_clock::now();
    }
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

  // dump recved img msg
  // std::ofstream ofs("img." + img_msg->encoding);
  // ofs.write(reinterpret_cast<const char*>(img_msg->data.data()),
  //   img_msg->data.size());

  auto tp_start = std::chrono::system_clock::now();

  // 1. 将图片处理成模型输入数据类型DNNInput
  // 使用图片生成pym，NV12PyramidInput为DNNInput的子类
  std::shared_ptr<hobot::easy_dnn::NV12PyramidInput> pyramid = nullptr;
  if ("rgb8" == img_msg->encoding) {
#ifdef CV_BRIDGE_PKG_ENABLED
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

    pyramid = ImageUtils::GetNV12Pyramid(cv_img->image,
      model_input_height_, model_input_width_);
#else
    RCLCPP_ERROR(rclcpp::get_logger("example"), "Unsupport cv bridge");
#endif
  } else if ("bgr8" == img_msg->encoding) {
#ifdef CV_BRIDGE_PKG_ENABLED
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

    pyramid = ImageUtils::GetNV12Pyramid(cv_img->image,
      model_input_height_, model_input_width_);
#else
    RCLCPP_ERROR(rclcpp::get_logger("example"), "Unsupport cv bridge");
#endif
  } else if ("nv12" == img_msg->encoding) {
    pyramid = hobot::dnn_node::ImageProc::GetNV12PyramidFromNV12Img(
      reinterpret_cast<const char*>(img_msg->data.data()),
      img_msg->height, img_msg->width,
      model_input_height_, model_input_width_);
  }

  if (!pyramid) {
    RCLCPP_ERROR(rclcpp::get_logger("example"),
      "Get Nv12 pym fail");
    return;
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
  auto dnn_output = std::make_shared<DnnExampleOutput>();
  dnn_output->image_msg_header = std::make_shared<std_msgs::msg::Header>();
  dnn_output->image_msg_header->set__frame_id(img_msg->header.frame_id);
  dnn_output->image_msg_header->set__stamp(img_msg->header.stamp);
  uint32_t ret = 0;
  // 3. 开始预测
  ret = Predict(inputs, nullptr, dnn_output);

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
    return;
  } else if (dump_render_img_ && is_sync_mode_) {
    std::string result_image = "render_" +
      std::to_string(img_msg->header.stamp.sec) + "." +
      std::to_string(img_msg->header.stamp.nanosec) + ".jpg";

    if ("rgb8" == img_msg->encoding) {
#ifdef CV_BRIDGE_PKG_ENABLED
      auto cv_img = cv_bridge::cvtColorForDisplay(
        cv_bridge::toCvShare(img_msg),
        "bgr8");
      auto mat = cv_img->image;
      if (parser == dnnParsers::KPS_PARSER) {
        Render(mat,
              dynamic_cast<Filter2DResult *>(
                dnn_output->outputs[box_output_index_].get()),
              dynamic_cast<LandmarksResult *>(
                dnn_output->outputs[kps_output_index_].get()),
              model_input_height_, model_input_width_);
      } else {
        Render(mat, dynamic_cast<Dnn_Parser_Result *>(
                      dnn_output->outputs[output_index_].get())->perception);
      }
      RCLCPP_INFO(rclcpp::get_logger("example"),
        "Draw result to file: %s", result_image.c_str());
      cv::imwrite(result_image, mat);
#endif
    } else if ("bgr8" == img_msg->encoding) {
#ifdef CV_BRIDGE_PKG_ENABLED
      auto cv_img = cv_bridge::cvtColorForDisplay(
        cv_bridge::toCvShare(img_msg),
        "bgr8");
      auto mat = cv_img->image;
      if (parser == dnnParsers::KPS_PARSER) {
        Render(mat,
              dynamic_cast<Filter2DResult *>(
                dnn_output->outputs[box_output_index_].get()),
              dynamic_cast<LandmarksResult *>(
                dnn_output->outputs[kps_output_index_].get()),
              model_input_height_, model_input_width_);
      } else {
        Render(mat, dynamic_cast<Dnn_Parser_Result *>(
                      dnn_output->outputs[output_index_].get())->perception);
      }
      RCLCPP_INFO(rclcpp::get_logger("example"),
        "Draw result to file: %s", result_image.c_str());
      cv::imwrite(result_image, mat);
#endif
    } else if ("nv12" == img_msg->encoding) {
        char* y_img = reinterpret_cast<char*>(pyramid->y_vir_addr);
        char* uv_img = reinterpret_cast<char*>(pyramid->uv_vir_addr);
        auto height = pyramid->height;
        auto width = pyramid->width;
        auto img_y_size = height * width;
        auto img_uv_size = img_y_size / 2;
        char* buf = new char[img_y_size + img_uv_size];
        memcpy(buf, y_img, img_y_size);
        memcpy(buf + img_y_size, uv_img, img_uv_size);
        cv::Mat nv12(height *3 / 2, width, CV_8UC1, buf);
        cv::Mat bgr;
        cv::cvtColor(nv12, bgr, CV_YUV2BGR_NV12);
        delete []buf;
        auto& mat = bgr;
        if (parser == dnnParsers::KPS_PARSER) {
          Render(mat,
                dynamic_cast<Filter2DResult *>(
                  dnn_output->outputs[box_output_index_].get()),
                dynamic_cast<LandmarksResult *>(
                  dnn_output->outputs[kps_output_index_].get()),
                model_input_height_, model_input_width_);
        } else {
          Render(mat, dynamic_cast<Dnn_Parser_Result *>(
                        dnn_output->outputs[output_index_].get())->perception);
        }
        RCLCPP_INFO(rclcpp::get_logger("example"),
          "Draw result to file: %s", result_image.c_str());
        cv::imwrite(result_image, mat);
    }
  }
}

#ifdef SHARED_MEM_ENABLED
void DnnExampleNode::SharedMemImgProcess(
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
                        tp_now - sub_img_tp_).count();
    if (interval >= 1000) {
      RCLCPP_WARN(rclcpp::get_logger("img_sub"),
      "Sub img fps = %d", sub_img_frameCount_);
      sub_img_frameCount_ = 0;
      sub_img_tp_ = std::chrono::system_clock::now();
    }
  }

  // dump recved img msg
  // std::ofstream ofs("img_" + std::to_string(img_msg->index) + "." +
  // std::string(reinterpret_cast<const char*>(img_msg->encoding.data())));
  // ofs.write(reinterpret_cast<const char*>(img_msg->data.data()),
  //   img_msg->data_size);

  auto tp_start = std::chrono::system_clock::now();

  // 1. 将图片处理成模型输入数据类型DNNInput
  // 使用图片生成pym，NV12PyramidInput为DNNInput的子类
  std::shared_ptr<hobot::dnn_node::NV12PyramidInput> pyramid = nullptr;
  if ("nv12" ==
  std::string(reinterpret_cast<const char*>(img_msg->encoding.data()))) {
    pyramid = hobot::dnn_node::ImageProc::GetNV12PyramidFromNV12Img(
      reinterpret_cast<const char*>(img_msg->data.data()),
      img_msg->height, img_msg->width,
      model_input_height_, model_input_width_);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("example"),
    "Unsupported img encoding: %s", img_msg->encoding.data());
    RCLCPP_ERROR(rclcpp::get_logger("example"),
    "Only nv12 img encoding is supported for shared mem test");
    return;
  }

  if (!pyramid) {
    RCLCPP_ERROR(rclcpp::get_logger("example"),
      "Get Nv12 pym fail");
    return;
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
  auto dnn_output = std::make_shared<DnnExampleOutput>();
  dnn_output->image_msg_header = std::make_shared<std_msgs::msg::Header>();
  dnn_output->image_msg_header->set__frame_id(std::to_string(img_msg->index));
  // dnn_output->image_msg_header->set__stamp(img_msg->time_stamp);
  uint32_t ret = 0;
  // 3. 开始预测
  ret = Predict(inputs, nullptr, dnn_output);

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
    return;
  } else if (dump_render_img_ && is_sync_mode_) {
    std::string result_image = "render_" +
        std::to_string(img_msg->index) + ".jpg";
    if ("nv12" ==
    std::string(reinterpret_cast<const char*>(img_msg->encoding.data()))) {
        char* y_img = reinterpret_cast<char*>(pyramid->y_vir_addr);
        char* uv_img = reinterpret_cast<char*>(pyramid->uv_vir_addr);
        auto height = pyramid->height;
        auto width = pyramid->width;
        auto img_y_size = height * width;
        auto img_uv_size = img_y_size / 2;
        char* buf = new char[img_y_size + img_uv_size];
        memcpy(buf, y_img, img_y_size);
        memcpy(buf + img_y_size, uv_img, img_uv_size);
        cv::Mat nv12(height *3 / 2, width, CV_8UC1, buf);
        cv::Mat bgr;
        cv::cvtColor(nv12, bgr, CV_YUV2BGR_NV12);
        delete []buf;
        auto& mat = bgr;
        if (parser == dnnParsers::KPS_PARSER) {
          Render(mat,
                dynamic_cast<Filter2DResult *>(
                  dnn_output->outputs[box_output_index_].get()),
                dynamic_cast<LandmarksResult *>(
                  dnn_output->outputs[kps_output_index_].get()),
                model_input_height_, model_input_width_);
        } else {
          Render(mat, dynamic_cast<Dnn_Parser_Result *>(
                        dnn_output->outputs[output_index_].get())->perception);
        }
        RCLCPP_INFO(rclcpp::get_logger("example"),
          "Draw result to file: %s", result_image.c_str());
        cv::imwrite(result_image, mat);
    }
  }
}
#endif

int DnnExampleNode::Render(
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

int DnnExampleNode::DnnParserInit()
{
  if (config_file.empty())
  {
    return 0;
  }
  // Parsing config
  std::ifstream ifs(config_file.c_str());
  rapidjson::IStreamWrapper isw(ifs);
  rapidjson::Document document;
  document.ParseStream(isw);
  if (document.HasParseError())
  {
    RCLCPP_ERROR(rclcpp::get_logger("example"), "Parsing config file failed");
    return -1;
  }

  if (document.HasMember("model_file"))
  {
    model_file_name_ = document["model_file"].GetString();
  }
  if (document.HasMember("model_name"))
  {
    model_name_ = document["model_name"].GetString();
  }
  if (document.HasMember("dnnParser"))
  {
    std::string str_parser = document["dnnParser"].GetString();
    if ("yolov2" == str_parser) {
      parser = dnnParsers::YOLOV2_PARSER;
    } else if ("yolov3" == str_parser) {
      parser = dnnParsers::YOLOV3_PARSER;
    } else if ("yolov5" == str_parser) {
      parser = dnnParsers::YOLOV5_PARSER;
    } else if ("kps_parser" == str_parser) {
      parser = dnnParsers::KPS_PARSER;
    } else {
      std::stringstream ss;
      ss << "Error!Invalid parser: "
        << str_parser
        << " . Only yolov2, yolov3, yolov5, kps_parser are supported";
      RCLCPP_ERROR(rclcpp::get_logger("example"),
        "%s", ss.str().c_str());
      return -3;
    }
  }

  model_output_count_ = document["model_output_count"].GetInt();
  return 0;
}

void DnnExampleNode::SetYolov2OutParser(Model* model_manage)
{
  output_index_ = model_output_count_ - 1;
  std::shared_ptr<OutputParser> yolov5_out_parser =
      std::make_shared<hobot::dnn_node::Yolo2OutputParser>();
  model_manage->SetOutputParser(output_index_, yolov5_out_parser);
}

void DnnExampleNode::SetYolov3OutParser(Model* model_manage)
{
  output_index_ = model_output_count_ - 1;
  for (int i = 0; i < output_index_; ++i)
  {
    std::shared_ptr<OutputParser> assist_parser =
        std::make_shared<hobot::dnn_node::Yolo3_darknetAssistParser>();
    model_manage->SetOutputParser(i, assist_parser);
  }

  // set yolov5 paser
  auto output_desc = std::make_shared<OutputDescription>(
      model_manage, output_index_, "yolov3_branch");
  for (int i = 0; i < output_index_; ++i)
  {
    output_desc->GetDependencies().push_back(i);
  }
  output_desc->SetType("yolov3");
  model_manage->SetOutputDescription(output_desc);
  std::shared_ptr<OutputParser> yolov3_out_parser =
      std::make_shared<hobot::dnn_node::Yolo3DarknetOutputParser>();
  model_manage->SetOutputParser(output_index_, yolov3_out_parser);
}

void DnnExampleNode::SetYolov5OutParser(Model* model_manage)
{
  output_index_ = model_output_count_ - 1;
  for (int i = 0; i < output_index_; ++i)
  {
    std::shared_ptr<OutputParser> assist_parser =
        std::make_shared<hobot::dnn_node::Yolo5AssistParser>();
    model_manage->SetOutputParser(i, assist_parser);
  }

  // set yolov5 paser
  auto output_desc = std::make_shared<OutputDescription>(
      model_manage, output_index_, "yolov5_branch");
  for (int i = 0; i < output_index_; ++i)
  {
    output_desc->GetDependencies().push_back(i);
  }
  output_desc->SetType("yolov5");
  model_manage->SetOutputDescription(output_desc);
  std::shared_ptr<OutputParser> yolov5_out_parser =
      std::make_shared<hobot::dnn_node::Yolo5OutputParser>();
  model_manage->SetOutputParser(output_index_, yolov5_out_parser);
}

void DnnExampleNode::SetFasterRcnnOutParser(Model* model_manage)
{
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
}

int DnnExampleNode::Render(cv::Mat &mat, Perception &result)
{
    // render box
  int image_width = mat.cols;
  int image_height = mat.rows;
  float factor = image_width * 1.0 / model_input_width_;
  for (auto &rect : result.det)
  {
    // 预处理是图像padding到width*width后,再缩放到模型输入size
    // 将坐标从模型输入大小，映射到(width*width)
    double xmin = rect.bbox.xmin * factor;
    double xmax = rect.bbox.xmax * factor;
    double ymin = rect.bbox.ymin * factor;
    double ymax = rect.bbox.ymax * factor;

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
    auto &color = colors[rect.id % 4];
    cv::rectangle(mat, cv::Point(xmin, ymin), cv::Point(xmax, ymax), color, 3);
  }
  return 0;
}
