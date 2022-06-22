// Copyright (c) 2022，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "include/dnn_example_node.h"

#include <unistd.h>

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "dnn_node/dnn_node.h"
#include "include/image_utils.h"
#include "include/post_process/post_process_base.h"
#include "include/post_process/post_process_classification.h"
#include "include/post_process/post_process_efficientdet.h"
#include "include/post_process/post_process_fasterrcnn.h"
#include "include/post_process/post_process_fcos.h"
#include "include/post_process/post_process_ssd.h"
#include "include/post_process/post_process_unet.h"
#include "include/post_process/post_process_yolov2.h"
#include "include/post_process/post_process_yolov3.h"
#include "include/post_process/post_process_yolov5.h"
#include "rapidjson/document.h"
#include "rapidjson/istreamwrapper.h"
#include "rapidjson/writer.h"
#include "rclcpp/rclcpp.hpp"

#ifdef CV_BRIDGE_PKG_ENABLED
#include <cv_bridge/cv_bridge.h>
#endif

builtin_interfaces::msg::Time ConvertToRosTime(
    const struct timespec &time_spec) {
  builtin_interfaces::msg::Time stamp;
  stamp.set__sec(time_spec.tv_sec);
  stamp.set__nanosec(time_spec.tv_nsec);
  return stamp;
}

int CalTimeMsDuration(const builtin_interfaces::msg::Time &start,
                      const builtin_interfaces::msg::Time &end) {
  return (end.sec - start.sec) * 1000 + end.nanosec / 1000 / 1000 -
         start.nanosec / 1000 / 1000;
}

DnnExampleNode::DnnExampleNode(const std::string &node_name,
                               const NodeOptions &options)
    : DnnNode(node_name, options) {
  this->declare_parameter<int>("feed_type", feed_type_);
  this->declare_parameter<std::string>("image", image_);
  this->declare_parameter<int>("image_type", image_type_);
  this->declare_parameter<int>("image_width", image_width);
  this->declare_parameter<int>("image_height", image_height);
  this->declare_parameter<int>("dump_render_img", dump_render_img_);
  this->declare_parameter<int>("is_sync_mode", is_sync_mode_);
  this->declare_parameter<int>("is_shared_mem_sub", is_shared_mem_sub_);
  this->declare_parameter<std::string>("config_file", config_file);
  this->declare_parameter<std::string>("msg_pub_topic_name",
                                       msg_pub_topic_name_);

  this->get_parameter<int>("feed_type", feed_type_);
  this->get_parameter<std::string>("image", image_);
  this->get_parameter<int>("image_type", image_type_);
  this->get_parameter<int>("image_width", image_width);
  this->get_parameter<int>("image_height", image_height);
  this->get_parameter<int>("dump_render_img", dump_render_img_);
  this->get_parameter<int>("is_sync_mode", is_sync_mode_);
  this->get_parameter<int>("is_shared_mem_sub", is_shared_mem_sub_);
  this->get_parameter<std::string>("config_file", config_file);
  this->get_parameter<std::string>("msg_pub_topic_name", msg_pub_topic_name_);

  auto ret = DnnParserInit();
  if (0 != ret) {
    RCLCPP_ERROR(rclcpp::get_logger("example"),
                 "dnn parse init failed!!! config_file: %s",
                 config_file.data());
    rclcpp::shutdown();
  }

  std::stringstream ss;
  ss << "Parameter:"
     << "\n feed_type(0:local, 1:sub): " << feed_type_ << "\n image: " << image_
     << "\n image_type: " << image_type_
     << "\n dump_render_img: " << dump_render_img_
     << "\n is_sync_mode_: " << is_sync_mode_
     << "\n is_shared_mem_sub: " << is_shared_mem_sub_
     << "\n model_file_name: " << model_file_name_
     << "\n model_name: " << model_name_;
  RCLCPP_WARN(rclcpp::get_logger("example"), "%s", ss.str().c_str());

  msg_publisher_ = this->create_publisher<ai_msgs::msg::PerceptionTargets>(
      msg_pub_topic_name_, 10);
  unet_publisher_ =
      this->create_publisher<sensor_msgs::msg::Image>(unet_pub_topic_name_, 10);

  if (Init() != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("example"), "Init failed!");
    rclcpp::shutdown();
  }

  if (GetModelInputSize(0, model_input_width_, model_input_height_) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("example"), "Get model input size fail!");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("example"),
                "The model input width is %d and height is %d",
                model_input_width_,
                model_input_height_);
    post_process_->SetModelInputWH(model_input_width_, model_input_height_);
  }

  RCLCPP_WARN(rclcpp::get_logger("example"),
              "Create ai msg publisher with topic_name: %s",
              msg_pub_topic_name_.c_str());

  if (static_cast<int>(DnnFeedType::FROM_LOCAL) == feed_type_) {
    RCLCPP_INFO(rclcpp::get_logger("example"),
                "Dnn node feed with local image: %s",
                image_.c_str());
    FeedFromLocal();
  } else if (static_cast<int>(DnnFeedType::FROM_SUB) == feed_type_) {
    RCLCPP_INFO(rclcpp::get_logger("example"),
                "Dnn node feed with subscription");

    if (is_shared_mem_sub_) {
#ifdef SHARED_MEM_ENABLED
      RCLCPP_WARN(rclcpp::get_logger("example"),
                  "Create img hbmem_subscription with topic_name: %s",
                  sharedmem_img_topic_name_.c_str());
      sharedmem_img_subscription_ =
          this->create_subscription_hbmem<hbm_img_msgs::msg::HbmMsg1080P>(
              sharedmem_img_topic_name_,
              10,
              std::bind(&DnnExampleNode::SharedMemImgProcess,
                        this,
                        std::placeholders::_1));
#else
      RCLCPP_ERROR(rclcpp::get_logger("example"), "Unsupport shared mem");
#endif
    } else {
      RCLCPP_WARN(rclcpp::get_logger("example"),
                  "Create img subscription with topic_name: %s",
                  ros_img_topic_name_.c_str());
      ros_img_subscription_ =
          this->create_subscription<sensor_msgs::msg::Image>(
              ros_img_topic_name_,
              10,
              std::bind(
                  &DnnExampleNode::RosImgProcess, this, std::placeholders::_1));
    }
  } else {
    RCLCPP_ERROR(
        rclcpp::get_logger("example"), "Invalid feed_type:%d", feed_type_);
  }
}

DnnExampleNode::~DnnExampleNode() {}

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

  // 创建PostProcess实例
  if (parser == dnnParsers::YOLOV2_PARSER) {
    post_process_ = std::dynamic_pointer_cast<PostProcessBase>(
        std::make_shared<Yolov2PostProcess>(model_output_count_));
  } else if (parser == dnnParsers::YOLOV3_PARSER) {
    post_process_ = std::dynamic_pointer_cast<PostProcessBase>(
        std::make_shared<Yolov3PostProcess>(model_output_count_));
  } else if (parser == dnnParsers::YOLOV5_PARSER) {
    post_process_ = std::dynamic_pointer_cast<PostProcessBase>(
        std::make_shared<Yolov5PostProcess>(model_output_count_));
  } else if (parser == dnnParsers::FASTERRCNN_PARSER) {
    post_process_ = std::dynamic_pointer_cast<PostProcessBase>(
        std::make_shared<FasterRcnnPostProcess>(model_output_count_));
  } else if (parser == dnnParsers::CLASSIFICATION_PARSER) {
    post_process_ = std::dynamic_pointer_cast<PostProcessBase>(
        std::make_shared<ClassificationPostProcess>(model_output_count_,
                                                    cls_name_file));
  } else if (parser == dnnParsers::EFFICIENTDET_PARSER) {
    post_process_ = std::dynamic_pointer_cast<PostProcessBase>(
        std::make_shared<EfficientDetPostProcess>(model_output_count_,
                                                  dequanti_file));
  } else if (parser == dnnParsers::SSD_PARSER) {
    post_process_ = std::dynamic_pointer_cast<PostProcessBase>(
        std::make_shared<SsdPostProcess>(model_output_count_));
  } else if (parser == dnnParsers::FCOS_PARSER) {
    post_process_ = std::dynamic_pointer_cast<PostProcessBase>(
        std::make_shared<FcosPostProcess>(model_output_count_));
  } else if (parser == dnnParsers::UNET_PARSER) {
    post_process_ = std::dynamic_pointer_cast<PostProcessBase>(
        std::make_shared<UnetPostProcess>(model_output_count_,
                                          dump_render_img_));
  } else {
    return -1;
  }

  if (!post_process_) {
    RCLCPP_ERROR(rclcpp::get_logger("example"), "Invalid post_process_!");
    return -1;
  }

  if (post_process_->SetOutParser(model_manage) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("example"), "Set output parser failed!");
    return -1;
  }

  return 0;
}

int DnnExampleNode::PostProcess(
    const std::shared_ptr<DnnNodeOutput> &node_output) {
  if (!rclcpp::ok()) {
    return -1;
  }

  struct timespec time_start = {0, 0};
  clock_gettime(CLOCK_REALTIME, &time_start);

  auto parser_output = std::dynamic_pointer_cast<DnnExampleOutput>(node_output);
  if (parser_output) {
    std::stringstream ss;
    ss << "Output from image_name: " << parser_output->image_name;
    ss << ", frame_id: " << parser_output->frame_id
       << ", stamp: " << parser_output->stamp.sec << "."
       << parser_output->stamp.nanosec;
    RCLCPP_INFO(rclcpp::get_logger("example"), "%s", ss.str().c_str());
  }

  if (!msg_publisher_ || !post_process_) {
    RCLCPP_ERROR(rclcpp::get_logger("example"),
                 "Invalid msg_publisher_/post_process_");
    return -1;
  }

  auto pub_data = std::move(post_process_->PostProcess(node_output));

  if (!pub_data) {
    RCLCPP_ERROR(rclcpp::get_logger("example"), "Invalid pub_data");
    return -1;
  }

  pub_data->header.set__stamp(parser_output->stamp);
  pub_data->header.set__frame_id(parser_output->frame_id);

  if (dump_render_img_ && parser_output->pyramid) {
    ImageUtils::Render(parser_output->pyramid, pub_data);
  }

  // preprocess
  ai_msgs::msg::Perf perf_preprocess;
  perf_preprocess.set__type(model_name_ + "_preprocess");
  perf_preprocess.set__stamp_start(
      ConvertToRosTime(parser_output->preprocess_timespec_start));
  perf_preprocess.set__stamp_end(
      ConvertToRosTime(parser_output->preprocess_timespec_end));
  perf_preprocess.set__time_ms_duration(CalTimeMsDuration(
      perf_preprocess.stamp_start, perf_preprocess.stamp_end));
  pub_data->perfs.emplace_back(perf_preprocess);

  if (node_output->rt_stat) {
    if (node_output->rt_stat->fps_updated) {
      RCLCPP_WARN(rclcpp::get_logger("example"),
                  "Sub img fps %.2f",
                  node_output->rt_stat->input_fps);
      RCLCPP_WARN(rclcpp::get_logger("example"),
                  "Smart fps %.2f",
                  node_output->rt_stat->output_fps);
    }

    struct timespec time_now = {0, 0};
    clock_gettime(CLOCK_REALTIME, &time_now);

    // predict
    ai_msgs::msg::Perf perf;
    perf.set__type(model_name_ + "_predict_infer");
    perf.stamp_start =
        ConvertToRosTime(node_output->rt_stat->infer_timespec_start);
    perf.stamp_end = ConvertToRosTime(node_output->rt_stat->infer_timespec_end);
    perf.set__time_ms_duration(node_output->rt_stat->infer_time_ms);
    pub_data->perfs.push_back(perf);

    perf.set__type(model_name_ + "_predict_parse");
    perf.stamp_start =
        ConvertToRosTime(node_output->rt_stat->parse_timespec_start);
    perf.stamp_end = ConvertToRosTime(node_output->rt_stat->parse_timespec_end);
    perf.set__time_ms_duration(node_output->rt_stat->parse_time_ms);
    pub_data->perfs.push_back(perf);

    // postprocess
    ai_msgs::msg::Perf perf_postprocess;
    perf_postprocess.set__type(model_name_ + "_postprocess");
    perf_postprocess.stamp_start = ConvertToRosTime(time_start);
    clock_gettime(CLOCK_REALTIME, &time_now);
    perf_postprocess.stamp_end = ConvertToRosTime(time_now);
    perf_postprocess.set__time_ms_duration(CalTimeMsDuration(
        perf_postprocess.stamp_start, perf_postprocess.stamp_end));
    pub_data->perfs.emplace_back(perf_postprocess);

    // 从发布图像到发布AI结果的延迟
    ai_msgs::msg::Perf perf_pipeline;
    perf_pipeline.set__type(model_name_ + "_pipeline");
    perf_pipeline.set__stamp_start(pub_data->header.stamp);
    perf_pipeline.set__stamp_end(perf_postprocess.stamp_end);
    perf_pipeline.set__time_ms_duration(
        CalTimeMsDuration(perf_pipeline.stamp_start, perf_pipeline.stamp_end));
    pub_data->perfs.push_back(perf_pipeline);

    pub_data->set__fps(round(node_output->rt_stat->input_fps));
  }

  msg_publisher_->publish(std::move(pub_data));
  return 0;
}

int DnnExampleNode::Predict(std::vector<std::shared_ptr<DNNInput>> &inputs,
                            const std::shared_ptr<std::vector<hbDNNRoi>> rois,
                            std::shared_ptr<DnnNodeOutput> dnn_output) {
  RCLCPP_INFO(rclcpp::get_logger("example"),
              "task_num: %d",
              dnn_node_para_ptr_->task_num);

  return Run(inputs, dnn_output, rois, is_sync_mode_ == 1 ? true : false);
}

int DnnExampleNode::FeedFromLocal() {
  if (access(image_.c_str(), R_OK) == -1) {
    RCLCPP_ERROR(
        rclcpp::get_logger("example"), "Image: %s not exist!", image_.c_str());
    return -1;
  }

  // 1. 将图片处理成模型输入数据类型DNNInput
  // 使用图片生成pym，NV12PyramidInput为DNNInput的子类
  std::shared_ptr<hobot::easy_dnn::NV12PyramidInput> pyramid = nullptr;
  if (static_cast<int>(ImageType::BGR) == image_type_) {
    // bgr img，支持将图片resize到模型输入size
    pyramid = ImageUtils::GetNV12Pyramid(
        image_, ImageType::BGR, model_input_height_, model_input_width_);
    if (!pyramid) {
      RCLCPP_ERROR(rclcpp::get_logger("example"),
                   "Get Nv12 pym fail with image: %s",
                   image_.c_str());
      return -1;
    }
  } else if (static_cast<int>(ImageType::NV12) == image_type_) {
    std::ifstream ifs(image_, std::ios::in | std::ios::binary);
    if (!ifs) {
      return -1;
    }
    ifs.seekg(0, std::ios::end);
    int len = ifs.tellg();
    ifs.seekg(0, std::ios::beg);
    char *data = new char[len];
    ifs.read(data, len);
    pyramid = hobot::dnn_node::ImageProc::GetNV12PyramidFromNV12Img(
        data,
        image_height,
        image_width,
        model_input_height_,
        model_input_width_);
    if (!pyramid) {
      RCLCPP_ERROR(rclcpp::get_logger("example"),
                   "Get Nv12 pym fail with image: %s",
                   image_.c_str());
      return -1;
    }

  } else {
    RCLCPP_ERROR(
        rclcpp::get_logger("example"), "Invalid image type: %d", image_type_);
    return -1;
  }

  // 2. 使用pyramid创建DNNInput对象inputs
  // inputs将会作为模型的输入通过RunInferTask接口传入
  auto inputs = std::vector<std::shared_ptr<DNNInput>>{pyramid};
  auto dnn_output = std::make_shared<DnnExampleOutput>();
  dnn_output->image_name = image_;
  dnn_output->frame_id = "feedback";
  if (dump_render_img_) {
    dnn_output->pyramid = pyramid;
  }
  uint32_t ret = 0;
  // 3. 开始预测
  ret = Predict(inputs, nullptr, dnn_output);
  // 4. 处理预测结果，如渲染到图片或者发布预测结果
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("example"), "Run predict failed!");
    return ret;
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

  std::stringstream ss;
  ss << "Recved img encoding: " << img_msg->encoding
     << ", h: " << img_msg->height << ", w: " << img_msg->width
     << ", step: " << img_msg->step
     << ", frame_id: " << img_msg->header.frame_id
     << ", stamp: " << img_msg->header.stamp.sec << "_"
     << img_msg->header.stamp.nanosec
     << ", data size: " << img_msg->data.size();
  RCLCPP_INFO(rclcpp::get_logger("example"), "%s", ss.str().c_str());

  // dump recved img msg
  // std::ofstream ofs("img_" + img_msg->header.frame_id +
  //    std::to_string(img_msg->header.stamp.sec) + "_" +
  //    std::to_string(img_msg->header.stamp.nanosec) + "." +
  //    img_msg->encoding);
  // ofs.write(reinterpret_cast<const char*>(img_msg->data.data()),
  //   img_msg->data.size());

  auto tp_start = std::chrono::system_clock::now();

  // 1. 将图片处理成模型输入数据类型DNNInput
  // 使用图片生成pym，NV12PyramidInput为DNNInput的子类
  std::shared_ptr<hobot::easy_dnn::NV12PyramidInput> pyramid = nullptr;
  if ("rgb8" == img_msg->encoding) {
#ifdef CV_BRIDGE_PKG_ENABLED
    auto cv_img =
        cv_bridge::cvtColorForDisplay(cv_bridge::toCvShare(img_msg), "bgr8");
    // dump recved img msg after convert
    // cv::imwrite("dump_raw_" +
    //     std::to_string(img_msg->header.stamp.sec) + "." +
    //     std::to_string(img_msg->header.stamp.nanosec) + ".jpg",
    //     cv_img->image);

    {
      auto tp_now = std::chrono::system_clock::now();
      auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                          tp_now - tp_start)
                          .count();
      RCLCPP_DEBUG(rclcpp::get_logger("example"),
                   "after cvtColorForDisplay cost ms: %d",
                   interval);
    }

    pyramid = ImageUtils::GetNV12Pyramid(
        cv_img->image, model_input_height_, model_input_width_);
#else
    RCLCPP_ERROR(rclcpp::get_logger("example"), "Unsupport cv bridge");
#endif
  } else if ("bgr8" == img_msg->encoding) {
#ifdef CV_BRIDGE_PKG_ENABLED
    auto cv_img =
        cv_bridge::cvtColorForDisplay(cv_bridge::toCvShare(img_msg), "bgr8");
    // dump recved img msg after convert
    // cv::imwrite("dump_raw_" +
    //     std::to_string(img_msg->header.stamp.sec) + "." +
    //     std::to_string(img_msg->header.stamp.nanosec) + ".jpg",
    //     cv_img->image);

    {
      auto tp_now = std::chrono::system_clock::now();
      auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                          tp_now - tp_start)
                          .count();
      RCLCPP_DEBUG(rclcpp::get_logger("example"),
                   "after cvtColorForDisplay cost ms: %d",
                   interval);
    }

    pyramid = ImageUtils::GetNV12Pyramid(
        cv_img->image, model_input_height_, model_input_width_);
#else
    RCLCPP_ERROR(rclcpp::get_logger("example"), "Unsupport cv bridge");
#endif
  } else if ("nv12" == img_msg->encoding) {
    pyramid = hobot::dnn_node::ImageProc::GetNV12PyramidFromNV12Img(
        reinterpret_cast<const char *>(img_msg->data.data()),
        img_msg->height,
        img_msg->width,
        model_input_height_,
        model_input_width_);
  }

  if (!pyramid) {
    RCLCPP_ERROR(rclcpp::get_logger("example"), "Get Nv12 pym fail");
    return;
  }

  {
    auto tp_now = std::chrono::system_clock::now();
    auto interval =
        std::chrono::duration_cast<std::chrono::milliseconds>(tp_now - tp_start)
            .count();
    RCLCPP_DEBUG(rclcpp::get_logger("example"),
                 "after GetNV12Pyramid cost ms: %d",
                 interval);
  }

  // 2. 使用pyramid创建DNNInput对象inputs
  // inputs将会作为模型的输入通过RunInferTask接口传入
  auto inputs = std::vector<std::shared_ptr<DNNInput>>{pyramid};
  auto dnn_output = std::make_shared<DnnExampleOutput>();
  dnn_output->frame_id = img_msg->header.frame_id;
  dnn_output->stamp = img_msg->header.stamp;

  if (dump_render_img_) {
    dnn_output->pyramid = pyramid;
  }

  uint32_t ret = 0;
  // 3. 开始预测
  ret = Predict(inputs, nullptr, dnn_output);

  {
    auto tp_now = std::chrono::system_clock::now();
    auto interval =
        std::chrono::duration_cast<std::chrono::milliseconds>(tp_now - tp_start)
            .count();
    RCLCPP_DEBUG(
        rclcpp::get_logger("example"), "after Predict cost ms: %d", interval);
  }

  // 4. 处理预测结果，如渲染到图片或者发布预测结果
  if (ret != 0) {
    RCLCPP_INFO(rclcpp::get_logger("example"), "Run predict failed!");
    return;
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

  struct timespec time_start = {0, 0};
  clock_gettime(CLOCK_REALTIME, &time_start);

  std::stringstream ss;
  ss << "Recved img encoding: "
     << std::string(reinterpret_cast<const char *>(img_msg->encoding.data()))
     << ", h: " << img_msg->height << ", w: " << img_msg->width
     << ", step: " << img_msg->step << ", index: " << img_msg->index
     << ", stamp: " << img_msg->time_stamp.sec << "_"
     << img_msg->time_stamp.nanosec << ", data size: " << img_msg->data_size;
  RCLCPP_INFO(rclcpp::get_logger("example"), "%s", ss.str().c_str());

  auto tp_start = std::chrono::system_clock::now();

  // 1. 将图片处理成模型输入数据类型DNNInput
  // 使用图片生成pym，NV12PyramidInput为DNNInput的子类
  std::shared_ptr<hobot::dnn_node::NV12PyramidInput> pyramid = nullptr;
  if ("nv12" ==
      std::string(reinterpret_cast<const char *>(img_msg->encoding.data()))) {
    pyramid = hobot::dnn_node::ImageProc::GetNV12PyramidFromNV12Img(
        reinterpret_cast<const char *>(img_msg->data.data()),
        img_msg->height,
        img_msg->width,
        model_input_height_,
        model_input_width_);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("example"),
                 "Unsupported img encoding: %s",
                 img_msg->encoding.data());
    RCLCPP_ERROR(rclcpp::get_logger("example"),
                 "Only nv12 img encoding is supported for shared mem test");
    return;
  }

  if (!pyramid) {
    RCLCPP_ERROR(rclcpp::get_logger("example"), "Get Nv12 pym fail");
    return;
  }

  {
    auto tp_now = std::chrono::system_clock::now();
    auto interval =
        std::chrono::duration_cast<std::chrono::milliseconds>(tp_now - tp_start)
            .count();
    RCLCPP_DEBUG(rclcpp::get_logger("example"),
                 "after GetNV12Pyramid cost ms: %d",
                 interval);
  }

  // 2. 使用pyramid创建DNNInput对象inputs
  // inputs将会作为模型的输入通过RunInferTask接口传入
  auto inputs = std::vector<std::shared_ptr<DNNInput>>{pyramid};
  auto dnn_output = std::make_shared<DnnExampleOutput>();
  dnn_output->frame_id = std::to_string(img_msg->index);
  dnn_output->stamp = img_msg->time_stamp;

  if (dump_render_img_) {
    dnn_output->pyramid = pyramid;
  }

  dnn_output->preprocess_timespec_start = time_start;
  struct timespec time_now = {0, 0};
  clock_gettime(CLOCK_REALTIME, &time_now);
  dnn_output->preprocess_timespec_end = time_now;

  uint32_t ret = 0;
  // 3. 开始预测
  ret = Predict(inputs, nullptr, dnn_output);

  {
    auto tp_now = std::chrono::system_clock::now();
    auto interval =
        std::chrono::duration_cast<std::chrono::milliseconds>(tp_now - tp_start)
            .count();
    RCLCPP_DEBUG(
        rclcpp::get_logger("example"), "after Predict cost ms: %d", interval);
  }

  // 4. 处理预测结果，如渲染到图片或者发布预测结果
  if (ret != 0) {
    RCLCPP_INFO(rclcpp::get_logger("example"), "Run predict failed!");
    return;
  }
}
#endif

int DnnExampleNode::DnnParserInit() {
  if (config_file.empty()) {
    return 0;
  }
  // Parsing config
  std::ifstream ifs(config_file.c_str());
  rapidjson::IStreamWrapper isw(ifs);
  rapidjson::Document document;
  document.ParseStream(isw);
  if (document.HasParseError()) {
    RCLCPP_ERROR(rclcpp::get_logger("example"),
                 "Parsing config file %s failed",
                 config_file.data());
    return -1;
  }

  if (document.HasMember("model_file")) {
    model_file_name_ = document["model_file"].GetString();
  }
  if (document.HasMember("model_name")) {
    model_name_ = document["model_name"].GetString();
  }
  if (document.HasMember("cls_names_list")) {
    cls_name_file = document["cls_names_list"].GetString();
  }
  if (document.HasMember("dequanti_file")) {
    dequanti_file = document["dequanti_file"].GetString();
  }
  if (document.HasMember("dnn_Parser")) {
    std::string str_parser = document["dnn_Parser"].GetString();
    if ("yolov2" == str_parser) {
      parser = dnnParsers::YOLOV2_PARSER;
    } else if ("yolov3" == str_parser) {
      parser = dnnParsers::YOLOV3_PARSER;
    } else if ("yolov5" == str_parser) {
      parser = dnnParsers::YOLOV5_PARSER;
    } else if ("kps_parser" == str_parser) {
      parser = dnnParsers::FASTERRCNN_PARSER;
    } else if ("classification" == str_parser) {
      parser = dnnParsers::CLASSIFICATION_PARSER;
    } else if ("ssd" == str_parser) {
      parser = dnnParsers::SSD_PARSER;
    } else if ("efficient_det" == str_parser) {
      parser = dnnParsers::EFFICIENTDET_PARSER;
    } else if ("fcos" == str_parser) {
      parser = dnnParsers::FCOS_PARSER;
    } else if ("unet" == str_parser) {
      parser = dnnParsers::UNET_PARSER;
    } else {
      std::stringstream ss;
      ss << "Error!Invalid parser: " << str_parser
         << " . Only yolov2, yolov3, yolov5, kps_parser, ssd, fcos"
         << " efficient_det, classification, unet are supported";
      RCLCPP_ERROR(rclcpp::get_logger("example"), "%s", ss.str().c_str());
      return -3;
    }
  }

  model_output_count_ = document["model_output_count"].GetInt();
  return 0;
}
