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

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#ifdef CV_BRIDGE_PKG_ENABLED
#include "cv_bridge/cv_bridge.h"
#endif
#include "dnn_node/dnn_node.h"
#include "dnn_node/util/image_proc.h"
#include "include/image_utils.h"
#include "include/post_process/post_process_base.h"
#include "sensor_msgs/msg/image.hpp"

#ifdef SHARED_MEM_ENABLED
#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"
#endif

#include "ai_msgs/msg/capture_targets.hpp"
#include "ai_msgs/msg/perception_targets.hpp"

#ifndef DNN_EXAMPLE_NODE_H_
#define DNN_EXAMPLE_NODE_H_

using rclcpp::NodeOptions;

using hobot::dnn_node::DnnNode;
using hobot::dnn_node::DnnNodeOutput;
using hobot::dnn_node::DnnNodePara;
using hobot::dnn_node::ModelTaskType;
using hobot::dnn_node::TaskId;

using hobot::dnn_node::DNNInput;
using hobot::dnn_node::DNNResult;
using hobot::dnn_node::NV12PyramidInput;

using hobot::dnn_node::Model;
using hobot::dnn_node::ModelInferTask;
using hobot::dnn_node::ModelManager;
using hobot::dnn_node::ModelRoiInferTask;

using ai_msgs::msg::PerceptionTargets;

enum class DnnFeedType {
  // 用于预测的图片来源，0：本地图片；1：订阅到的image msg
  FROM_LOCAL = 0,
  FROM_SUB = 1
};

typedef enum {
  YOLOV2_PARSER = 0,      // 对应dnn_node中yolov2的output_parser算法
  YOLOV3_PARSER,          // 对应dnn_node中yolov3的output_parser算法
  YOLOV5_PARSER,          // 对应dnn_node中yolov5的output_parser算法
  FASTERRCNN_PARSER,      // 对应dnn_node中fasterRcnn的output_parser算法
  CLASSIFICATION_PARSER,  // 对应dnn_node中classification的output_parser算法
  SSD_PARSER,
  EFFICIENTDET_PARSER,
  FCOS_PARSER,
  UNET_PARSER
  /*define more*/
} dnnParsers;

struct DnnExampleOutput : public DnnNodeOutput {
  std::string image_name = "";
  std::string frame_id = "";
  std_msgs::msg::Header::_stamp_type stamp;
  float ratio = 1.0;       //缩放比例系数，无需缩放为1
  uint32_t padding_l = 0;  //图片左边padding空相素个数
  uint32_t padding_r = 0;  //图片右边padding空相素个数
  uint32_t padding_t = 0;  //图片上边padding空相素个数
  uint32_t padding_b = 0;  //图片下边padding空相素个数
  std::shared_ptr<hobot::easy_dnn::NV12PyramidInput> pyramid = nullptr;

  struct timespec preprocess_timespec_start;
  struct timespec preprocess_timespec_end;
};

class DnnExampleNode : public DnnNode {
 public:
  DnnExampleNode(const std::string &node_name,
                 const NodeOptions &options = NodeOptions());
  ~DnnExampleNode() override;

 protected:
  int SetNodePara() override;
  int SetOutputParser() override;

  int PostProcess(const std::shared_ptr<DnnNodeOutput> &outputs) override;

 private:
  int DnnParserInit();

  std::string config_file = "";
  std::string dequanti_file = "";
  int32_t output_index_ = 2;
  dnnParsers parser = dnnParsers::FASTERRCNN_PARSER;
  std::string cls_name_file;

  std::string msg_pub_topic_name_ = "hobot_dnn_detection";
  rclcpp::Publisher<ai_msgs::msg::PerceptionTargets>::SharedPtr msg_publisher_ =
      nullptr;

 private:
  std::shared_ptr<PostProcessBase> post_process_ = nullptr;
  std::string model_file_name_ = "config/multitask_body_kps_960x544.hbm";
  std::string model_name_ = "multitask_body_kps_960x544";
  ModelTaskType model_task_type_ = ModelTaskType::ModelInferType;

  int model_input_width_ = -1;
  int model_input_height_ = -1;
  int32_t model_output_count_ = 3;

  // 用于预测的图片来源，0：本地图片；1：订阅到的image msg
  int feed_type_ = 0;
  std::string image_ = "config/test.jpg";
  // 0: bgr, 1: nv12
  int image_type_ = 0;
  int image_width = 0;
  int image_height = 0;
  int dump_render_img_ = 0;
  int is_sync_mode_ = 0;
  // 使用shared mem通信方式订阅图片
  int is_shared_mem_sub_ = 0;

  int Predict(std::vector<std::shared_ptr<DNNInput>> &inputs,
              std::vector<std::shared_ptr<OutputDescription>> &output_descs,
              const std::shared_ptr<std::vector<hbDNNRoi>> rois,
              std::shared_ptr<DnnNodeOutput> dnn_output);
  int FeedFromLocal();
  int FeedFromSubscriber();

#ifdef SHARED_MEM_ENABLED
  rclcpp::SubscriptionHbmem<hbm_img_msgs::msg::HbmMsg1080P>::ConstSharedPtr
      sharedmem_img_subscription_ = nullptr;
  std::string sharedmem_img_topic_name_ = "/hbmem_img";
  void SharedMemImgProcess(
      const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr msg);
#endif

  rclcpp::Subscription<sensor_msgs::msg::Image>::ConstSharedPtr
      ros_img_subscription_ = nullptr;
  // 目前只支持订阅原图，可以使用压缩图"/image_raw/compressed" topic
  // 和sensor_msgs::msg::CompressedImage格式扩展订阅压缩图
  std::string ros_img_topic_name_ = "/image_raw";
  void RosImgProcess(const sensor_msgs::msg::Image::ConstSharedPtr msg);
};

#endif  // DNN_EXAMPLE_NODE_H_
