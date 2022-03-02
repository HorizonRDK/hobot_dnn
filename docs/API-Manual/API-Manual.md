# 1.概述
本文档主要介绍了地平线机器人开发平台项目中DNN Node的API、数据类型、结构体等。

# 2.数据相关类及结构
## 2.1 模型任务实体类型
```cpp
// 任务实体类型，包括以下几种类型
// - ModelInferTask: Pyramid或DDR模型任务
// - ModelRoiInferTask: Resizer模型任务，有且只有一个输入源为resizer (pyramid和roi)，剩余的为pyramid或DDR
enum class ModelTaskType {
  InvalidType = 0,
  ModelInferType = 1,
  ModelRoiInferType = 2
};
```
## 2.2 Dnn Node配置参数
```cpp
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
```



# 3.DnnNode类接口

## 3.1 Init()

```cpp
int Init();
```
执行初始化流程，只做pipeline的串联，具体的每个初始化步骤由用户（子类中）实现。

- 返回值
    - 0成功，非0失败。

## 3.2 PreProcess()
```cpp
virtual int PreProcess(std::vector<std::shared_ptr<DNNInput>> &inputs,
                         const TaskId& task_id,
                         const std::shared_ptr<std::vector<hbDNNRoi>> rois = nullptr) = 0;
```
1. 批量配置预测任务的输入数据。
2. 更新模型输入描述InputDescription/SetInputProcessor（如果需要，例如需要为crop检测模型指定抠图方法，全图检测不需要更新）。
3. 如果是抠图检测模型，还需要指定抠图区域roi数据（hbDNNRoi类型）。
4. DNNInput是easy dnn中定义的模型输入数据类型，用户必须根据实际使用的模型数据输入，继承DNNInput并定义模型输入数据类型。
5. easy dnn中定义了一些常用模型的输入数据类型，如pym输入类型NV12PyramidInput。

- 参数
    - [in] inputs 输入数据智能指针列表。
    - [in] task_id 预测任务ID。
    - [in] rois 抠图roi数据，只对抠图检测模型有效。
    
- 返回值
    - 0成功，非0失败。

## 3.3 RunInferTask()
```cpp
int RunInferTask(std::vector<std::shared_ptr<DNNResult>> &sync_outputs,
                   const TaskId& task_id,
                   const bool is_sync_mode = true,
                   const int timeout_ms = 1000);
```
执行推理任务。

- 参数
    - [out] outputs 输出数据智能指针列表，同步模式有效。
    - [in] task_id 预测任务ID。
    - [in] is_sync_mode 预测模式，true为同步模式，false为异步模式。
    - [in] timeout_ms 预测推理超时时间。
- 返回值
    - 0成功，非0失败。

## 3.4 PostProcess()
```cpp
virtual int PostProcess(const std::vector<std::shared_ptr<DNNResult>> &outputs) = 0;
```
处理解析后的模型输出数据，例如将输出封装成msg发布到总线。

DNNResult是easy dnn中定义的模型输出数据类型，用户必须根据实际使用的模型输出数据类型，继承DNNResult并定义模型输出数据类型。

例如对于检测模型，继承DNNResult的子类中需要定义检测框、置信度等输出数据类型。

- 参数
    - [out] outputs 输出数据智能指针列表。
- 返回值
    - 0成功，非0失败。

## 3.5 GetModel()
```cpp
Model* GetModel();
```
获取dnn node管理和推理使用的模型。

- 返回值
    - 返回已加载的模型指针。

## 3.6 SetNodePara()
```cpp
virtual int SetNodePara() = 0;
```
设置DnnNodePara类型的node para。

- 返回值
    - 0成功，非0失败。

## 3.7 SetOutputParser()
```cpp
virtual int SetOutputParser() = 0;
```

配置模型输出的解析方式。

- 返回值
  - 0成功，非0失败。

## 3.8 AllocTask()
```cpp
TaskId AllocTask(int timeout_ms = -1);
```
申请模型预测任务。

- 参数
    - [in] timeout_ms 申请超时时间。
- 返回值
    - 返回申请到的task id，小于0为无效id。

## 3.9 ReleaseTask()
```cpp
int ReleaseTask(const TaskId& task_id);
```
释放模型预测任务。

- 参数
    - [in] task_id 需要释放的task id。
- 返回值
    - 0成功，非0失败。

## 3.10 GetTask()
```cpp
std::shared_ptr<Task> GetTask(const TaskId& task_id);
```
根据预测任务ID获取任务task。

- 参数
    - [in] task_id 预测任务ID。
- 返回值
    - 返回预测任务task。

