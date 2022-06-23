# Changelog for package dnn_node

v1.0.1 (2022-06-23)
------------------
1. 优化异步推理流程，提升推理输出帧率。
2. 输出的推理结果中增加性能统计数据，包括推理和解析模型输出统计（开始和结束时间点，以及耗时）、推理输入帧率和推理成功的输出帧率，用于模型推理的性能分析。
3. 优化预置的mobilenet_unet模型后处理，降低后处理耗时和算法输出数据带宽。