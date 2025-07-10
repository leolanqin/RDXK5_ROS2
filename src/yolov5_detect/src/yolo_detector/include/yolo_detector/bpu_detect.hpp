// 标准C++库
#include <iostream>  // 输入输出流
#include <vector>    // 向量容器
#include <algorithm> // 算法库
#include <chrono>    // 时间相关功能
#include <iomanip>   // 输入输出格式控制

// OpenCV库
#include <opencv2/opencv.hpp>  // OpenCV主要头文件
#include <opencv2/dnn/dnn.hpp> // OpenCV深度学习模块

// 地平线RDK BPU API
#include "dnn/hb_dnn.h"               // BPU基础功能
#include "dnn/hb_dnn_ext.h"           // BPU扩展功能
#include "dnn/plugin/hb_dnn_layer.h"  // BPU层定义
#include "dnn/plugin/hb_dnn_plugin.h" // BPU插件
#include "dnn/hb_sys.h"               // BPU系统功能

// 错误检查宏定义
#define RDK_CHECK_SUCCESS(value, errmsg)                        \
    do                                                          \
    {                                                           \
        auto ret_code = value;                                  \
        if (ret_code != 0)                                      \
        {                                                       \
            std::cout << errmsg << ", error code:" << ret_code; \
            return ret_code;                                    \
        }                                                       \
    } while (0);

// 模型和检测相关的默认参数定义
#define DEFAULT_MODEL_PATH "/home/AZHSXF/data/best3_640x640_bayese_nv12.bin" // 默认模型路径
#define DEFAULT_CLASSES_NUM 8                                                  // 默认类别数量
// #define CLASSES_LIST "close,open,6,5,4,3,2,1"
// #define CLASSES_LIST "close", "open", "6", "5", "4", "3", "2", "1"// 类别名称
#define CLASSES_LIST {"close", "open", "6", "5", "4", "3", "2", "1"}
#define DEFAULT_NMS_THRESHOLD 0.45f   // 非极大值抑制阈值
#define DEFAULT_SCORE_THRESHOLD 0.25f // 置信度阈值
#define DEFAULT_NMS_TOP_K 300         // NMS保留的最大框数
#define DEFAULT_FONT_SIZE 1.0f        // 绘制文字大小
#define DEFAULT_FONT_THICKNESS 1.0f   // 绘制文字粗细
#define DEFAULT_LINE_SIZE 2.0f        // 绘制线条粗细

// 运行模式选择
#define DETECT_MODE 1   // 检测模式: 0-单张图片, 1-实时检测
#define ENABLE_DRAW 1   // 绘图开关: 0-禁用, 1-启用
#define LOAD_FROM_DDR 0 // 模型加载方式: 0-从文件加载, 1-从内存加载

// 特征图尺度定义 (基于输入尺寸的倍数关系)
#define H_8 (input_h_ / 8)   // 输入高度的1/8
#define W_8 (input_w_ / 8)   // 输入宽度的1/8
#define H_16 (input_h_ / 16) // 输入高度的1/16
#define W_16 (input_w_ / 16) // 输入宽度的1/16
#define H_32 (input_h_ / 32) // 输入高度的1/32
#define W_32 (input_w_ / 32) // 输入宽度的1/32

// BPU目标检测类
class BPU_Detect
{
public:
    // 构造函数：初始化检测器的参数
    // @param model_path: 模型文件路径
    // @param classes_num: 检测类别数量
    // @param nms_threshold: NMS阈值
    // @param score_threshold: 置信度阈值
    // @param nms_top_k: NMS保留的最大框数
    BPU_Detect(const std::string &model_path = DEFAULT_MODEL_PATH,
               int classes_num = DEFAULT_CLASSES_NUM,
               float nms_threshold = DEFAULT_NMS_THRESHOLD,
               float score_threshold = DEFAULT_SCORE_THRESHOLD,
               int nms_top_k = DEFAULT_NMS_TOP_K,
               const std::vector<std::string> &class_names = std::vector<std::string>(CLASSES_LIST));

    // 析构函数：释放资源
    ~BPU_Detect();

    // 主要功能接口
    bool Init();                                                // 初始化BPU和模型
    bool Detect(const cv::Mat &input_img, cv::Mat &output_img); // 执行目标检测
    bool Release();                                             // 释放所有资源
    const std::vector<std::string> &GetClassNames() const 
    { 
        return class_names_; 
    }
    const std::vector<std::vector<cv::Rect2d>> &GetBBoxes() const
    {
        return bboxes_;
    }
    const std::vector<std::vector<float>> &GetScores() const
    {
        return scores_;
    }
    const std::vector<std::vector<int>> &GetIndices() const
    {
        return indices_;
    }

private:
    // 内部工具函数
    bool LoadModel();                          // 加载模型文件
    bool GetModelInfo();                       // 获取模型的输入输出信息
    bool PreProcess(const cv::Mat &input_img); // 图像预处理（resize和格式转换）
    bool Inference();                          // 执行模型推理
    bool PostProcess();                        // 后处理（NMS等）
    void DrawResults(cv::Mat &img);            // 在图像上绘制检测结果
    void PrintResults() const;                 // 打印检测结果到控制台

    // 特征图处理辅助函数
    // @param output_tensor: 输出tensor
    // @param height, width: 特征图尺寸
    // @param anchors: 对应尺度的anchor boxes
    // @param conf_thres_raw: 原始置信度阈值
    void ProcessFeatureMap(hbDNNTensor &output_tensor,
                           int height, int width,
                           const std::vector<std::pair<double, double>> &anchors,
                           float conf_thres_raw);

    // 成员变量（按照构造函数初始化顺序排列）
    std::string model_path_; // 模型文件路径
    int classes_num_;        // 类别数量
    float nms_threshold_;    // NMS阈值
    float score_threshold_;  // 置信度阈值
    int nms_top_k_;          // NMS保留的最大框数
    bool is_initialized_;    // 初始化状态标志
    float font_size_;        // 绘制文字大小
    float font_thickness_;   // 绘制文字粗细
    float line_size_;        // 绘制线条粗细

    // BPU相关变量
    hbPackedDNNHandle_t packed_dnn_handle_; // 打包模型句柄
    hbDNNHandle_t dnn_handle_;              // 模型句柄
    const char *model_name_;                // 模型名称

    // 输入输出张量
    hbDNNTensor input_tensor_;               // 输入tensor
    hbDNNTensor *output_tensors_;            // 输出tensor数组
    hbDNNTensorProperties input_properties_; // 输入tensor属性

    // 任务相关
    hbDNNTaskHandle_t task_handle_; // 推理任务句柄

    // 模型输入参数
    int input_h_; // 输入高度
    int input_w_; // 输入宽度

    // 检测结果存储
    std::vector<std::vector<cv::Rect2d>> bboxes_; // 每个类别的边界框
    std::vector<std::vector<float>> scores_;      // 每个类别的得分
    std::vector<std::vector<int>> indices_;       // NMS后的索引

    // 图像处理参数
    float x_scale_;       // X方向缩放比例
    float y_scale_;       // Y方向缩放比例
    int x_shift_;         // X方向偏移量
    int y_shift_;         // Y方向偏移量
    cv::Mat resized_img_; // 缩放后的图像

    // YOLOv5 anchors信息
    std::vector<std::pair<double, double>> s_anchors_; // 小目标anchors
    std::vector<std::pair<double, double>> m_anchors_; // 中目标anchors
    std::vector<std::pair<double, double>> l_anchors_; // 大目标anchors

    // 输出处理
    int output_order_[3];                  // 输出顺序映射
    std::vector<std::string> class_names_; // 类别名称列表
};
