#include "./TRTModule.h"

float X,Y;

namespace TRTInferV1
{
    // 来自老代码（rm_auto）（mlx）
    // static constexpr float MERGE_CONF_ERROR = 0.15;
    // static constexpr float MERGE_MIN_IOU = 0.9;

    // 来自老代码（rm_hero）（lsn）
    // static constexpr float MERGE_CONF_ERROR = 0.45;
    // static constexpr float MERGE_MIN_IOU = 0.25;

    //#ifdef Plane
    /**
     * @brief 计算两条线段的交点。
     *
     * 该函数计算由点 A 和 B 定义的线段与由点 C 和 D 定义的线段的交点。
     * 使用了线性方程组求解的方法。如果两条线段平行，则返回 (-1, -1)。
     *
     * @param A 线段 AB 的起点
     * @param B 线段 AB 的终点
     * @param C 线段 CD 的起点
     * @param D 线段 CD 的终点
     * @return cv::Point2f 交点坐标，如果线段平行则返回 (-1, -1)
     */
    cv::Point2f intersect(cv::Point2f A, cv::Point2f B, cv::Point2f C, cv::Point2f D) 
    {
        // Line AB represented as a1x + b1y = c1
        float a1 = B.y - A.y;
        float b1 = A.x - B.x;
        float c1 = a1 * (A.x) + b1 * (A.y);

        // Line CD represented as a2x + b2y = c2
        float a2 = D.y - C.y;
        float b2 = C.x - D.x;
        float c2 = a2 * (C.x) + b2 * (C.y);

        float determinant = a1 * b2 - a2 * b1;

        if (determinant == 0)
        { // The lines are parallel. This is simplified by assuming that the lines are not coincident.
            return cv::Point2f(-1, -1);
        }
        else
        {
            float x = (b2 * c1 - b1 * c2) / determinant;
            float y = (a1 * c2 - a2 * c1) / determinant;
            return cv::Point2f(x, y);
        }
    }
    
    /**
     * @brief 将图像坐标系下的点转换到以图像中心为原点的坐标系下。
     *
     * 该函数将给定的点从图像坐标系转换到以图像中心为原点的坐标系。
     * 通常用于将检测到的目标框坐标转换到以图像中心为参考的坐标系，方便后续计算。
     *
     * @param point 图像坐标系下的点
     * @param width 图像宽度
     * @param height 图像高度
     * @return cv::Point2f 以图像中心为原点的坐标系下的点
     */
    cv::Point2f translate_to_center(const cv::Point2f& point, int width, int height) 
    {
        // 计算图像中心坐标
        float center_x =  1280 / 2.0f;
        float center_y =  1024 / 2.0f;

        // 将点坐标转换为以图像中心为原点的坐标系
        return cv::Point2f(point.x - center_x, center_y - point.y);
    }
    //#endif

    /**
     * @brief 找出数组中最大值的索引
     * 
     * 该函数遍历一个浮点型数组，找出其中最大值的索引并返回。
     * 
     * @param ptr 指向浮点型数组的指针
     * @param len 数组的长度
     * @return int 数组中最大值的索引
     */
    inline int TRTInfer::argmax(const float *ptr, int len)
    {
        // 初始化最大值的索引为 0
        int max_arg = 0;
        // 从数组的第二个元素开始遍历
        for (int i = 1; i < len; ++i)
        {
            // 如果当前元素大于最大值，则更新最大值的索引
            if (ptr[i] > ptr[max_arg])
                max_arg = i;
        }
        // 返回最大值的索引
        return max_arg;
    }

    /**
     * @brief 对 DetectObject 向量进行原地快速排序，按置信度降序排列
     * 
     * 该函数使用快速排序算法对 `DetectObject` 向量进行原地排序，排序依据是每个对象的置信度（prob）。
     * 排序结果是置信度高的对象排在前面，置信度低的对象排在后面。
     * 
     * @param objects 存储 DetectObject 对象的向量
     * @param left 当前排序范围的左边界索引
     * @param right 当前排序范围的右边界索引
     */
    void TRTInfer::qsort_descent_inplace(std::vector<DetectObject> &objects, int left, int right)
    {
        // 初始化左指针为左边界索引
        int i = left;
        // 初始化右指针为右边界索引
        int j = right;
        // 选择中间元素的置信度作为基准值
        float p = objects[(left + right) / 2].prob;

        // 当左指针小于等于右指针时，继续分区操作
        while (i <= j)
        {
            // 从左向右找到第一个置信度小于等于基准值的元素
            while (objects[i].prob > p)
                i++;

            // 从右向左找到第一个置信度大于等于基准值的元素
            while (objects[j].prob < p)
                j--;

            // 如果左指针小于等于右指针，交换这两个元素
            if (i <= j)
            {
                // 交换元素
                std::swap(objects[i], objects[j]);
                // 左指针右移
                i++;
                // 右指针左移
                j--;
            }
        }

        // 如果左边界小于右指针，对左半部分继续排序
        if (left < j)
            this->qsort_descent_inplace(objects, left, j);
        // 如果左指针小于右边界，对右半部分继续排序
        if (i < right)
            this->qsort_descent_inplace(objects, i, right);
    }

    /**
     * @brief 对 DetectObject 向量进行原地快速排序，按置信度降序排列 (重载版本)
     *
     *  这是 `qsort_descent_inplace` 函数的重载版本，用于简化调用，直接对整个向量进行排序。
     *
     * @param objects 存储 DetectObject 对象的向量
     */
    void TRTInfer::qsort_descent_inplace(std::vector<DetectObject> &objects)
    {
        if (objects.empty())
            return;
        this->qsort_descent_inplace(objects, 0, objects.size() - 1);
    }

    /**
     * @brief 计算两个检测框的交集区域面积
     *
     *  该函数计算两个 `DetectObject` 的矩形框 `rect` 的交集面积。
     *  交集面积是计算 IOU (Intersection over Union) 的基础。
     *
     * @param a 第一个检测框
     * @param b 第二个检测框
     * @return float 交集区域面积
     */
    inline float TRTInfer::intersection_area(const DetectObject &a, const DetectObject &b)
    {
        return (a.rect & b.rect).area();
    }

    /**
     * @brief 对排序后的检测框进行非极大值抑制 (NMS)
     *
     *  该函数实现非极大值抑制算法，用于移除冗余的检测框。
     *  首先计算每个检测框的面积，然后遍历所有检测框，如果当前检测框与已选择的检测框的 IOU 大于阈值 `nms_threshold`，
     *  则认为当前检测框是冗余的，将其移除。
     *
     * @param objects 存储 DetectObject 对象的向量 (已按置信度排序)
     * @param picked 存储经过 NMS 后保留的检测框的索引
     * @param nms_threshold IOU 阈值，用于判断两个检测框是否重叠
     */
    void TRTInfer::nms_sorted_bboxes(std::vector<DetectObject> &objects, std::vector<int> &picked, float nms_threshold)
    {
        picked.clear(); // 清空已选择的检测框索引
        const int n = objects.size(); // 获取检测框数量

        std::vector<float> areas(n); // 存储每个检测框的面积
        for (int i = 0; i < n; i++)
        {
            areas[i] = objects[i].rect.area(); // 计算每个检测框的面积
        }

        for (int i = 0; i < n; i++)
        {
            DetectObject &a = objects[i]; // 当前检测框
            int keep = 1; // 标记是否保留当前检测框 (1: 保留, 0: 移除)
            for (int j = 0; j < (int)picked.size(); j++)
            {
                DetectObject &b = objects[picked[j]]; // 已选择的检测框
                // intersection over union
                float inter_area = intersection_area(a, b); // 计算交集面积
                float union_area = areas[i] + areas[picked[j]] - inter_area; // 计算并集面积
                float iou = inter_area / union_area; // 计算 IOU
                if (iou > nms_threshold || isnan(iou))
                {
                    keep = 0; // 如果 IOU 大于阈值，则移除当前检测框
                    // Stored for Merge
                    // if (iou > MERGE_MIN_IOU && abs(a.prob - b.prob) < MERGE_CONF_ERROR && a.label == b.label && a.color == b.color)//效果不明显
                    if (a.label == b.label && a.color == b.color)
                    {
                        for (int i = 0; i < 4; i++)
                        {
                            b.pts.emplace_back(a.apex[i]);
                        }
                    }
                }
            }
            if (keep)
                picked.emplace_back(i); // 如果保留当前检测框，则将其索引添加到已选择的检测框列表中
        }
    }

    /**
     * @brief 计算三角形的面积
     *
     *  使用海伦公式计算由三个点 `pts[3]` 定义的三角形的面积。
     *
     * @param pts 包含三个点的数组，定义三角形的顶点
     * @return float 三角形的面积
     */
    float TRTInfer::calcTriangleArea(cv::Point2f pts[3])
    {
        auto a = sqrt(pow((pts[0] - pts[1]).x, 2) + pow((pts[0] - pts[1]).y, 2));
        auto b = sqrt(pow((pts[1] - pts[2]).x, 2) + pow((pts[1] - pts[2]).y, 2));
        auto c = sqrt(pow((pts[2] - pts[0]).x, 2) + pow((pts[2] - pts[0]).y, 2));
        auto p = (a + b + c) / 2.f;
        return sqrt(p * (p - a) * (p - b) * (p - c));
    }

    /**
     * @brief 计算多边形的面积
     *
     *  该函数计算由 `pts[32]` 定义的多边形的面积。
     *  如果顶点数量大于 2，则将多边形分解为多个三角形，并计算所有三角形的面积之和。
     *  如果顶点数量小于等于 2，则计算一个矩形的面积（可能不准确）。
     *
     * @param pts 包含多边形顶点的数组
     * @return float 多边形的面积
     */
    float TRTInfer::calcPolygonArea(cv::Point2f pts[32])
    {
        int area = 0;
        if (this->num_apex > 2)
        {
            for (int i = 0; i < this->num_apex - 3; ++i)
            {
                area += calcTriangleArea(&pts[i]);
            }
        }
        else
        {
            area = abs(pts[1].x - pts[0].x) * abs(pts[1].y - pts[0].y);
        }
        return area;
    }

    /**
     * @brief 生成网格和步长信息
     *
     *  该函数根据目标宽度 `target_w`、目标高度 `target_h` 和步长列表 `strides`，生成网格和步长信息。
     *  网格和步长信息用于将特征图上的坐标映射回原始图像上的坐标。
     *
     * @param target_w 目标宽度
     * @param target_h 目标高度
     * @param strides 步长列表
     * @param grid_strides 存储网格和步长信息的向量
     */
    void TRTInfer::generate_grids_and_stride(const int target_w, const int target_h, std::vector<int> &strides, std::vector<GridAndStride> &grid_strides)
    {
        for (auto stride : strides)
        {
            int num_grid_w = target_w / stride;
            int num_grid_h = target_h / stride;

            for (int g1 = 0; g1 < num_grid_h; g1++)
            {
                for (int g0 = 0; g0 < num_grid_w; g0++)
                {
                    GridAndStride grid_stride = {g0, g1, stride};
                    grid_strides.emplace_back(grid_stride);
                }
            }
        }
    }

    /**
     * @brief 生成 YOLOX 的候选框
     *
     *  该函数根据网格和步长信息 `grid_strides`、特征图指针 `feat_ptr`、变换矩阵 `transform_matrix` 和置信度阈值 `prob_threshold`，
     *  生成 YOLOX 的候选框。
     *  该函数实现了 YOLOX 的解码逻辑，将特征图上的预测结果转换为原始图像上的坐标。
     *
     * @param grid_strides 网格和步长信息
     * @param feat_ptr 指向特征图的指针
     * @param transform_matrix 变换矩阵，用于将坐标从特征图映射回原始图像
     * @param prob_threshold 置信度阈值，用于过滤低置信度的候选框
     * @param objects 存储生成的候选框的向量
     */
    void TRTInfer::generateYoloxProposals(std::vector<GridAndStride> grid_strides, const float *feat_ptr,
                                          Eigen::Matrix<float, 3, 3> &transform_matrix, float prob_threshold,
                                          std::vector<DetectObject> &objects)
    {

        const int num_anchors = grid_strides.size();
        // Travel all the anchors
        for (int anchor_idx = 0; anchor_idx < num_anchors; anchor_idx++)
        {
            const int grid0 = grid_strides[anchor_idx].grid0;
            const int grid1 = grid_strides[anchor_idx].grid1;
            const int stride = grid_strides[anchor_idx].stride;
            const int basic_pos = anchor_idx * (9 + (num_colors) + num_classes);

            // yolox/models/yolo_head.py decode logic
            //  outputs[..., :2] = (outputs[..., :2] + grids) * strides
            //  outputs[..., 2:4] = torch.exp(outputs[..., 2:4]) * strides
            float x_1 = (feat_ptr[basic_pos + 0] + grid0) * stride;
            float y_1 = (feat_ptr[basic_pos + 1] + grid1) * stride;
            float x_2 = (feat_ptr[basic_pos + 2] + grid0) * stride;
            float y_2 = (feat_ptr[basic_pos + 3] + grid1) * stride;
            float x_3 = (feat_ptr[basic_pos + 4] + grid0) * stride;
            float y_3 = (feat_ptr[basic_pos + 5] + grid1) * stride;
            float x_4 = (feat_ptr[basic_pos + 6] + grid0) * stride;
            float y_4 = (feat_ptr[basic_pos + 7] + grid1) * stride;

            float box_objectness = (feat_ptr[basic_pos + 8]);
            int box_color = argmax(feat_ptr + basic_pos + 9, num_colors);
            int box_class = argmax(feat_ptr + basic_pos + 9 + num_colors, num_classes);

            // cout << "output:" << endl;
            // for (int ii = 0; ii < 25; ii++)
            // {
            //     cout << feat_ptr[basic_pos + ii] << " ";
            // }
            // cout << endl;
            // float color_conf = (feat_ptr[basic_pos + 9 + box_color]);
            // float cls_conf = (feat_ptr[basic_pos + 9 + NUM_COLORS + box_class]);
            // float box_prob = (box_objectness + cls_conf + color_conf) / 3.0;
            float box_prob = box_objectness;
            if (box_prob >= prob_threshold)
            {
                
                DetectObject obj;

                Eigen::Matrix<float, 3, 4> apex_norm;
                Eigen::Matrix<float, 3, 4> apex_dst;

                apex_norm << x_1, x_2, x_3, x_4,
                    y_1, y_2, y_3, y_4,
                    1, 1, 1, 1;

                apex_dst = transform_matrix * apex_norm;
                obj.pts.clear();
                for (int i = 0; i < 4; i++)
                {
                    obj.apex[i] = cv::Point2f(apex_dst(0, i), apex_dst(1, i));
                    obj.pts.push_back(obj.apex[i]);
                }

                std::vector<cv::Point2f> tmp(obj.apex, obj.apex + 4);
                obj.rect = cv::boundingRect(tmp);
                obj.label = box_class;
                obj.color = box_color;
                obj.prob = box_prob;
                if (obj.color != static_cast<int>(color_id))
                {
                    continue;
                }
                objects.push_back(obj);
            }
        } // point anchor loop
    }

    /**
     * @brief 解码输出特征图，生成检测结果
     *
     *  该函数是后处理的核心函数，它接收输出特征图 `prob`，并将其解码为 `DetectObject` 向量。
     *  该函数首先生成网格和步长信息，然后使用 `generateYoloxProposals` 函数生成候选框，
     *  接着使用 `qsort_descent_inplace` 函数对候选框按置信度进行排序，
     *  最后使用 `nms_sorted_bboxes` 函数进行非极大值抑制，得到最终的检测结果。
     *
     * @param prob 指向输出特征图的指针
     * @param objects 存储检测结果的向量
     * @param transform_matrix 变换矩阵，用于将坐标从特征图映射回原始图像
     * @param confidence_threshold 置信度阈值，用于过滤低置信度的候选框
     * @param nms_threshold IOU 阈值，用于非极大值抑制
     */
    void TRTInfer::decodeOutputs(const float *prob, std::vector<DetectObject> &objects, Eigen::Matrix<float, 3, 3> &transform_matrix, float confidence_threshold, float nms_threshold)
    {
        std::vector<DetectObject> proposals;
        std::vector<int> strides = {8, 16, 32};
        std::vector<GridAndStride> grid_strides;

        this->generate_grids_and_stride(this->input_dims.d[3], this->input_dims.d[2], strides, grid_strides);
        this->generateYoloxProposals(grid_strides, prob, transform_matrix, confidence_threshold, proposals);
        this->qsort_descent_inplace(proposals);
        if (int(proposals.size()) >= this->topK)
            proposals.resize(this->topK);
        std::vector<int> picked;
        this->nms_sorted_bboxes(proposals, picked, nms_threshold);
        int count = picked.size();
        objects.resize(count);

        for (int i = 0; i < count; i++)
        {
            objects[i] = proposals[picked[i]];
        }
    }

    /**
     * @brief 后处理函数，用于对模型的输出进行处理
     *
     *  该函数是整个后处理流程的入口函数，它接收模型的输出 `batch_res` 和原始图像 `frames`，
     *  并对每个图像的输出进行处理。
     *  处理过程包括：
     *  1. 计算缩放比例和填充大小，用于将图像缩放到模型输入大小。
     *  2. 构建变换矩阵，用于将坐标从模型输入空间映射回原始图像空间。
     *  3. 调用 `decodeOutputs` 函数解码输出特征图，生成检测结果。
     *  4. 对检测结果进行后处理，例如对角点进行平均，计算多边形面积等。
     *  5. (可选) 在图像上绘制检测结果。
     *
     * @param batch_res 存储每个图像的检测结果的向量
     * @param frames 原始图像向量
     * @param confidence_threshold 置信度阈值，用于过滤低置信度的候选框
     * @param nms_threshold IOU 阈值，用于非极大值抑制
     */
    void TRTInfer::postprocess(std::vector<std::vector<DetectObject>> &batch_res, std::vector<cv::Mat> &frames, float &confidence_threshold, float &nms_threshold)
    {

        for (int b = 0; b < int(frames.size()); ++b)
        {
            auto &res = batch_res[b];
            float r = std::min(this->input_dims.d[3] / (frames[b].cols * 1.0), this->input_dims.d[2] / (frames[b].rows * 1.0));
            int unpad_w = r * frames[b].cols;
            int unpad_h = r * frames[b].rows;

            int dw = this->input_dims.d[3] - unpad_w;
            int dh = this->input_dims.d[2] - unpad_h;

            dw /= 2;
            dh /= 2;

            Eigen::Matrix3f transform_matrix;
            transform_matrix << 1.0 / r, 0, -dw / r,
                0, 1.0 / r, -dh / r,
                0, 0, 1;
            this->decodeOutputs(&this->output[b * this->output_size], res, transform_matrix, confidence_threshold, nms_threshold);
            for (auto object = res.begin(); object != res.end(); ++object)
            {
                // 对候选框预测角点进行平均,降低误差
                if ((*object).pts.size() >= 8)
                {
                    auto N = (*object).pts.size();
                    cv::Point2f pts_final[4];
                    for (int i = 0; i < (int)N; i++)
                    {
                        pts_final[i % 4] += (*object).pts[i];
                    }

                    for (int i = 0; i < 4; i++)
                    {
                        pts_final[i].x = pts_final[i].x / (N / 4);
                        pts_final[i].y = pts_final[i].y / (N / 4);
                    }

                    (*object).apex[0] = pts_final[0];
                    (*object).apex[1] = pts_final[1];
                    (*object).apex[2] = pts_final[2];
                    (*object).apex[3] = pts_final[3];
                }
                (*object).area = (int)(this->calcPolygonArea((*object).apex));
            }
#ifdef UI_Show
            // 注释：以下代码块仅在定义了 Img_Show 宏时编译，用于在图像上绘制检测结果。
            for (int i = 0; i < int(res.size()); ++i)
            {
                //std::cout << "point nums: " << res[i].pts.size() << std::endl;

                cv::line(frames[b], cv::Point(res[i].pts[0].x, res[i].pts[0].y), cv::Point(res[i].pts[2].x, res[i].pts[2].y), cv::Scalar(0, 255, 0), 2);
                cv::line(frames[b], cv::Point(res[i].pts[2].x, res[i].pts[2].y), cv::Point(res[i].pts[3].x, res[i].pts[3].y), cv::Scalar(0, 255, 0), 2);
                cv::line(frames[b], cv::Point(res[i].pts[3].x, res[i].pts[3].y), cv::Point(res[i].pts[1].x, res[i].pts[1].y), cv::Scalar(0, 255, 0), 2);
                cv::line(frames[b], cv::Point(res[i].pts[1].x, res[i].pts[1].y), cv::Point(res[i].pts[0].x, res[i].pts[0].y), cv::Scalar(0, 255, 0), 2);

                cv::Point line1Start = cv::Point(res[i].pts[0].x, res[i].pts[0].y);
                cv::Point line1End = cv::Point(res[i].pts[2].x, res[i].pts[2].y);
                cv::Point line2Start = cv::Point(res[i].pts[2].x, res[i].pts[2].y);
                cv::Point line2End = cv::Point(res[i].pts[3].x, res[i].pts[3].y);
                cv::Point line3Start = cv::Point(res[i].pts[3].x, res[i].pts[3].y);
                cv::Point line3End = cv::Point(res[i].pts[1].x, res[i].pts[1].y);
                cv::Point line4Start = cv::Point(res[i].pts[1].x, res[i].pts[1].y);
                cv::Point line4End = cv::Point(res[i].pts[0].x, res[i].pts[0].y);
                cv::Vec4i line1(line1Start.x, line1Start.y, line1End.x, line1End.y);  
                cv::Vec4i line2(line2Start.x, line2Start.y, line2End.x, line2End.y);
                cv::Vec4i line3(line3Start.x, line3Start.y, line3End.x, line3End.y);
                cv::Vec4i line4(line4Start.x, line4Start.y, line2End.x, line4End.y);
                cv::Point2f intersection=intersect(line1Start, line1End, line3Start, line3End);
                #ifdef NT
                //circle(frames[b], intersection, 30, Scalar(0, 255, 255), -1);
                cv::circle(frames[b], intersection, 3, Scalar(255, 255, 255), cv::LINE_AA); //白点为绿线的实际中心
                #endif

                char test[100];
                sprintf(test, "label:%.1d", res[i].label);
                cv::putText(frames[b], test, cv::Point(res[i].pts[0].x, res[i].pts[0].y - 10), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2, 8);
                sprintf(test, "color:%.1d", res[i].color);
                #ifdef Plane
                cv::putText(frames[b], test, cv::Point(res[i].pts[0].x, res[i].pts[0].y - 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2, 8);
                cv::Point line1Start = cv::Point(res[i].pts[0].x, res[i].pts[0].y);
                cv::Point line1End = cv::Point(res[i].pts[2].x, res[i].pts[2].y);
                cv::Point line2Start = cv::Point(res[i].pts[2].x, res[i].pts[2].y);
                cv::Point line2End = cv::Point(res[i].pts[3].x, res[i].pts[3].y);
                cv::Point line3Start = cv::Point(res[i].pts[3].x, res[i].pts[3].y);
                cv::Point line3End = cv::Point(res[i].pts[1].x, res[i].pts[1].y);
                cv::Point line4Start = cv::Point(res[i].pts[1].x, res[i].pts[1].y);
                cv::Point line4End = cv::Point(res[i].pts[0].x, res[i].pts[0].y);
                cv::line(frames[b], line1Start, line1End,(0, 255, 255), 2);
                cv::line(frames[b], line3Start, line3End,(0, 255, 255), 2);
                cv::Vec4i line1(line1Start.x, line1Start.y, line1End.x, line1End.y);
                
                cv::Vec4i line2(line2Start.x, line2Start.y, line2End.x, line2End.y);
                cv::Vec4i line3(line3Start.x, line3Start.y, line3End.x, line3End.y);
                cv::Vec4i line4(line4Start.x, line4Start.y, line2End.x, line4End.y);
                cv::Point2f intersection=intersect(line1Start, line1End, line3Start, line3End);
                //if (intersection.x >= 0 && intersection.y >= 0) {
                circle(frames[b], intersection, 30, Scalar(0, 255, 255), -1);
                std::cout << "交点的 x=" << intersection.x << ", y=" << intersection.y << std::endl;
                cv::Point2f centeredIntersection = translate_to_center(intersection, frames[b].cols,frames[b].rows);
                std::cout << "Centered intersection point: (" << centeredIntersection.x << ", " << centeredIntersection.y << ")" << std::endl;
    
                X = centeredIntersection.x;
                Y = centeredIntersection.y;
                #endif
            }
#endif
        }
    }

    /**
     * @brief TRTInfer 类的构造函数
     *
     *  该构造函数用于初始化 TRTInfer 类，并设置 CUDA 设备。
     *
     * @param device CUDA 设备 ID
     */
    TRTInfer::TRTInfer(const int device)
    {
        cudaSetDevice(device);
    }

    /**
     * @brief TRTInfer 类的析构函数
     *
     *  该析构函数用于释放 TRTInfer 类占用的资源。
     */
    TRTInfer::~TRTInfer()
    {
    }

    /**
     * @brief 初始化 TRT 推理模块
     *
     *  该函数用于初始化 TRT 推理模块，包括：
     *  1. 从文件中加载 TRT engine。
     *  2. 创建 TRT runtime、engine 和 context。
     *  3. 分配 CUDA 内存用于输入和输出。
     *  4. 设置输入和输出 tensor 的名称和数据类型。
     *
     * @param engine_file_path TRT engine 文件路径
     * @param batch_size 推理批大小
     * @param num_apex 角点数量
     * @param num_classes 类别数量
     * @param num_colors 颜色数量
     * @param topK  保留的最大候选框数量
     * @return true 如果初始化成功，则返回 true；否则返回 false
     */
    bool TRTInfer::initModule(const std::string engine_file_path, const int batch_size, const int num_apex, const int num_classes, const int num_colors, const int topK)
    {
        assert(num_apex <= 32 && num_apex >= 2); // 确保角点数量在有效范围内
        assert(batch_size > 0 && num_classes > 0 && num_colors > 0 && topK > 0); // 确保参数大于 0
        this->num_apex = num_apex;
        this->num_classes = num_classes;
        this->num_colors = num_colors;
        this->topK = topK;
        char *trtModelStream{nullptr};
        size_t size{0};
        std::ifstream file(engine_file_path, std::ios::binary);
        if (file.good())
        {
            file.seekg(0, file.end);
            size = file.tellg();
            file.seekg(0, file.beg);
            trtModelStream = new char[size];
            assert(trtModelStream);
            file.read(trtModelStream, size);
            file.close();
        }
        else
        {
            this->gLogger.log(ILogger::Severity::kERROR, "Engine bad file");
            return false;
        }
        this->runtime = createInferRuntime(this->gLogger);
        assert(runtime != nullptr);
        this->engine = this->runtime->deserializeCudaEngine(trtModelStream, size);
        assert(this->engine != nullptr);
        this->context = this->engine->createExecutionContext();
        assert(context != nullptr);
        delete trtModelStream;
        this->input_dims = this->engine->getTensorShape(INPUT_BLOB_NAME);
        this->input_dims.d[0] = batch_size;
        this->output_dims = this->engine->getTensorShape(OUTPUT_BLOB_NAME);
        this->context->setInputShape(INPUT_BLOB_NAME, input_dims);
        this->output_size = output_dims.d[1] * output_dims.d[2];
        int IOtensorsNum = engine->getNbIOTensors();
        assert(IOtensorsNum == 2);
        for (int i = 0; i < IOtensorsNum; ++i)
        {
            if (strcmp(this->engine->getIOTensorName(i), INPUT_BLOB_NAME))
            {
                this->inputIndex = i;
                assert(this->engine->getTensorDataType(INPUT_BLOB_NAME) == nvinfer1::DataType::kFLOAT);
            }
            else if (strcmp(this->engine->getIOTensorName(i), OUTPUT_BLOB_NAME))
            {
                this->outputIndex = i;
                assert(this->engine->getTensorDataType(OUTPUT_BLOB_NAME) == nvinfer1::DataType::kFLOAT);
            }
        }
        if (this->inputIndex == -1 || this->outputIndex == -1)
        {
            this->gLogger.log(ILogger::Severity::kERROR, "Uncorrect Input/Output tensor name");
            delete context;
            delete engine;
            return false;
        }
        CHECK(cudaMalloc(&buffers[inputIndex], batch_size * this->input_dims.d[1] * this->input_dims.d[2] * this->input_dims.d[3] * sizeof(float)));
        CHECK(cudaMalloc(&buffers[outputIndex], batch_size * this->output_size * sizeof(float)));
        CHECK(cudaMallocHost((void **)&this->img_host, MAX_IMAGE_INPUT_SIZE_THRESH * 3 * sizeof(float)));
        CHECK(cudaMalloc((void **)&this->img_device, MAX_IMAGE_INPUT_SIZE_THRESH * 3 * sizeof(float)));
        this->output = (float *)malloc(batch_size * this->output_size * sizeof(float));
        this->_is_inited = true;
        return true;
    }

    void TRTInfer::unInitModule()
    {
        this->_is_inited = false;
        delete this->context;
        delete this->runtime;
        delete this->engine;
        CHECK(cudaFree(img_device));
        CHECK(cudaFreeHost(img_host));
        CHECK(cudaFree(this->buffers[this->inputIndex]));
        CHECK(cudaFree(this->buffers[this->outputIndex]));
    }

    std::vector<std::vector<DetectObject>> TRTInfer::doInference(std::vector<cv::Mat> &frames, float confidence_threshold, float nms_threshold)
    {
        if (!this->_is_inited)
        {
            this->gLogger.log(ILogger::Severity::kERROR, "Module not inited !");
            return {};
        }
        if (frames.size() == 0 || int(frames.size()) > this->input_dims.d[0])
        {
            this->gLogger.log(ILogger::Severity::kWARNING, "Invalid frames size");
            return {};
        }
        std::vector<std::vector<DetectObject>> batch_res(frames.size());
        cudaStream_t stream = nullptr;
        CHECK(cudaStreamCreate(&stream));
        float *buffer_idx = (float *)buffers[this->inputIndex];
        for (size_t b = 0; b < frames.size(); ++b)
        {
            cv::Mat &img = frames[b];
            if (img.empty())
                continue;
            size_t size_image = img.cols * img.rows * 3;
            size_t size_image_dst = this->input_dims.d[3] * this->input_dims.d[2] * 3;
            memcpy(img_host, img.data, size_image);
            CHECK(cudaMemcpyAsync(img_device, img_host, size_image, cudaMemcpyHostToDevice, stream));
            preprocess_kernel_img(img_device, img.cols, img.rows, buffer_idx, this->input_dims.d[3], this->input_dims.d[2], stream);
            buffer_idx += size_image_dst;
        }
        this->context->setOptimizationProfileAsync(0, stream);
        this->context->setTensorAddress(INPUT_BLOB_NAME, this->buffers[this->inputIndex]);
        this->context->setTensorAddress(OUTPUT_BLOB_NAME, this->buffers[this->outputIndex]);
        bool success = this->context->enqueueV3(stream);
        if (!success)
        {
            this->gLogger.log(ILogger::Severity::kERROR, "DoInference failed");
            CHECK(cudaStreamDestroy(stream));
            return {};
        }
        CHECK(cudaMemcpyAsync(this->output, buffers[this->outputIndex], frames.size() * this->output_size * sizeof(float), cudaMemcpyDeviceToHost, stream));
        CHECK(cudaStreamSynchronize(stream));
        CHECK(cudaStreamDestroy(stream));

        this->postprocess(batch_res, frames, confidence_threshold, nms_threshold);

        return batch_res;
    }

}
