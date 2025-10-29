function [position, rotation] = extractPose(T)
% extractPose 从齐次变换矩阵提取位置和姿态
% 输入: 4x4齐次变换矩阵
% 输出: 位置向量(1x3), 旋转矩阵(3x3)

    position = T(1:3, 4)';
    rotation = T(1:3, 1:3);
end