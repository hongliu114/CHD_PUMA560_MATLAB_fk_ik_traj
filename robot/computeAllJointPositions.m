function [joint_positions, T_mats] = computeAllJointPositions(alpha, a, d, theta)
% computeAllJointPositions 计算所有关节的位置和变换矩阵
% 输出:
%   joint_positions: 7x3矩阵 (基座 + 6个关节)
%   T_mats: 6x1细胞数组, 每个元素为4x4变换矩阵

    joint_positions = zeros(7, 3); % 基座 + 6个关节
    T_mats = cell(6, 1);
    
    % 基座位置 (原点)
    joint_positions(1, :) = [0, 0, 0];
    
    % 初始化当前变换矩阵
    T_current = eye(4);
    
    % 计算每个关节的位置
    for i = 1:6
        % 计算当前连杆的变换矩阵
        T_i = computeTransformMatrix(alpha(i), a(i), d(i), theta(i));
        T_current = T_current * T_i;
        T_mats{i} = T_current;
        
        % 提取位置 (变换矩阵的第四列前三个元素)
        joint_positions(i+1, :) = T_current(1:3, 4)';
    end
end

function T_i = computeTransformMatrix(alpha_i, a_i, d_i, theta_i)
% computeTransformMatrix 计算单个连杆的变换矩阵
    c_theta = cos(theta_i);
    s_theta = sin(theta_i);
    c_alpha = cos(alpha_i);
    s_alpha = sin(alpha_i);
    
    T_i = [c_theta, -s_theta, 0, a_i;
           s_theta*c_alpha, c_theta*c_alpha, -s_alpha, -d_i*s_alpha;
           s_theta*s_alpha, c_theta*s_alpha, c_alpha, d_i*c_alpha;
           0, 0, 0, 1];
end