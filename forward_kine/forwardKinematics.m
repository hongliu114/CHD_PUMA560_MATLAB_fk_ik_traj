function T_total = forwardKinematics(alpha, a, d, theta)
% forwardKinematics 计算六连杆机械臂的正运动学
% 输入: D-H参数向量 (1x6)
% 输出: 4x4齐次变换矩阵

    % 参数验证
    if length(alpha) ~= 6 || length(a) ~= 6 || length(d) ~= 6 || length(theta) ~= 6
        error('输入参数必须为1x6向量');
    end
    
    % 初始化单位矩阵
    T_total = eye(4);
    
    % 逐连杆计算变换矩阵
    for i = 1:6
        % 提取当前连杆参数
        alpha_i = alpha(i);
        a_i = a(i);
        d_i = d(i);
        theta_i = theta(i);
        
        % 计算三角函数
        c_theta = cos(theta_i);
        s_theta = sin(theta_i);
        c_alpha = cos(alpha_i);
        s_alpha = sin(alpha_i);
        
        % 严格按指定公式构建变换矩阵
        T_i = [c_theta, -s_theta, 0, a_i;
               s_theta*c_alpha, c_theta*c_alpha, -s_alpha, -d_i*s_alpha;
               s_theta*s_alpha, c_theta*s_alpha, c_alpha, d_i*c_alpha;
               0, 0, 0, 1];
        
        % 累乘得到总变换矩阵
        T_total = T_total * T_i;
    end
end