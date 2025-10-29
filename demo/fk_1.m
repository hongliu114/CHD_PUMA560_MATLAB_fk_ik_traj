% 正运动学单组数据测试程序
function fk_1()
    
    % 定义测试数据
    fprintf('=== 六连杆机械臂正运动学测试 ===\n\n');
    
    % D-H参数定义
    [alpha,a,d,qlim1,qlim2,qlim3,qlim4,qlim5,qlim6] = puma560_dh();
    theta = [0.5, -1, 0.2, -0.4, 0.1, 0.6];    % θ_i (弧度)
    
    % 显示D-H参数
    fprintf('D-H参数:\n');
    fprintf('α = [%s] 弧度\n', strjoin(arrayfun(@(x) sprintf('%.2f', x), alpha, 'UniformOutput', false), ', '));
    fprintf('a = [%s] m\n', strjoin(arrayfun(@(x) sprintf('%.2f', x), a, 'UniformOutput', false), ', '));
    fprintf('d = [%s] m\n', strjoin(arrayfun(@(x) sprintf('%.2f', x), d, 'UniformOutput', false), ', '));
    fprintf('θ = [%s] 弧度\n\n', strjoin(arrayfun(@(x) sprintf('%.2f', x), theta, 'UniformOutput', false), ', '));
    
    % 计算正运动学
    
    % 调用正运动学函数
    T_end = forwardKinematics(alpha, a, d, theta);
    
    % 提取末端位姿
    [pos, rot] = extractPose(T_end);
    
    % 结果显示
    fprintf('--- 计算结果 ---\n');
    
    % 显示完整的齐次变换矩阵
    fprintf('末端齐次变换矩阵 T_end:\n');
    for i = 1:4
        fprintf('[ ');
        for j = 1:4
            fprintf('%8.3f ', T_end(i,j));
        end
        fprintf(']\n');
    end
    fprintf('\n');
    
    % 显示末端位置和姿态
    fprintf('末端位置: [%.3f, %.3f, %.3f] m\n', pos(1), pos(2), pos(3));
    fprintf('\n末端旋转矩阵:\n');
    for i = 1:3
        fprintf('[ ');
        for j = 1:3
            fprintf('%8.4f ', rot(i,j));
        end
        fprintf(']\n');
    end
    
    % 3D可视化
    fprintf('--- 启动3D可视化 ---\n');
    visualizeRobot(alpha, a, d, theta, ...
        'frame_scale', 0.15, ...
        'link_style', '-bo', ...
        'joint_size', 160);


end