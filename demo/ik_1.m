function all_solutions = ik_1()
% ik_1 - PUMA560机械臂逆运动学求解函数
% 输入: target_pose - 4x4目标位姿矩阵
% 输出: all_solutions - 8x6矩阵，每行为一组关节角解(弧度)

    target_pose = [1, 0, 0, 0.5;
                   0, 1, 0, 0.2;
                   0, 0, 1, 0.1;
                   0, 0, 0, 1];
    
    % 获取DH参数
    [alpha, a, d, qlim1, qlim2, qlim3, qlim4, qlim5, qlim6] = puma560_dh();
    
    % 提取目标位姿的位置和姿态信息
    nx = target_pose(1,1); ox = target_pose(1,2); ax = target_pose(1,3); px = target_pose(1,4);
    ny = target_pose(2,1); oy = target_pose(2,2); ay = target_pose(2,3); py = target_pose(2,4);
    nz = target_pose(3,1); oz = target_pose(3,2); az = target_pose(3,3); pz = target_pose(3,4);
    
    % DH参数赋值
    a2 = a(3); a3 = a(4); d2 = d(2); d4 = d(4);
    
    % 存储所有解
    all_solutions = zeros(8, 6);
    solution_count = 0;
    
    % 1) 求解theta1 - 两个解
    rho = sqrt(px^2 + py^2);
    phi = atan2(py, px);
    
    % 检查是否有解
    if abs(d2/rho) > 1
        warning('目标位置超出工作空间，无解');
        return;
    end
    
    % theta1的两个解
    discriminant = sqrt(px^2 + py^2 - d2^2);
    theta1_1 = atan2(py, px) - atan2(d2, discriminant);
    theta1_2 = atan2(py, px) - atan2(d2, -discriminant);
    
    theta1_solutions = [theta1_1, theta1_2];
    
    for i = 1:2
        theta1 = theta1_solutions(i);
        
        % 2) 求解theta3 - 每个theta1对应两个解
        k = (px^2 + py^2 + pz^2 - a2^2 - a3^2 - d2^2 - d4^2) / (2*a2);
        
        % 检查theta3是否有解
        if abs(k) > sqrt(a3^2 + d4^2)
            continue; % 跳过无解的情况
        end
        
        % theta3的两个解
        discriminant_3 = sqrt(a3^2 + d4^2 - k^2);
        theta3_1 = atan2(a3, d4) - atan2(k, discriminant_3);
        theta3_2 = atan2(a3, d4) - atan2(k, -discriminant_3);
        
        theta3_solutions = [theta3_1, theta3_2];
        
        for j = 1:2
            theta3 = theta3_solutions(j);
            
            % 3) 求解theta2
            s1 = sin(theta1); c1 = cos(theta1);
            s3 = sin(theta3); c3 = cos(theta3);
            
            % 计算theta23
            numerator1 = -(a3 + a2*c3)*pz + (c1*px + s1*py)*(a2*s3 - d4);
            numerator2 = (-d4 + a2*s3)*pz + (c1*px + s1*py)*(a2*c3 + a3);
            
            theta23 = atan2(numerator1, numerator2);
            theta2 = theta23 - theta3;
            
            % 4) 求解theta4
            s2 = sin(theta2); c2 = cos(theta2);
            s23 = sin(theta23); c23 = cos(theta23);
            
            % 检查奇异性
            arg1 = -ax*s1 + ay*c1;
            arg2 = -ax*c1*c23 - ay*s1*c23 + az*s23;
            
            if abs(arg1) < 1e-6 && abs(arg2) < 1e-6
                % 奇异位置，任意选择theta4
                theta4 = 0;
            else
                theta4 = atan2(arg1, arg2);
            end
            
            % 5) 求解theta5
            s4 = sin(theta4); c4 = cos(theta4);
            
            s5_arg = ax*(c1*c23*c4 + s1*s4) + ay*(s1*c23*c4 - c1*s4) - az*(s23*c4);
            c5_arg = ax*(-c1*s23) + ay*(-s1*s23) + az*(-c23);
            
            theta5 = atan2(-s5_arg, c5_arg);
            
            % 6) 求解theta6
            s5 = sin(theta5); c5 = cos(theta5);
            
            s6_arg = -nx*(c1*c23*s4 - s1*c4) - ny*(s1*c23*s4 + c1*c4) + nz*(s23*s4);
            c6_arg = nx*((c1*c23*c4 + s1*s4)*c5 - c1*s23*s5) + ...
                     ny*((s1*c23*c4 - c1*s4)*c5 - s1*s23*s5) - ...
                     nz*(s23*c4*c5 + c23*s5);
            
            theta6 = atan2(s6_arg, c6_arg);
            
            % 存储解
            solution_count = solution_count + 1;
            all_solutions(solution_count, :) = [theta1, theta2, theta3, theta4, theta5, theta6];
        end
    end
    
    % 对于每个前三个关节的解，theta4有两个可能的解（相差π）
    % 复制现有解并添加theta4 + π的解
    original_count = solution_count;
    for i = 1:original_count
        if solution_count < 8
            solution_count = solution_count + 1;
            all_solutions(solution_count, :) = all_solutions(i, :);
            all_solutions(solution_count, 4) = all_solutions(i, 4) + pi;
            
            % 重新计算theta5和theta6
            theta1 = all_solutions(solution_count, 1);
            theta2 = all_solutions(solution_count, 2);
            theta3 = all_solutions(solution_count, 3);
            theta4 = all_solutions(solution_count, 4);
            
            s1 = sin(theta1); c1 = cos(theta1);
            s4 = sin(theta4); c4 = cos(theta4);
            s23 = sin(theta2 + theta3); c23 = cos(theta2 + theta3);
            
            % 重新计算theta5
            s5_arg = ax*(c1*c23*c4 + s1*s4) + ay*(s1*c23*c4 - c1*s4) - az*(s23*c4);
            c5_arg = ax*(-c1*s23) + ay*(-s1*s23) + az*(-c23);
            theta5 = atan2(-s5_arg, c5_arg);
            
            % 重新计算theta6
            s5 = sin(theta5); c5 = cos(theta5);
            s6_arg = -nx*(c1*c23*s4 - s1*c4) - ny*(s1*c23*s4 + c1*c4) + nz*(s23*s4);
            c6_arg = nx*((c1*c23*c4 + s1*s4)*c5 - c1*s23*s5) + ...
                     ny*((s1*c23*c4 - c1*s4)*c5 - s1*s23*s5) - ...
                     nz*(s23*c4*c5 + c23*s5);
            theta6 = atan2(s6_arg, c6_arg);
            
            all_solutions(solution_count, 5) = theta5;
            all_solutions(solution_count, 6) = theta6;
        end
    end
    
    % 只返回有效解
    all_solutions = all_solutions(1:solution_count, :);
    
    % 显示结果
    fprintf('找到 %d 个逆运动学解:\n', solution_count);
    for i = 1:solution_count
        fprintf('解 %d: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f] (弧度)\n', ...
                i, all_solutions(i, :));
        fprintf('解 %d: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f] (度)\n', ...
                i, all_solutions(i, :) * 180/pi);
    end
    
    % 可视化所有解（时间序列展示）
    if solution_count > 0
        fprintf('\n正在可视化所有解（每2秒切换一个解）...\n');
        visualizeAllSolutionsSequential(all_solutions, target_pose, solution_count);
    end
end

function visualizeAllSolutionsSequential(all_solutions, target_pose, solution_count)
% 按时间顺序可视化所有逆运动学解
% 输入:
%   all_solutions: 所有解的关节角
%   target_pose: 目标位姿
%   solution_count: 解的数量

    % 创建图形窗口
    fig = figure('Name', 'PUMA560逆运动学解时序展示', 'NumberTitle', 'off', ...
                 'Position', [100, 100, 1200, 800]);
    
    % 创建机器人模型
    base_xyz = [0 0 0];
    robot_RTB = puma560_RTB(base_xyz);
    
    % 预定义显示范围
    axis_range = [-1, 1];
    
    % 提取目标点位置
    target_pos = target_pose(1:3, 4);
    
    % 创建控制按钮
    uicontrol('Style', 'pushbutton', 'String', '暂停/继续', ...
              'Position', [20, 20, 100, 30], ...
              'Callback', @togglePause);
    
    uicontrol('Style', 'pushbutton', 'String', '重新开始', ...
              'Position', [130, 20, 100, 30], ...
              'Callback', @restart);
    
    % 创建解信息显示文本
    info_text = uicontrol('Style', 'text', ...
                         'Position', [250, 20, 400, 30], ...
                         'FontSize', 12, 'FontWeight', 'bold', ...
                         'BackgroundColor', get(fig, 'Color'));
    
    % 全局变量控制动画
    global isPaused isRestart;
    isPaused = false;
    isRestart = false;
    
    % 循环展示所有解
    current_solution = 1;
    
    while true
        % 检查是否需要重新开始
        if isRestart
            current_solution = 1;
            isRestart = false;
        end
        
        % 检查窗口是否还存在
        if ~ishandle(fig)
            break;
        end
        
        % 如果没有暂停，则更新显示
        if ~isPaused
            % 获取当前解
            theta = all_solutions(current_solution, :);
            
            % 清除当前图形内容
            clf(fig);
            
            % 重新创建控制按钮（因为clf会清除所有内容）
            uicontrol('Style', 'pushbutton', 'String', '暂停/继续', ...
                      'Position', [20, 20, 100, 30], ...
                      'Callback', @togglePause);
            
            uicontrol('Style', 'pushbutton', 'String', '重新开始', ...
                      'Position', [130, 20, 100, 30], ...
                      'Callback', @restart);
            
            % 重新创建信息显示文本
            info_text = uicontrol('Style', 'text', ...
                                 'Position', [250, 20, 400, 30], ...
                                 'FontSize', 12, 'FontWeight', 'bold', ...
                                 'BackgroundColor', get(fig, 'Color'));
            
            % 创建主绘图区域
            ax = axes('Position', [0.1, 0.15, 0.8, 0.8]);
            
            % 使用机器人工具箱绘制当前解
            try
                robot_RTB.plot(theta, 'workspace', axis_range, 'view', [45, 30], ...
                              'nobase', 'noshadow', 'nowrist', 'nojaxes');
                hold on;
            catch
                % 如果复杂参数失败，使用简单参数
                robot_RTB.plot(theta);
                hold on;
                view(45, 30);
                xlim(axis_range);
                ylim(axis_range);
                zlim(axis_range);
            end
            
            % 添加目标点标记
            plot3(target_pos(1), target_pos(2), target_pos(3), ...
                  'ro', 'MarkerSize', 25, 'MarkerFaceColor', 'red', 'LineWidth', 3);
            
            % 添加目标坐标系
            plotTargetFrame(target_pose, 0.1);
            
            % 计算并显示末端执行器位置
            try
                T_end = robot_RTB.fkine(theta);
                if isa(T_end, 'SE3')
                    end_effector_pos = T_end.t;
                elseif isa(T_end, 'double') && size(T_end, 1) == 4 && size(T_end, 2) == 4
                    end_effector_pos = T_end(1:3, 4);
                else
                    error('Unexpected fkine output format');
                end
            catch
                % 备用计算方法
                [alpha, a, d, ~, ~, ~, ~, ~, ~] = puma560_dh();
                end_effector_pos = calculateEndEffectorPosition(alpha, a, d, theta);
            end
            
            plot3(end_effector_pos(1), end_effector_pos(2), end_effector_pos(3), ...
                  'go', 'MarkerSize', 12, 'MarkerFaceColor', 'green', ...
                  'MarkerEdgeColor', 'black', 'LineWidth', 3);
            
            % 绘制从目标点到末端执行器的连线
            plot3([target_pos(1), end_effector_pos(1)], ...
                  [target_pos(2), end_effector_pos(2)], ...
                  [target_pos(3), end_effector_pos(3)], ...
                  'k--', 'LineWidth', 2);
            
            % 计算位置误差
            position_error = norm(target_pos - end_effector_pos);
            
            % 设置标题
            title(sprintf('PUMA560逆运动学解 %d/%d\n位置误差: %.4f m\n关节角度: [%.1f°, %.1f°, %.1f°, %.1f°, %.1f°, %.1f°]', ...
                         current_solution, solution_count, position_error, theta * 180/pi), ...
                  'FontSize', 14, 'FontWeight', 'bold');
            
            % 设置坐标轴标签
            xlabel('X (m)', 'FontSize', 12);
            ylabel('Y (m)', 'FontSize', 12);
            zlabel('Z (m)', 'FontSize', 12);
            
            % 设置网格和坐标轴属性
            grid on;
            axis equal;
            
            % 更新信息显示
            set(info_text, 'String', sprintf('当前显示: 解 %d/%d | 误差: %.4f m | 按钮控制动画', ...
                                            current_solution, solution_count, position_error));
            
            hold off;
            
            % 移动到下一个解
            current_solution = current_solution + 1;
            if current_solution > solution_count
                current_solution = 1; % 循环回到第一个解
            end
        end
        
        % 等待2秒（分成小段检查，以便响应暂停）
        for wait_step = 1:20
            if ~ishandle(fig)
                return;
            end
            pause(0.1);
            if isPaused || isRestart
                break;
            end
        end
    end
    
    % 嵌套函数：暂停/继续控制
    function togglePause(~, ~)
        isPaused = ~isPaused;
        if isPaused
            fprintf('动画已暂停\n');
        else
            fprintf('动画继续\n');
        end
    end
    
    % 嵌套函数：重新开始控制
    function restart(~, ~)
        isRestart = true;
        isPaused = false;
        fprintf('动画重新开始\n');
    end
end

function plotTargetFrame(target_pose, scale)
% 绘制目标坐标系
    origin = target_pose(1:3, 4);
    x_axis = target_pose(1:3, 1) * scale;
    y_axis = target_pose(1:3, 2) * scale;
    z_axis = target_pose(1:3, 3) * scale;
    
    % X轴 - 红色
    quiver3(origin(1), origin(2), origin(3), ...
            x_axis(1), x_axis(2), x_axis(3), ...
            'r', 'LineWidth', 3, 'MaxHeadSize', 0.3);
    
    % Y轴 - 绿色
    quiver3(origin(1), origin(2), origin(3), ...
            y_axis(1), y_axis(2), y_axis(3), ...
            'g', 'LineWidth', 3, 'MaxHeadSize', 0.3);
    
    % Z轴 - 蓝色
    quiver3(origin(1), origin(2), origin(3), ...
            z_axis(1), z_axis(2), z_axis(3), ...
            'b', 'LineWidth', 3, 'MaxHeadSize', 0.3);
end

function end_pos = calculateEndEffectorPosition(alpha, a, d, theta)
% 计算末端执行器位置
    T = eye(4);
    for i = 1:6
        T_i = [cos(theta(i)), -sin(theta(i))*cos(alpha(i)), sin(theta(i))*sin(alpha(i)), a(i)*cos(theta(i));
               sin(theta(i)), cos(theta(i))*cos(alpha(i)), -cos(theta(i))*sin(alpha(i)), a(i)*sin(theta(i));
               0, sin(alpha(i)), cos(alpha(i)), d(i);
               0, 0, 0, 1];
        T = T * T_i;
    end
    end_pos = T(1:3, 4);
end
