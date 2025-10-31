function trajectory_solutions = ik_2()
% ik_2 - PUMA560机械臂连续位姿逆运动学求解函数
% 功能: 对一组连续位姿进行逆运动学求解，取每个位姿的第一个解
% 输出: trajectory_solutions - Nx6矩阵，每行为一个位姿的关节角解(弧度)

    % 定义轨迹参数
    x_start = -0.5;
    x_end = 0.6;
    x_step = 0.1;
    y_fixed = 0.3;
    z_fixed = 0.4;
    
    % 生成X坐标序列
    x_positions = x_start:x_step:x_end;
    num_poses = length(x_positions);
    
    fprintf('开始计算连续位姿轨迹...\n');
    fprintf('轨迹参数: X从%.1f到%.1f，步进%.1f，Y=%.1f，Z=%.1f\n', ...
            x_start, x_end, x_step, y_fixed, z_fixed);
    fprintf('总共%d个位姿点\n\n', num_poses);
    
    % 存储轨迹解
    trajectory_solutions = zeros(num_poses, 6);
    trajectory_poses = zeros(4, 4, num_poses);
    valid_poses = true(num_poses, 1);
    
    % 获取DH参数
    [alpha, a, d, qlim1, qlim2, qlim3, qlim4, qlim5, qlim6] = puma560_dh();
    
    % 对每个位姿点计算逆运动学
    for pose_idx = 1:num_poses
        % 构造当前目标位姿
        current_x = x_positions(pose_idx);
        target_pose = [1, 0, 0, current_x;
                       0, 1, 0, y_fixed;
                       0, 0, 1, z_fixed;
                       0, 0, 0, 1];
        
        trajectory_poses(:, :, pose_idx) = target_pose;
        
        fprintf('计算位姿 %d/%d: [%.1f, %.1f, %.1f]\n', ...
                pose_idx, num_poses, current_x, y_fixed, z_fixed);
        
        % 调用单个位姿的逆运动学求解
        solutions = solveSinglePoseIK(target_pose, alpha, a, d);
        
        if ~isempty(solutions)
            % 取第一个解
            trajectory_solutions(pose_idx, :) = solutions(1, :);
            fprintf('  -> 找到解: [%.2f°, %.2f°, %.2f°, %.2f°, %.2f°, %.2f°]\n', ...
                    solutions(1, :) * 180/pi);
        else
            valid_poses(pose_idx) = false;
            fprintf('  -> 无解，跳过此位姿\n');
        end
    end
    
    % 移除无解的位姿
    trajectory_solutions = trajectory_solutions(valid_poses, :);
    trajectory_poses = trajectory_poses(:, :, valid_poses);
    valid_count = sum(valid_poses);
    
    fprintf('\n轨迹计算完成！\n');
    fprintf('有效位姿数量: %d/%d\n', valid_count, num_poses);
    
    if valid_count > 0
        % 询问是否保存GIF动画
        save_gif = askForGifSave();
        gif_filename = '';
        if save_gif
            gif_filename = input('请输入GIF文件名（不含扩展名）: ', 's');
            if isempty(gif_filename)
                gif_filename = 'ik_trajectory_animation';
            end
            gif_filename = [gif_filename, '.gif'];
            fprintf('将保存动画为: %s\n', gif_filename);
        end
        
        fprintf('\n开始可视化连续轨迹...\n');
        visualizeTrajectory(trajectory_solutions, trajectory_poses, valid_count, save_gif, gif_filename);
    else
        fprintf('没有有效解，无法可视化\n');
    end
end

% function save_gif = askForGifSave()
% % 询问用户是否保存GIF动画
%     while true
%         user_input = input('是否保存为GIF动画？(y/n): ', 's');
%         if strcmpi(user_input, 'y') || strcmpi(user_input, 'yes')
%             save_gif = true;
%             break;
%         elseif strcmpi(user_input, 'n') || strcmpi(user_input, 'no')
%             save_gif = false;
%             break;
%         else
%             fprintf('请输入 y 或 n\n');
%         end
%     end
% end

function solutions = solveSinglePoseIK(target_pose, alpha, a, d)
% 单个位姿的逆运动学求解（基于ik_1的核心算法）
% 输入: target_pose - 4x4目标位姿矩阵
% 输出: solutions - 所有可能解的矩阵

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
    
    % 检查是否有解
    if abs(d2/rho) > 1
        solutions = [];
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
    solutions = all_solutions(1:solution_count, :);
end

function visualizeTrajectory(trajectory_solutions, trajectory_poses, num_poses, save_gif, gif_filename)
% 可视化连续轨迹（无clf/cla，稳定版）
% 输入:
%   trajectory_solutions: 轨迹关节角解 [N x 6]
%   trajectory_poses: 轨迹位姿 [4 x 4 x N]
%   num_poses: 位姿数量
%   save_gif: 是否保存GIF
%   gif_filename: GIF文件名

    fig = figure('Name', 'PUMA560连续轨迹可视化', 'NumberTitle', 'off', ...
                 'Position', [70, 70, 1000, 800]);

    % 创建机器人模型
    base_xyz = [0 0 0];
    robot_RTB = puma560_RTB(base_xyz);
    axis_range = [-0.5, 1.5];

    ax = axes('Parent', fig, 'Position', [0.15, 0.15, 0.7, 0.7]);
    view(ax, [45, 30]);
    hold(ax, 'on');

    % 初始化绘制
    theta_init = trajectory_solutions(1, :);
    try
        robot_RTB.plot(theta_init, 'workspace', axis_range, 'view', [45, 30], ...
                       'nobase', 'noshadow', 'nowrist', 'nojaxes');
    catch
        robot_RTB.plot(theta_init);
    end

    xlabel(ax, 'X (m)', 'FontSize', 12);
    ylabel(ax, 'Y (m)', 'FontSize', 12);
    zlabel(ax, 'Z (m)', 'FontSize', 12);
    grid(ax, 'on');
    axis(ax, 'equal');
    xlim(ax, axis_range);
    ylim(ax, axis_range);
    zlim(ax, axis_range);

    % 轨迹点（静态）
    target_positions = zeros(num_poses, 3);
    for i = 1:num_poses
        target_positions(i, :) = trajectory_poses(1:3, 4, i)';
    end
    plot3(ax, target_positions(:,1), target_positions(:,2), target_positions(:,3), ...
          '-', 'LineWidth', 2, 'DisplayName', '目标轨迹');
    plot3(ax, target_positions(1,1), target_positions(1,2), target_positions(1,3), ...
          'ks', 'MarkerFaceColor', 'k');
    plot3(ax, target_positions(end,1), target_positions(end,2), target_positions(end,3), ...
          'k^', 'MarkerFaceColor', 'k');

    % 动态对象（只创建一次）
    h_completed = plot3(ax, nan, nan, nan, 'go', 'MarkerSize', 6, 'MarkerFaceColor', 'green');
    h_future = plot3(ax, nan, nan, nan, 'co', 'MarkerSize', 4, 'MarkerFaceColor', 'cyan');
    h_current = plot3(ax, nan, nan, nan, 'ro', 'MarkerSize', 15, 'MarkerFaceColor', 'red', 'LineWidth', 3);
    h_end_effector = plot3(ax, nan, nan, nan, 'mo', 'MarkerSize', 10, 'MarkerFaceColor', 'magenta', ...
                           'MarkerEdgeColor', 'black', 'LineWidth', 2);
    h_error_line = plot3(ax, nan, nan, nan, 'k--', 'LineWidth', 1);
    h_target_x = quiver3(ax, 0,0,0, 0,0,0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.3);
    h_target_y = quiver3(ax, 0,0,0, 0,0,0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.3);
    h_target_z = quiver3(ax, 0,0,0, 0,0,0, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.3);

    % 控件
    uicontrol('Style', 'pushbutton', 'String', '暂停/继续', ...
              'Position', [20, 20, 100, 30], 'Callback', @togglePause);
    uicontrol('Style', 'pushbutton', 'String', '重新开始', ...
              'Position', [130, 20, 100, 30], 'Callback', @restart);
    info_text = uicontrol('Style', 'text', ...
                         'Position', [570, 20, 500, 30], ...
                         'FontSize', 12, 'FontWeight', 'bold', ...
                         'BackgroundColor', get(fig, 'Color'));

    % 全局控制
    global isPaused isRestart showTrajectory animationSpeed;
    isPaused = false;
    isRestart = false;
    showTrajectory = true;
    animationSpeed = 1.0;

    gif_delay = 0.4; % GIF间隔
    current_pose = 1;

    while true
        if isRestart
            current_pose = 1;
            isRestart = false;
        end
        if ~ishandle(fig), break; end
        if ~isPaused
            theta = trajectory_solutions(current_pose, :);
            current_target_pose = trajectory_poses(:,:,current_pose);
            current_target_pos = current_target_pose(1:3,4);

            % ✅ 用 animate 更新，不重绘整个图
            try
                robot_RTB.animate(theta);
            catch
                robot_RTB.plot(theta, 'workspace', axis_range, 'view', [45, 30], ...
                               'nobase', 'noshadow', 'nowrist', 'nojaxes');
            end

            % 更新轨迹点
            set(h_completed, 'XData', target_positions(1:current_pose-1,1), ...
                             'YData', target_positions(1:current_pose-1,2), ...
                             'ZData', target_positions(1:current_pose-1,3));
            set(h_future, 'XData', target_positions(current_pose+1:end,1), ...
                          'YData', target_positions(current_pose+1:end,2), ...
                          'ZData', target_positions(current_pose+1:end,3));
            set(h_current, 'XData', current_target_pos(1), ...
                           'YData', current_target_pos(2), ...
                           'ZData', current_target_pos(3));

            % 末端位置
            try
                T_end = robot_RTB.fkine(theta);
                if isa(T_end,'SE3')
                    end_effector_pos = T_end.t;
                else
                    end_effector_pos = T_end(1:3,4);
                end
            catch
                [alpha,a,d,~,~,~,~,~,~,~] = puma560_dh();
                end_effector_pos = calculateEndEffectorPosition(alpha,a,d,theta);
            end

            % 更新末端与连线
            set(h_end_effector,'XData',end_effector_pos(1),...
                               'YData',end_effector_pos(2),...
                               'ZData',end_effector_pos(3));
            set(h_error_line,'XData',[current_target_pos(1), end_effector_pos(1)],...
                             'YData',[current_target_pos(2), end_effector_pos(2)],...
                             'ZData',[current_target_pos(3), end_effector_pos(3)]);

            % 更新目标坐标系
            scale = 0.08;
            ox = current_target_pose(1:3,1)*scale;
            oy = current_target_pose(1:3,2)*scale;
            oz = current_target_pose(1:3,3)*scale;
            origin = current_target_pos;
            set(h_target_x,'XData',origin(1),'YData',origin(2),'ZData',origin(3),...
                           'UData',ox(1),'VData',ox(2),'WData',ox(3));
            set(h_target_y,'XData',origin(1),'YData',origin(2),'ZData',origin(3),...
                           'UData',oy(1),'VData',oy(2),'WData',oy(3));
            set(h_target_z,'XData',origin(1),'YData',origin(2),'ZData',origin(3),...
                           'UData',oz(1),'VData',oz(2),'WData',oz(3));

            title(ax, sprintf('PUMA560轨迹 %d/%d\n[%.1f°, %.1f°, %.1f°, %.1f°, %.1f°, %.1f°]', ...
                  current_pose, num_poses, theta*180/pi), ...
                  'FontSize',12,'FontWeight','bold');
            set(info_text,'String',sprintf('位姿 %d/%d | 目标 [%.2f, %.2f, %.2f]', ...
                current_pose, num_poses, current_target_pos));

            drawnow;

            % 保存GIF帧
            if save_gif && current_pose>1
                frame = getframe(fig);
                im = frame2im(frame);
                [imind, cm] = rgb2ind(im, 256);
                if current_pose==2
                    imwrite(imind, cm, gif_filename, 'gif', 'Loopcount', inf, 'DelayTime', gif_delay);
                    fprintf('开始保存GIF: %s\n', gif_filename);
                else
                    imwrite(imind, cm, gif_filename, 'gif', 'WriteMode', 'append', 'DelayTime', gif_delay);
                end
                if current_pose==num_poses
                    fprintf('GIF保存完成: %s\n', gif_filename);
                end
            end

            current_pose = current_pose + 1;
            if current_pose > num_poses, break; end
        end

        wait_time = 0.5 / animationSpeed;
        for i = 1:max(1, round(wait_time * 10))
            if ~ishandle(fig), return; end
            pause(0.1);
            if isPaused || isRestart, break; end
        end
    end

    % 控制函数
    function togglePause(~,~)
        isPaused = ~isPaused;
        if isPaused
            fprintf('暂停\n');
        else
            fprintf('继续\n');
        end
    end

    function restart(~,~)
        isRestart = true;
        isPaused = false;
        fprintf('重新开始\n');
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
            'r', 'LineWidth', 2, 'MaxHeadSize', 0.3);
    
    % Y轴 - 绿色
    quiver3(origin(1), origin(2), origin(3), ...
            y_axis(1), y_axis(2), y_axis(3), ...
            'g', 'LineWidth', 2, 'MaxHeadSize', 0.3);
    
    % Z轴 - 蓝色
    quiver3(origin(1), origin(2), origin(3), ...
            z_axis(1), z_axis(2), z_axis(3), ...
            'b', 'LineWidth', 2, 'MaxHeadSize', 0.3);
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
