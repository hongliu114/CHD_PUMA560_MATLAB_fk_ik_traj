function demonstrateContinuousMotion(alpha, a, d, theta_sequence, velocity, acceleration, save_gif, gif_filename, motion_info)
% 执行连续运动演示
    
    % 创建图形窗口
    fig = figure('Name', '机械臂连续运动演示', ...
                 'NumberTitle', 'off', ...
                 'Position', [100, 100, 1400, 800]);
    
    % 预计算所有末端位置
    num_frames = size(theta_sequence, 1);
    end_effector_path = zeros(num_frames, 3);
    
    for i = 1:num_frames
        T = forwardKinematics(alpha, a, d, theta_sequence(i, :));
        end_effector_path(i, :) = T(1:3, 4)';
    end
    
    % 计算关键路径点的位置
    keyframe_indices = [1, round(num_frames*0.33), round(num_frames*0.67), num_frames];
    keyframe_positions = end_effector_path(keyframe_indices, :);
    
    % 初始化子图布局（只初始化一次）
    ax_main = subplot(3,3,[1,2,4,5,7,8]);
    
    % 预先创建机器人对象并设置初始姿态
    base_xyz = [0 0 0];
    robot_RTB = puma560_RTB(base_xyz);
    robot_RTB.display();
    
    % 切换到主视图并绘制初始姿态
    axes(ax_main);
    robot_RTB.plot(theta_sequence(1, :), 'nowrist', 'noname', 'noshadow');
    hold on;
    
    ax_joints = subplot(3,3,3);
    ax_velocity = subplot(3,3,6);
    ax_acceleration = subplot(3,3,9);
    
    % 预初始化其他子图的图形对象
    % 关节角度图
    axes(ax_joints);
    joint_lines = plot(1:num_frames, theta_sequence, 'LineWidth', 1.5);
    hold on;
    joint_markers = gobjects(6, 1);
    for joint = 1:6
        joint_markers(joint) = plot(1, theta_sequence(1, joint), ...
             'o', 'MarkerSize', 6, 'MarkerFaceColor', 'r', 'Color', joint_lines(joint).Color);
    end
    title('关节角度变化', 'FontSize', 10);
    xlabel('帧数'); ylabel('角度 (rad)');
    grid on;
    legend({'关节1', '关节2', '关节3', '关节4', '关节5', '关节6'}, ...
           'Location', 'northeast', 'FontSize', 8);
    xlim([1, num_frames]);
    
    % 速度图
    axes(ax_velocity);
    velocity_lines = plot(1:num_frames, velocity, 'LineWidth', 1.5);
    hold on;
    velocity_markers = gobjects(6, 1);
    for joint = 1:6
        velocity_markers(joint) = plot(1, velocity(1, joint), ...
             'o', 'MarkerSize', 6, 'MarkerFaceColor', 'r', 'Color', velocity_lines(joint).Color);
    end
    title('关节速度变化', 'FontSize', 10);
    xlabel('帧数'); ylabel('速度 (rad/s)');
    grid on;
    legend({'关节1', '关节2', '关节3', '关节4', '关节5', '关节6'}, ...
           'Location', 'northeast', 'FontSize', 8);
    xlim([1, num_frames]);
    
    % 加速度图
    axes(ax_acceleration);
    acceleration_lines = plot(1:num_frames, acceleration, 'LineWidth', 1.5);
    hold on;
    acceleration_markers = gobjects(6, 1);
    for joint = 1:6
        acceleration_markers(joint) = plot(1, acceleration(1, joint), ...
             'o', 'MarkerSize', 6, 'MarkerFaceColor', 'r', 'Color', acceleration_lines(joint).Color);
    end
    title('关节加速度变化', 'FontSize', 10);
    xlabel('帧数'); ylabel('加速度 (rad/s²)');
    grid on;
    legend({'关节1', '关节2', '关节3', '关节4', '关节5', '关节6'}, ...
           'Location', 'northeast', 'FontSize', 8);
    xlim([1, num_frames]);
   
    % 初始化动画
    if save_gif
        fprintf('生成GIF动画中...\n');
    end
    
    % 运动演示循环
    for frame = 1:num_frames
        % 当前关节角度
        theta_current = theta_sequence(frame, :);
        
        % 切换到主视图
        axes(ax_main);
        
        % 直接更新机器人姿态（不清除坐标轴）
        robot_RTB.plot(theta_current, 'nowrist', 'noname', 'noshadow');
        
        % 绘制末端执行器运动轨迹
        hold on;
        plot3(end_effector_path(1:frame, 1), ...
              end_effector_path(1:frame, 2), ...
              end_effector_path(1:frame, 3), ...
              'r-', 'LineWidth', 2, 'Color', [1, 0.3, 0.3]);
        
        % 绘制关键路径点（大的空心三角形）
        plot3(keyframe_positions(:,1), keyframe_positions(:,2), keyframe_positions(:,3), ...
              '^', 'MarkerSize', 5, 'MarkerEdgeColor', 'b', 'MarkerFaceColor', 'none', ...
              'LineWidth', 0.5, 'DisplayName', '关键路径点');
        
        % 标记当前末端位置
        scatter3(end_effector_path(frame, 1), ...
                 end_effector_path(frame, 2), ...
                 end_effector_path(frame, 3), ...
                 10, 'ro', 'filled', 'MarkerEdgeColor', 'r');
        
        % 设置主视图属性
        title(sprintf('机械臂连续运动演示 - 帧 %d/%d', frame, num_frames), ...
              'FontSize', 14, 'FontWeight', 'bold');
        xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
        grid on; axis equal;
        
        % 计算视图范围
        margin = 0.3;
        x_lim = [min(end_effector_path(:,1))-margin, max(end_effector_path(:,1))+margin];
        y_lim = [min(end_effector_path(:,2))-margin, max(end_effector_path(:,2))+margin];
        z_lim = [min(end_effector_path(:,3))-margin, max(end_effector_path(:,3))+margin];
        xlim(x_lim); ylim(y_lim); zlim(z_lim);
        view(45, 30);
        
        % 更新其他子图的标记点（不重新绘制整个图）
        for joint = 1:6
            set(joint_markers(joint), 'XData', frame, 'YData', theta_sequence(frame, joint));
            set(velocity_markers(joint), 'XData', frame, 'YData', velocity(frame, joint));
            set(acceleration_markers(joint), 'XData', frame, 'YData', acceleration(frame, joint));
        end
        
        % 刷新图形
        drawnow;
        
        % 保存GIF帧
        if save_gif
            saveGifFrame(fig, gif_filename, frame, num_frames);
        end
        
        % % 控制动画速度
        % pause(0.05); % 20fps
    end
    
    if save_gif
        fprintf('GIF动画已保存: %s\n', gif_filename);
    end
end