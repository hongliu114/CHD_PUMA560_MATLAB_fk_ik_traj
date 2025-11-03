function trajectory_solutions = ik_3()
% ik_3 - PUMA560机械臂连续位姿逆运动学求解函数（带实时图表，支持关键点插值）
% 功能: 通过关键位姿点插值生成轨迹，进行逆运动学求解
% 输出: trajectory_solutions - Nx6矩阵，每行为一个位姿的关节角解(弧度)

    % ========== 定义关键位姿点 ==========
    fprintf('========== 定义轨迹关键位姿点 ==========\n');
    
    % 关键位姿点1
    keypoint1.x = -0.5;
    keypoint1.y = 0.3;
    keypoint1.z = 0.0;
    keypoint1.roll = 0;      % 单位：度
    keypoint1.pitch = 0;
    keypoint1.yaw = 0;
    
    % 关键位姿点2
    keypoint2.x = -0.4;
    keypoint2.y = 0.3;
    keypoint2.z = 0.4;
    keypoint2.roll = 0;
    keypoint2.pitch = 0;
    keypoint2.yaw = 0;
    
    % 关键位姿点3
    keypoint3.x = 0.3;
    keypoint3.y = 0.3;
    keypoint3.z = 0.4;
    keypoint3.roll = 0;
    keypoint3.pitch = 90;
    keypoint3.yaw = 0;
    
    % 关键位姿点4
    keypoint4.x = 0.3;
    keypoint4.y = 0.3;
    keypoint4.z = 0.6;
    keypoint4.roll = 0;
    keypoint4.pitch = 0;
    keypoint4.yaw = 70;

    % 关键位姿点5
    keypoint5.x = 0.3;
    keypoint5.y = -0.3;
    keypoint5.z = 0.6;
    keypoint5.roll = 0;
    keypoint5.pitch = 0;
    keypoint5.yaw = 70;

    % 关键位姿点6
    keypoint6.x = 0.3;
    keypoint6.y = -0.3;
    keypoint6.z = 0.0;
    keypoint6.roll = 0;
    keypoint6.pitch = 0;
    keypoint6.yaw = -30;

    % 关键位姿点7
    keypoint7.x = 0.3;
    keypoint7.y= -0.3;
    keypoint7.z = 0.7;
    keypoint7.roll = 0;
    keypoint7.pitch = 0;
    keypoint7.yaw = -30;
    
    % 将关键位姿点放入数组
    keypoints = {keypoint1, keypoint2, keypoint3, keypoint4, keypoint5, keypoint6, keypoint7};
    
    % ========== 定义每两个关键点之间的插值点数量 ==========
    % 注意：如果有N个关键点，需要定义N-1个插值数量
    % 插值数量包含起点，不包含终点（终点是下一段的起点）
    num_interpolations = [4, 8, 3, 7, 6, 4];
    
    % 检查插值数量定义是否正确
    if length(num_interpolations) ~= length(keypoints) - 1
        error('插值数量定义错误！应该定义%d个插值数量（关键点数-1）', length(keypoints)-1);
    end
    
    % 打印关键点信息
    fprintf('\n关键位姿点数量: %d\n', length(keypoints));
    for i = 1:length(keypoints)
        kp = keypoints{i};
        fprintf('关键点%d: 位置[%.2f, %.2f, %.2f] 姿态[R=%.1f°, P=%.1f°, Y=%.1f°]\n', ...
                i, kp.x, kp.y, kp.z, kp.roll, kp.pitch, kp.yaw);
    end
    
    fprintf('\n插值配置:\n');
    total_poses = sum(num_interpolations) + 1;  % 总位姿数 = 所有插值点 + 最后一个关键点
    for i = 1:length(num_interpolations)
        fprintf('  段%d (关键点%d -> 关键点%d): %d个插值点\n', ...
                i, i, i+1, num_interpolations(i));
    end
    fprintf('总位姿点数量: %d\n', total_poses);
    
    % ========== 生成插值轨迹 ==========
    fprintf('\n========== 开始生成插值轨迹 ==========\n');
    
    trajectory_poses = zeros(4, 4, total_poses);
    pose_idx = 1;
    
    for seg_idx = 1:length(num_interpolations)
        % 获取起点和终点
        start_point = keypoints{seg_idx};
        end_point = keypoints{seg_idx + 1};
        
        % 当前段的插值点数量
        num_points = num_interpolations(seg_idx);
        
        fprintf('\n生成段%d: 关键点%d -> 关键点%d (%d个点)\n', ...
                seg_idx, seg_idx, seg_idx+1, num_points);
        
        % 生成插值序列（位置）
        x_interp = linspace(start_point.x, end_point.x, num_points);
        y_interp = linspace(start_point.y, end_point.y, num_points);
        z_interp = linspace(start_point.z, end_point.z, num_points);
        
        % 生成插值序列（姿态角，度转弧度）
        roll_interp = linspace(start_point.roll, end_point.roll, num_points) * pi/180;
        pitch_interp = linspace(start_point.pitch, end_point.pitch, num_points) * pi/180;
        yaw_interp = linspace(start_point.yaw, end_point.yaw, num_points) * pi/180;
        
        % 生成当前段的所有位姿（不包括最后一个点，避免重复）
        for i = 1:num_points
            if seg_idx == length(num_interpolations) || i < num_points
                % 构造位姿矩阵
                trajectory_poses(:, :, pose_idx) = constructPoseMatrix(...
                    x_interp(i), y_interp(i), z_interp(i), ...
                    roll_interp(i), pitch_interp(i), yaw_interp(i));
                
                if i == 1 || i == num_points || mod(i, max(1, floor(num_points/3))) == 0
                    fprintf('  点%d: [%.2f, %.2f, %.2f] [R=%.1f°, P=%.1f°, Y=%.1f°]\n', ...
                            pose_idx, x_interp(i), y_interp(i), z_interp(i), ...
                            roll_interp(i)*180/pi, pitch_interp(i)*180/pi, yaw_interp(i)*180/pi);
                end
                
                pose_idx = pose_idx + 1;
            end
        end
    end
    
    % 添加最后一个关键点
    last_point = keypoints{end};
    trajectory_poses(:, :, pose_idx) = constructPoseMatrix(...
        last_point.x, last_point.y, last_point.z, ...
        last_point.roll * pi/180, last_point.pitch * pi/180, last_point.yaw * pi/180);
    
    fprintf('  点%d (终点): [%.2f, %.2f, %.2f] [R=%.1f°, P=%.1f°, Y=%.1f°]\n', ...
            pose_idx, last_point.x, last_point.y, last_point.z, ...
            last_point.roll, last_point.pitch, last_point.yaw);
    
    num_poses = pose_idx;
    
    fprintf('\n========== 轨迹生成完成 ==========\n');
    fprintf('实际生成位姿点数量: %d\n', num_poses);
    
    % ========== 对所有位姿进行逆运动学求解 ==========
    fprintf('\n========== 开始逆运动学求解 ==========\n');
    
    trajectory_solutions = zeros(num_poses, 6);
    valid_poses = true(num_poses, 1);
    
    % 获取DH参数
    [alpha, a, d, qlim1, qlim2, qlim3, qlim4, qlim5, qlim6] = puma560_dh();
    
    % 对每个位姿点计算逆运动学
    for pose_idx = 1:num_poses
        target_pose = trajectory_poses(:, :, pose_idx);
        
        fprintf('计算位姿 %d/%d: [%.2f, %.2f, %.2f]\n', ...
                pose_idx, num_poses, ...
                target_pose(1,4), target_pose(2,4), target_pose(3,4));
        
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

function T = constructPoseMatrix(x, y, z, roll, pitch, yaw)
% 根据位置和RPY角度构造4x4位姿变换矩阵
% 输入: 
%   x, y, z - 位置（米）
%   roll, pitch, yaw - 姿态角（弧度）
% 输出: T - 4x4齐次变换矩阵
% 旋转顺序：R = Rz(yaw) * Ry(pitch) * Rx(roll)

    % Roll旋转矩阵（绕X轴）
    Rx = [1,         0,          0;
          0,  cos(roll), -sin(roll);
          0,  sin(roll),  cos(roll)];
    
    % Pitch旋转矩阵（绕Y轴）
    Ry = [cos(pitch),  0,  sin(pitch);
          0,           1,           0;
         -sin(pitch),  0,  cos(pitch)];
    
    % Yaw旋转矩阵（绕Z轴）
    Rz = [cos(yaw), -sin(yaw),  0;
          sin(yaw),  cos(yaw),  0;
          0,         0,          1];
    
    % 组合旋转矩阵：R = Rz * Ry * Rx
    R = Rz * Ry * Rx;
    
    % 构造4x4齐次变换矩阵
    T = [R, [x; y; z];
         0, 0, 0, 1];
end

function save_gif = askForGifSave()
    save_gif = true;
end

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

function [roll, pitch, yaw] = extractRPY(T)
% 从4x4变换矩阵中提取Roll-Pitch-Yaw角度（ZYX欧拉角）
% 输入: T - 4x4齐次变换矩阵
% 输出: roll, pitch, yaw - 弧度制的欧拉角

    R = T(1:3, 1:3);  % 提取旋转矩阵
    
    % ZYX欧拉角提取（Roll-Pitch-Yaw）
    pitch = atan2(-R(3,1), sqrt(R(3,2)^2 + R(3,3)^2));
    
    % 检查万向锁情况
    if abs(cos(pitch)) > 1e-6
        roll = atan2(R(3,2), R(3,3));
        yaw = atan2(R(2,1), R(1,1));
    else
        % 万向锁情况
        roll = 0;
        yaw = atan2(-R(1,2), R(2,2));
    end
end

function visualizeTrajectory(trajectory_solutions, trajectory_poses, num_poses, save_gif, gif_filename)
% 可视化连续轨迹（带实时图表）
% 输入:
%   trajectory_solutions: 轨迹关节角解 [N x 6]
%   trajectory_poses: 轨迹位姿 [4 x 4 x N]
%   num_poses: 位姿数量
%   save_gif: 是否保存GIF
%   gif_filename: GIF文件名

    % 创建大窗口
    fig = figure('Name', 'PUMA560连续轨迹可视化（带实时图表）', 'NumberTitle', 'off', ...
                 'Position', [50, 50, 1600, 900]);

    % 创建机器人模型
    base_xyz = [0 0 0];
    robot_RTB = puma560_RTB(base_xyz);
    axis_range = [-0.5, 1.5];

    % ========== 左侧3D图 ==========
    ax_3d = subplot('Position', [0.05, 0.15, 0.50, 0.75]);
    view(ax_3d, [45, 30]);
    hold(ax_3d, 'on');

    % 初始化绘制
    theta_init = trajectory_solutions(1, :);
    try
        robot_RTB.plot(theta_init, 'workspace', axis_range, 'view', [45, 30], ...
                       'nobase', 'noshadow', 'nowrist', 'nojaxes');
    catch
        robot_RTB.plot(theta_init);
    end

    xlabel(ax_3d, 'X (m)', 'FontSize', 11);
    ylabel(ax_3d, 'Y (m)', 'FontSize', 11);
    zlabel(ax_3d, 'Z (m)', 'FontSize', 11);
    grid(ax_3d, 'on');
    axis(ax_3d, 'equal');
    xlim(ax_3d, axis_range);
    ylim(ax_3d, axis_range);
    zlim(ax_3d, axis_range);

    % 轨迹点（静态）
    target_positions = zeros(num_poses, 3);
    for i = 1:num_poses
        target_positions(i, :) = trajectory_poses(1:3, 4, i)';
    end
    plot3(ax_3d, target_positions(:,1), target_positions(:,2), target_positions(:,3), ...
          '-', 'LineWidth', 2, 'DisplayName', '目标轨迹');
    plot3(ax_3d, target_positions(1,1), target_positions(1,2), target_positions(1,3), ...
          'ks', 'MarkerFaceColor', 'k');
    plot3(ax_3d, target_positions(end,1), target_positions(end,2), target_positions(end,3), ...
          'k^', 'MarkerFaceColor', 'k');

    % 动态对象（只创建一次）
    h_completed = plot3(ax_3d, nan, nan, nan, 'go', 'MarkerSize', 6, 'MarkerFaceColor', 'green');
    h_future = plot3(ax_3d, nan, nan, nan, 'co', 'MarkerSize', 4, 'MarkerFaceColor', 'cyan');
    h_current = plot3(ax_3d, nan, nan, nan, 'ro', 'MarkerSize', 15, 'MarkerFaceColor', 'red', 'LineWidth', 3);
    h_end_effector = plot3(ax_3d, nan, nan, nan, 'mo', 'MarkerSize', 10, 'MarkerFaceColor', 'magenta', ...
                           'MarkerEdgeColor', 'black', 'LineWidth', 2);
    h_error_line = plot3(ax_3d, nan, nan, nan, 'k--', 'LineWidth', 1);
    h_target_x = quiver3(ax_3d, 0,0,0, 0,0,0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.3);
    h_target_y = quiver3(ax_3d, 0,0,0, 0,0,0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.3);
    h_target_z = quiver3(ax_3d, 0,0,0, 0,0,0, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.3);

    % ========== 右侧四个2D图表 ==========
    % 图1: 关节角度
    ax_joints = subplot('Position', [0.62, 0.77, 0.35, 0.18]);
    hold(ax_joints, 'on');
    grid(ax_joints, 'on');
    title(ax_joints, '关节角度变化', 'FontSize', 10, 'FontWeight', 'bold');
    xlabel(ax_joints, '位姿序号', 'FontSize', 9);
    ylabel(ax_joints, '角度 (°)', 'FontSize', 9);
    colors = lines(6);
    h_joint_lines = gobjects(6, 1);
    for i = 1:6
        h_joint_lines(i) = plot(ax_joints, nan, nan, '-o', 'Color', colors(i,:), ...
                                'LineWidth', 1.5, 'MarkerSize', 4, 'DisplayName', sprintf('θ%d', i));
    end
    legend(ax_joints, 'Location', 'eastoutside', 'FontSize', 8);
    xlim(ax_joints, [1, num_poses]);
    
    % 图2: Roll角度
    ax_roll = subplot('Position', [0.62, 0.53, 0.35, 0.18]);
    hold(ax_roll, 'on');
    grid(ax_roll, 'on');
    title(ax_roll, '末端Roll角度', 'FontSize', 10, 'FontWeight', 'bold');
    xlabel(ax_roll, '位姿序号', 'FontSize', 9);
    ylabel(ax_roll, 'Roll (°)', 'FontSize', 9);
    h_roll = plot(ax_roll, nan, nan, '-o', 'Color', [0.8500 0.3250 0.0980], ...
                  'LineWidth', 2, 'MarkerSize', 5, 'MarkerFaceColor', [0.8500 0.3250 0.0980]);
    xlim(ax_roll, [1, num_poses]);
    
    % 图3: Pitch角度
    ax_pitch = subplot('Position', [0.62, 0.29, 0.35, 0.18]);
    hold(ax_pitch, 'on');
    grid(ax_pitch, 'on');
    title(ax_pitch, '末端Pitch角度', 'FontSize', 10, 'FontWeight', 'bold');
    xlabel(ax_pitch, '位姿序号', 'FontSize', 9);
    ylabel(ax_pitch, 'Pitch (°)', 'FontSize', 9);
    h_pitch = plot(ax_pitch, nan, nan, '-o', 'Color', [0.9290 0.6940 0.1250], ...
                   'LineWidth', 2, 'MarkerSize', 5, 'MarkerFaceColor', [0.9290 0.6940 0.1250]);
    xlim(ax_pitch, [1, num_poses]);
    
    % 图4: Yaw角度
    ax_yaw = subplot('Position', [0.62, 0.05, 0.35, 0.18]);
    hold(ax_yaw, 'on');
    grid(ax_yaw, 'on');
    title(ax_yaw, '末端Yaw角度', 'FontSize', 10, 'FontWeight', 'bold');
    xlabel(ax_yaw, '位姿序号', 'FontSize', 9);
    ylabel(ax_yaw, 'Yaw (°)', 'FontSize', 9);
    h_yaw = plot(ax_yaw, nan, nan, '-o', 'Color', [0.4940 0.1840 0.5560], ...
                 'LineWidth', 2, 'MarkerSize', 5, 'MarkerFaceColor', [0.4940 0.1840 0.5560]);
    xlim(ax_yaw, [1, num_poses]);

    % 预计算所有位姿的RPY角度（用于设置Y轴范围）
    all_rolls = zeros(num_poses, 1);
    all_pitches = zeros(num_poses, 1);
    all_yaws = zeros(num_poses, 1);
    [alpha, a, d, ~, ~, ~, ~, ~, ~] = puma560_dh();
    
    for i = 1:num_poses
        theta = trajectory_solutions(i, :);
        T_end = calculateForwardKinematics(alpha, a, d, theta);
        [roll, pitch, yaw] = extractRPY(T_end);
        all_rolls(i) = roll * 180/pi;
        all_pitches(i) = pitch * 180/pi;
        all_yaws(i) = yaw * 180/pi;
    end
    
    % 设置Y轴范围
    ylim(ax_roll, [-180, 180]);
    ylim(ax_pitch, [-180, 180]);
    ylim(ax_yaw, [-180, 180]);

    % 控件
    % uicontrol('Style', 'pushbutton', 'String', '暂停/继续', ...
    %           'Position', [20, 20, 100, 30], 'Callback', @togglePause);
    % uicontrol('Style', 'pushbutton', 'String', '重新开始', ...
    %           'Position', [130, 20, 100, 30], 'Callback', @restart);
    info_text = uicontrol('Style', 'text', ...
                         'Position', [250, 20, 600, 30], ...
                         'FontSize', 11, 'FontWeight', 'bold', ...
                         'BackgroundColor', get(fig, 'Color'));

    % 全局控制
    global isPaused isRestart;
    isPaused = false;
    isRestart = false;

    gif_delay = 0.4;
    current_pose = 1;
    
    % 存储历史数据
    pose_indices = [];
    joint_angles_history = zeros(0, 6);
    roll_history = [];
    pitch_history = [];
    yaw_history = [];

    while true
        if isRestart
            current_pose = 1;
            pose_indices = [];
            joint_angles_history = zeros(0, 6);
            roll_history = [];
            pitch_history = [];
            yaw_history = [];
            isRestart = false;
        end
        if ~ishandle(fig), break; end
        if ~isPaused
            theta = trajectory_solutions(current_pose, :);
            current_target_pose = trajectory_poses(:,:,current_pose);
            current_target_pos = current_target_pose(1:3,4);

            % 更新3D机械臂
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
                    T_end_matrix = T_end.T;
                else
                    end_effector_pos = T_end(1:3,4);
                    T_end_matrix = T_end;
                end
            catch
                T_end_matrix = calculateForwardKinematics(alpha, a, d, theta);
                end_effector_pos = T_end_matrix(1:3,4);
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

            title(ax_3d, sprintf('PUMA560轨迹 %d/%d', current_pose, num_poses), ...
                  'FontSize',11,'FontWeight','bold');
            
            % ========== 更新2D图表 ==========
            % 添加当前数据点
            pose_indices(end+1) = current_pose;
            joint_angles_history(end+1, :) = theta * 180/pi;
            
            % 提取RPY角度
            [roll, pitch, yaw] = extractRPY(T_end_matrix);
            roll_history(end+1) = roll * 180/pi;
            pitch_history(end+1) = pitch * 180/pi;
            yaw_history(end+1) = yaw * 180/pi;
            
            % 更新关节角度图
            for i = 1:6
                set(h_joint_lines(i), 'XData', pose_indices, 'YData', joint_angles_history(:, i));
            end
            
            % 更新RPY图
            set(h_roll, 'XData', pose_indices, 'YData', roll_history);
            set(h_pitch, 'XData', pose_indices, 'YData', pitch_history);
            set(h_yaw, 'XData', pose_indices, 'YData', yaw_history);
            
            % 更新信息文本
            set(info_text,'String',sprintf('位姿 %d/%d | 目标 [%.2f, %.2f, %.2f] | RPY: [%.1f°, %.1f°, %.1f°]', ...
                current_pose, num_poses, current_target_pos, roll*180/pi, pitch*180/pi, yaw*180/pi));

            drawnow;

            % 保存GIF帧
            if save_gif && current_pose >= 1
                frame = getframe(fig);
                im = frame2im(frame);
                [imind, cm] = rgb2ind(im, 256);
                if current_pose == 1
                    imwrite(imind, cm, gif_filename, 'gif', 'Loopcount', inf, 'DelayTime', gif_delay);
                    fprintf('开始保存GIF: %s\n', gif_filename);
                else
                    imwrite(imind, cm, gif_filename, 'gif', 'WriteMode', 'append', 'DelayTime', gif_delay);
                end
                if current_pose == num_poses
                    fprintf('GIF保存完成: %s\n', gif_filename);
                end
            end

            current_pose = current_pose + 1;
            if current_pose > num_poses, break; end
        end

        pause(0.1);
        if ~ishandle(fig), return; end
        if isPaused || isRestart, continue; end
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

function T = calculateForwardKinematics(alpha, a, d, theta)
% 计算正运动学（返回末端执行器变换矩阵）
    T = eye(4);
    for i = 1:6
        T_i = [cos(theta(i)), -sin(theta(i))*cos(alpha(i)), sin(theta(i))*sin(alpha(i)), a(i)*cos(theta(i));
               sin(theta(i)), cos(theta(i))*cos(alpha(i)), -cos(theta(i))*sin(alpha(i)), a(i)*sin(theta(i));
               0, sin(alpha(i)), cos(alpha(i)), d(i);
               0, 0, 0, 1];
        T = T * T_i;
    end
end

function end_pos = calculateEndEffectorPosition(alpha, a, d, theta)
% 计算末端执行器位置
    T = calculateForwardKinematics(alpha, a, d, theta);
    end_pos = T(1:3, 4);
end