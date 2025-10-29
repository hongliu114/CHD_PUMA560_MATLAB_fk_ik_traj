% 正运动学连续运动演示程序
function fk_3()
    
    % 清屏初始化
    clc;
    fprintf('=================================================\n');
    fprintf('六连杆机械臂连续运动轨迹演示\n');
    fprintf('=================================================\n\n');
    
    % D-H参数定义
    [alpha,a,d,qlim1,qlim2,qlim3,qlim4,qlim5,qlim6] = puma560_dh();
    
    % 显示D-H参数
    fprintf('机械臂D-H参数:\n');
    fprintf('α = [%s] 弧度\n', strjoin(arrayfun(@(x) sprintf('%.3f', x), alpha, 'UniformOutput', false), ', '));
    fprintf('a = [%s] m\n', strjoin(arrayfun(@(x) sprintf('%.3f', x), a, 'UniformOutput', false), ', '));
    fprintf('d = [%s] m\n\n', strjoin(arrayfun(@(x) sprintf('%.3f', x), d, 'UniformOutput', false), ', '));
    
    % 定义连续运动轨迹参数
    fprintf('定义连续运动轨迹...\n');
    [theta_sequence, motion_info] = defineMotionTrajectory();
    
    % 计算速度和加速度
    [velocity, acceleration] = calculateVelocityAcceleration(theta_sequence, motion_info.duration);
    
    % 显示运动轨迹信息
    displayMotionInfo(motion_info);
    
    % 询问是否保存GIF动画
    save_gif = askForGifSave();
    gif_filename = '';
    if save_gif
        gif_filename = input('请输入GIF文件名（不含扩展名）: ', 's');
        if isempty(gif_filename)
            gif_filename = 'robot_arm_animation';
        end
        gif_filename = [gif_filename, '.gif'];
        fprintf('将保存动画为: %s\n', gif_filename);
    end
    
    % 执行运动演示
    fprintf('\n开始运动演示...\n');
    demonstrateContinuousMotion(alpha, a, d, theta_sequence, velocity, acceleration, save_gif, gif_filename, motion_info);
    
    fprintf('\n运动演示完成！\n');
end

function [theta_sequence, motion_info] = defineMotionTrajectory()
% 定义连续运动轨迹参数
    
    % 运动参数设置
    num_frames = 100; % 总帧数
    motion_info.num_frames = num_frames;
    motion_info.duration = 5; % 运动总时长（秒）
    
    % 定义起始和目标关节角度（6个关节）
    % 起始姿态
    theta_start = [0.5,  -1.0,  0.1, -0.4,  0.1,  0.2];
    % 中间姿态1
    theta_mid1  = [1.0,  -0.5,  0.5,  0.0,  0.3,  1.0];
    % 中间姿态2  
    theta_mid2  = [1.5,   0.0,  1.5,  0.4,  0.5,  0.6];
    % 目标姿态
    theta_end   = [2.0,   0.5,  2.5,  0.8,  0.7,  1.8];
    
    % 生成平滑的运动轨迹（使用五次多项式插值）
    t = linspace(0, 1, num_frames);
    theta_sequence = zeros(num_frames, 6);
    
    % 关键帧
    keyframes = [theta_start; theta_mid1; theta_mid2; theta_end];
    keyframe_times = [0, 0.33, 0.67, 1];
    
    % 对每个关节进行五次多项式插值
    for joint = 1:6
        joint_angles = [keyframes(1,joint), keyframes(2,joint), ...
                       keyframes(3,joint), keyframes(4,joint)];
        
        % 使用pchip插值（保持形状的立方Hermite插值）
        theta_sequence(:, joint) = pchip(keyframe_times, joint_angles, t);
    end
    
    motion_info.theta_start = theta_start;
    motion_info.theta_end = theta_end;
    motion_info.keyframes = keyframes;
    motion_info.keyframe_times = keyframe_times;
end

function displayMotionInfo(motion_info)
% 显示运动轨迹信息
    fprintf('=== 运动轨迹参数 ===\n');
    fprintf('总帧数: %d\n', motion_info.num_frames);
    fprintf('运动时长: %.1f秒\n', motion_info.duration);
    fprintf('帧率: %.1f fps\n', motion_info.num_frames/motion_info.duration);
    
    fprintf('\n起始姿态关节角度（弧度）:\n');
    fprintf('θ_start = [%s]\n', strjoin(arrayfun(@(x) sprintf('%6.3f', x), ...
        motion_info.theta_start, 'UniformOutput', false), ', '));
    
    fprintf('\n目标姿态关节角度（弧度）:\n');
    fprintf('θ_end   = [%s]\n', strjoin(arrayfun(@(x) sprintf('%6.3f', x), ...
        motion_info.theta_end, 'UniformOutput', false), ', '));
    
    fprintf('\n关键帧数量: %d\n', size(motion_info.keyframes, 1));
end
