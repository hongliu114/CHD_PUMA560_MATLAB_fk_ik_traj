% 正运动学多组数据测试程序
function fk_2()
    
    % 清屏初始化
    clc;
    fprintf('=================================================\n');
    fprintf('六连杆机械臂多组随机姿态正运动学测试\n');
    fprintf('=================================================\n\n');
    
    % D-H参数定义
    [alpha,a,d,qlim1,qlim2,qlim3,qlim4,qlim5,qlim6] = puma560_dh();
    qlim = [qlim1;qlim2;qlim3;qlim4;qlim5;qlim6]; % 组合关节限位
    
    % 显示D-H参数
    fprintf('机械臂D-H参数:\n');
    fprintf('α = [%s] 弧度\n', strjoin(arrayfun(@(x) sprintf('%.3f', x), alpha, 'UniformOutput', false), ', '));
    fprintf('a = [%s] m\n', strjoin(arrayfun(@(x) sprintf('%.3f', x), a, 'UniformOutput', false), ', '));
    fprintf('d = [%s] m\n\n', strjoin(arrayfun(@(x) sprintf('%.3f', x), d, 'UniformOutput', false), ', '));
    
    % 生成15组随机关节角度
    fprintf('正在生成15组随机关节角度...\n');
    theta_groups = zeros(15, 6);
    for i = 1:15
        for j = 1:6
            % 在关节限位范围内随机生成角度
            theta_groups(i, j) = qlim(j, 1) + (qlim(j, 2) - qlim(j, 1)) * rand();
        end
    end
    fprintf('随机角度生成完成！\n\n');
    
    % 开始交互式展示
    fprintf('开始展示15组随机姿态：\n');
    fprintf('按回车键查看下一组，按q键退出展示\n\n');
    
    % 初始化统计变量
    all_positions = zeros(15, 3);
    
    for group_idx = 1:15
        % 获取当前组的theta值
        theta = theta_groups(group_idx, :);
        
        % 显示当前组信息
        fprintf('=== 第%d组/15：关节角度数据 ===\n', group_idx);
        fprintf('θ = [%s] 弧度\n', strjoin(arrayfun(@(x) sprintf('%7.3f', x), theta, 'UniformOutput', false), ', '));
        fprintf('θ = [%s] 度\n\n', strjoin(arrayfun(@(x) sprintf('%7.1f', rad2deg(x)), theta, 'UniformOutput', false), ', '));
        
        % 计算正运动学
        T_end = forwardKinematics(alpha, a, d, theta);
        [pos, rot] = extractPose(T_end);
        all_positions(group_idx, :) = pos;
        
        % 显示计算结果
        fprintf('--- 正运动学计算结果 ---\n');
        fprintf('末端齐次变换矩阵 T_end:\n');
        for i = 1:4
            fprintf('[ ');
            for j = 1:4
                fprintf('%8.3f ', T_end(i,j));
            end
            fprintf(']\n');
        end
        fprintf('\n');
        
        fprintf('末端位置: [%8.3f, %8.3f, %8.3f] m\n', pos(1), pos(2), pos(3));
        fprintf('\n末端旋转矩阵:\n');
        for i = 1:3
            fprintf('[ ');
            for j = 1:3
                fprintf('%8.4f ', rot(i,j));
            end
            fprintf(']\n');
        end
        fprintf('\n');
        
        % 计算并显示欧拉角
        eul_angles = rotm2eul(rot, 'ZYZ');
        fprintf('欧拉角 (ZYZ约定):\n');
        fprintf('  绕Z轴: %6.1f°\n', rad2deg(eul_angles(1)));
        fprintf('  绕Y轴: %6.1f°\n', rad2deg(eul_angles(2)));
        fprintf('  绕Z轴: %6.1f°\n', rad2deg(eul_angles(3)));
        fprintf('\n');
        
        % 3D可视化
        fprintf('--- 启动3D可视化 ---\n');
        
        % 创建或更新图形窗口
        if group_idx == 1
            figure('Name', '机械臂多组随机姿态可视化', 'NumberTitle', 'off', ...
                   'Position', [100, 100, 1200, 800]);
        else
            clf; % 清除当前图形，保留窗口
        end
        
        % 调用可视化函数
        visualizeRobot(alpha, a, d, theta, ...
            'frame_scale', 0.15, ...
            'link_style', '-bo', ...
            'joint_size', 160);
        
        % 添加标题和组信息
        title(sprintf('PUMA560机械臂 - 第%d组随机姿态 (共15组)', group_idx), ...
              'FontSize', 14, 'FontWeight', 'bold');
        
        fprintf('可视化图形已更新！\n\n');
        
        % 等待用户输入（最后一组不等待）
        if group_idx < 15
            user_input = input('按回车键继续下一组，或输入q退出: ', 's');
            
            if isempty(user_input)
                % 清屏准备下一组
                clc;
                fprintf('准备显示第%d组数据...\n\n', group_idx + 1);
            elseif lower(user_input) == 'q'
                fprintf('\n用户请求退出展示。\n');
                break;
            else
                clc;
                fprintf('继续展示...\n\n');
            end
        else
            fprintf('所有15组数据展示完成！\n\n');
        end
    end
    
    fprintf('\n程序执行完毕！\n');
end