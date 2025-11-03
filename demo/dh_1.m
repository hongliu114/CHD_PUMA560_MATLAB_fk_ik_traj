function dh_1()
% dh_1 - 显示PUMA560机械臂DH参数表和末端变换矩阵
% 功能: 调用puma560_dh函数获取DH参数，并以表格形式打印到命令行
%       计算并显示末端的齐次变换矩阵

    % 调用puma560_dh函数获取DH参数
    [alpha, a, d, ~, ~, ~, ~, ~, ~] = puma560_dh();
    
    % 打印标题
    fprintf('\n');
    fprintf('=================================================================\n');
    fprintf('                PUMA560 机械臂 DH 参数表                        \n');
    fprintf('=================================================================\n\n');
    
    % 打印表头
    fprintf('+--------+-------------+-------------+-------------+-------------+\n');
    fprintf('| 关节号 |   alpha     |   alpha     |      a      |      d      |\n');
    fprintf('|   i    |   (rad)     |    (deg)    |     (m)     |     (m)     |\n');
    fprintf('+--------+-------------+-------------+-------------+-------------+\n');
    
    % 打印每个关节的DH参数
    for i = 1:6
        alpha_deg = alpha(i) * 180 / pi;  % 转换为度
        fprintf('|   %d    | %11.6f | %11.4f | %11.4f | %11.4f |\n', ...
                i, alpha(i), alpha_deg, a(i), d(i));
    end
    
    % 打印表尾
    fprintf('+--------+-------------+-------------+-------------+-------------+\n\n');
    
    % ========== 计算并打印末端变换矩阵 ==========
    fprintf('=================================================================\n');
    fprintf('                  末端齐次变换矩阵计算                           \n');
    fprintf('=================================================================\n\n');
    
    % 询问用户是否输入关节角度
    fprintf('请选择关节角度输入方式:\n');
    fprintf('  1 - 使用零位 (所有关节角度为0)\n');
    fprintf('  2 - 手动输入关节角度\n');
    choice = input('请输入选择 (1或2，默认为1): ', 's');
    
    if isempty(choice)
        choice = '1';
    end
    
    % 根据选择获取关节角度
    if strcmp(choice, '2')
        fprintf('\n请输入6个关节角度 (单位：度):\n');
        theta = zeros(1, 6);
        for i = 1:6
            theta_deg = input(sprintf('  关节%d角度 (度): ', i));
            theta(i) = theta_deg * pi / 180;  % 转换为弧度
        end
    else
        fprintf('\n使用零位配置 (所有关节角度为0°)\n');
        theta = zeros(1, 6);
    end
    
    % 打印当前关节角度配置
    fprintf('\n当前关节角度配置:\n');
    fprintf('  关节号     角度(rad)     角度(deg)\n');
    fprintf('  ----------------------------------------\n');
    for i = 1:6
        theta_deg = theta(i) * 180 / pi;
        fprintf('    %d      %11.6f    %11.4f\n', i, theta(i), theta_deg);
    end
    fprintf('\n');
    
    % 调用正运动学函数计算末端变换矩阵
    T_end = forwardKinematics(alpha, a, d, theta);
    
    % 打印变换矩阵
    fprintf('末端齐次变换矩阵 T:\n\n');
    fprintf('    ┌                                                      ┐\n');
    fprintf('    │ %9.6f  %9.6f  %9.6f  %9.6f │\n', T_end(1,1), T_end(1,2), T_end(1,3), T_end(1,4));
    fprintf('    │ %9.6f  %9.6f  %9.6f  %9.6f │\n', T_end(2,1), T_end(2,2), T_end(2,3), T_end(2,4));
    fprintf('    │ %9.6f  %9.6f  %9.6f  %9.6f │\n', T_end(3,1), T_end(3,2), T_end(3,3), T_end(3,4));
    fprintf('    │ %9.6f  %9.6f  %9.6f  %9.6f │\n', T_end(4,1), T_end(4,2), T_end(4,3), T_end(4,4));
    fprintf('    └                                                      ┘\n\n');
    
    % 提取并打印位置和姿态信息
    fprintf('末端位置和姿态信息:\n');
    fprintf('  ----------------------------------------\n');
    
    % 提取位置
    position = T_end(1:3, 4);
    fprintf('  位置 (Position):\n');
    fprintf('    X = %.6f m\n', position(1));
    fprintf('    Y = %.6f m\n', position(2));
    fprintf('    Z = %.6f m\n', position(3));
    fprintf('\n');
    
    % 提取旋转矩阵
    R = T_end(1:3, 1:3);
    
    % 计算欧拉角 (ZYX顺序，即Roll-Pitch-Yaw)
    pitch = atan2(-R(3,1), sqrt(R(3,2)^2 + R(3,3)^2));
    if abs(cos(pitch)) > 1e-6
        roll = atan2(R(3,2), R(3,3));
        yaw = atan2(R(2,1), R(1,1));
    else
        roll = 0;
        yaw = atan2(-R(1,2), R(2,2));
    end
    
    fprintf('  姿态 (Orientation - RPY角):\n');
    fprintf('    Roll  = %.6f rad = %.4f°\n', roll, roll*180/pi);
    fprintf('    Pitch = %.6f rad = %.4f°\n', pitch, pitch*180/pi);
    fprintf('    Yaw   = %.6f rad = %.4f°\n', yaw, yaw*180/pi);
    fprintf('\n');
    
    fprintf('=================================================================\n\n');
end