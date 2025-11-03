function showtime()
% main - PUMA560机械臂仿真系统主入口
% 功能: 提供交互式菜单界面，可重复执行各项功能

    % 清理环境
    clc;
    close all;
    
    % 全局变量设置
    global visualize_opt;
    visualize_opt = 0;  % 可视化工具选择：0为使用RTB工具箱
    
    % 显示欢迎信息
    fprintf('\n');
    fprintf('========================================================\n');
    fprintf('      PUMA560 机械臂运动学仿真分析系统\n');
    fprintf('========================================================\n');
    pause(0.5);
    
    % 主循环
    while true
        % 显示主菜单
        fprintf('\n');
        fprintf('========================================================\n');
        fprintf('                     主菜单\n');
        fprintf('========================================================\n');
        fprintf('\n');
        fprintf('  1. 打印DH表和变换矩阵T\n');
        fprintf('  2. 随机生成关节角并正运动学展示\n');
        fprintf('  3. 对已知规划进行逆运动学解算和展示\n');
        fprintf('  0. 退出系统\n');
        fprintf('\n');
        fprintf('========================================================\n');
        
        % 获取用户选择
        choice = input('请输入选项 (0-3): ', 's');
        
        fprintf('\n');
        
        % 处理用户选择
        switch choice
            case '1'
                fprintf('>>> 执行：打印DH表和变换矩阵\n\n');
                try
                    dh_1();
                catch ME
                    fprintf('\n错误：%s\n', ME.message);
                end
                fprintf('\n>>> 功能执行完成\n');
                input('\n按 Enter 键返回主菜单...', 's');
                clc;
                
            case '2'
                fprintf('>>> 执行：随机生成关节角并正运动学展示\n\n');
                try
                    fk_2();
                catch ME
                    fprintf('\n错误：%s\n', ME.message);
                end
                fprintf('\n>>> 功能执行完成\n');
                input('\n按 Enter 键返回主菜单...', 's');
                clc;
                
            case '3'
                fprintf('>>> 执行：已知规划的逆运动学解算和展示\n\n');
                try
                    ik_3();
                catch ME
                    fprintf('\n错误：%s\n', ME.message);
                end
                fprintf('\n>>> 功能执行完成\n');
                input('\n按 Enter 键返回主菜单...', 's');
                clc;
                
            case '0'
                fprintf('感谢使用！再见！\n\n');
                break;
                
            case 'q'
                fprintf('感谢使用！再见！\n\n');
                break;
                
            case 'Q'
                fprintf('感谢使用！再见！\n\n');
                break;
                
            otherwise
                fprintf('⚠ 无效选项！请输入 0-3。\n');
                pause(1.5);
                clc;
        end
    end
end