function visualizeRobot(alpha, a, d, theta, varargin)
% visualizeRobot 六连杆机械臂3D可视化函数
% 输入:
%   alpha, a, d, theta: D-H参数
%   varargin: 可选参数 ('animate', 'trajectory', etc.)

    % 解析可选参数
    p = inputParser;
    addParameter(p, 'animate', false, @islogical);
    addParameter(p, 'trajectory', [], @isnumeric);
    addParameter(p, 'frame_scale', 0.1, @isnumeric);
    addParameter(p, 'link_style', '-b', @ischar);
    addParameter(p, 'joint_size', 50, @isnumeric);
    parse(p, varargin{:});
    
    global visualize_opt;
    if visualize_opt
        % 计算所有关节的位置
        [joint_positions, T_mats] = computeAllJointPositions(alpha, a, d, theta);
        % 创建图形窗口
        figure('Name', '六连杆机械臂3D可视化', 'NumberTitle', 'off', ...
               'Position', [100, 100, 1200, 800]);
        % 绘制机械臂
        plotArmStructure(joint_positions, T_mats, p.Results);
        % 设置图形属性
        setupPlotProperties(joint_positions);
        % 添加交互控件
        addInteractiveControls();
    else
        base_xyz = [0 0 0];
        robot_RTB = puma560_RTB(base_xyz);
        robot_RTB.display();
        robot_RTB.plot(theta);
    end
end