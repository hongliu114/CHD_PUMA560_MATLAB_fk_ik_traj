function plotArmStructure(joint_positions, T_mats, params)
% plotArmStructure 绘制机械臂的3D结构

    hold on;
    grid on;
    
    % 1. 绘制连杆
    for i = 1:6
        % 连接相邻关节
        x_data = [joint_positions(i,1), joint_positions(i+1,1)];
        y_data = [joint_positions(i,2), joint_positions(i+1,2)];
        z_data = [joint_positions(i,3), joint_positions(i+1,3)];
        
        plot3(x_data, y_data, z_data, params.link_style, ...
              'LineWidth', 3, 'Marker', 'none');
    end
    
    % 2. 绘制关节 (球体表示)
    for i = 1:7
        scatter3(joint_positions(i,1), joint_positions(i,2), joint_positions(i,3), ...
                params.joint_size, 'filled', 'MarkerFaceColor', 'r');
        
        % 添加关节标签
        text(joint_positions(i,1), joint_positions(i,2), joint_positions(i,3), ...
             sprintf('J%d', i-1), 'FontSize', 12, 'FontWeight', 'bold');
    end
    
    % 3. 绘制坐标系框架
    if params.frame_scale > 0
        for i = 1:6
            plotCoordinateFrame(T_mats{i}, params.frame_scale, i);
        end
    end
    
    % 4. 绘制末端执行器
    plotEndEffector(T_mats{6});
end

function plotCoordinateFrame(T, scale, joint_num)
% plotCoordinateFrame 在指定位置绘制坐标系框架
    origin = T(1:3, 4);
    
    % 提取旋转矩阵的列向量 (X,Y,Z轴方向)
    x_axis = T(1:3, 1) * scale;
    y_axis = T(1:3, 2) * scale;
    z_axis = T(1:3, 3) * scale;
    
    % 绘制坐标轴
    quiver3(origin(1), origin(2), origin(3), ...
            x_axis(1), x_axis(2), x_axis(3), ...
            'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    quiver3(origin(1), origin(2), origin(3), ...
            y_axis(1), y_axis(2), y_axis(3), ...
            'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    quiver3(origin(1), origin(2), origin(3), ...
            z_axis(1), z_axis(2), z_axis(3), ...
            'b', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    
    % 添加坐标系标签
    text(origin(1)+x_axis(1), origin(2)+x_axis(2), origin(3)+x_axis(3), ...
         'X', 'Color', 'r', 'FontSize', 10, 'FontWeight', 'bold');
    text(origin(1)+y_axis(1), origin(2)+y_axis(2), origin(3)+y_axis(3), ...
         'Y', 'Color', 'g', 'FontSize', 10, 'FontWeight', 'bold');
    text(origin(1)+z_axis(1), origin(2)+z_axis(2), origin(3)+z_axis(3), ...
         'Z', 'Color', 'b', 'FontSize', 10, 'FontWeight', 'bold');
end

function plotEndEffector(T)
% plotEndEffector 绘制末端执行器
    origin = T(1:3, 4);
    
    % 绘制末端工具（简单的几何形状）
    tool_length = 0.05;
    tool_direction = T(1:3, 3) * tool_length; % Z轴方向
    
    % 绘制工具轴线
    plot3([origin(1), origin(1)+tool_direction(1)], ...
          [origin(2), origin(2)+tool_direction(2)], ...
          [origin(3), origin(3)+tool_direction(3)], ...
          'k', 'LineWidth', 4);
    
    % 绘制工具头
    scatter3(origin(1)+tool_direction(1), ...
             origin(2)+tool_direction(2), ...
             origin(3)+tool_direction(3), ...
             80, 'filled', 'MarkerFaceColor', 'm');
end