function setupPlotProperties(joint_positions)
% setupPlotProperties 设置图形显示属性

    % 设置坐标轴属性
    axis equal;
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title('六连杆机械臂3D可视化', 'FontSize', 14, 'FontWeight', 'bold');
    
    % 自动调整坐标轴范围
    margin = 0.2;
    x_limits = [min(joint_positions(:,1))-margin, max(joint_positions(:,1))+margin];
    y_limits = [min(joint_positions(:,2))-margin, max(joint_positions(:,2))+margin];
    z_limits = [min(joint_positions(:,3))-margin, max(joint_positions(:,3))+margin];
    
    xlim(x_limits); ylim(y_limits); zlim(z_limits);
    
    % 设置视角
    view(45, 30);
    
    % 添加网格和光照
    grid on;
    lighting gouraud;
    light('Position',[1 1 1],'Style','infinite');
    
    % % 添加图例
    % legend('连杆', '关节', 'X轴', 'Y轴', 'Z轴', '末端执行器', ...
    %        'Location', 'northeastoutside');
end