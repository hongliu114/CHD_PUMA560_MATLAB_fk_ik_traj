function addInteractiveControls()
% addInteractiveControls 添加交互式控制控件

    % % 创建控制面板
    % uipanel('Position', [0.75, 0.1, 0.2, 0.8], 'Title', '控制面板', ...
    %         'BackgroundColor', [0.9, 0.9, 0.9]);
    
    % % 视角控制按钮
    % uicontrol('Style', 'pushbutton', 'String', '顶视图', ...
    %           'Position', [760, 600, 80, 30], ...
    %           'Callback', @(src,evt) view(0, 90));
    % 
    % uicontrol('Style', 'pushbutton', 'String', '前视图', ...
    %           'Position', [760, 560, 80, 30], ...
    %           'Callback', @(src,evt) view(0, 0));
    % 
    % uicontrol('Style', 'pushbutton', 'String', '侧视图', ...
    %           'Position', [760, 520, 80, 30], ...
    %           'Callback', @(src,evt) view(90, 0));
    
    % % 显示/隐藏选项
    % uicontrol('Style', 'checkbox', 'String', '显示坐标系', ...
    %           'Position', [760, 480, 100, 30], 'Value', 1, ...
    %           'Callback', @toggleFrames);
    % 
    % uicontrol('Style', 'checkbox', 'String', '显示网格', ...
    %           'Position', [760, 450, 100, 30], 'Value', 1, ...
    %           'Callback', @toggleGrid);
    
    % % 数据信息显示
    % info_text = uicontrol('Style', 'text', 'String', '机械臂信息', ...
    %                      'Position', [760, 350, 200, 100], ...
    %                      'BackgroundColor', [1, 1, 1], ...
    %                      'HorizontalAlignment', 'left');
end

function toggleFrames(src, ~)
% toggleFrames 切换坐标系显示
    if src.Value
        set(findobj('Type', 'quiver'), 'Visible', 'on');
    else
        set(findobj('Type', 'quiver'), 'Visible', 'off');
    end
end

function toggleGrid(src, ~)
% toggleGrid 切换网格显示
    if src.Value
        grid on;
    else
        grid off;
    end
end