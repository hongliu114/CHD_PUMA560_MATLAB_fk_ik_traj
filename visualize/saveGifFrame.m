function saveGifFrame(fig, filename, frame, total_frames)
% 保存当前帧到GIF文件
    % 捕获当前帧
    frame_image = getframe(fig);
    im = frame2im(frame_image);
    [imind, cm] = rgb2ind(im, 256);
    
    % 写入GIF文件
    if frame == 1
        % 第一帧：创建文件
        imwrite(imind, cm, filename, 'gif', ...
                'Loopcount', inf, 'DelayTime', 0.05);
    else
        % 后续帧：追加到文件
        imwrite(imind, cm, filename, 'gif', ...
                'WriteMode', 'append', 'DelayTime', 0.05);
    end
    
    % 显示保存进度
    if mod(frame, 10) == 0
        fprintf('保存进度: %d/%d 帧\n', frame, total_frames);
    end
end