function [velocity, acceleration] = calculateVelocityAcceleration(theta_sequence, duration)
% 计算关节速度和加速度
    num_frames = size(theta_sequence, 1);
    dt = duration / (num_frames - 1);
    
    % 计算速度（一阶差分）
    velocity = diff(theta_sequence) / dt;
    velocity = [velocity; velocity(end,:)]; % 保持相同长度
    
    % 计算加速度（二阶差分）
    acceleration = diff(velocity) / dt;
    acceleration = [acceleration; acceleration(end,:)]; % 保持相同长度
end