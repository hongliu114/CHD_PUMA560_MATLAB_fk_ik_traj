%% 
clear;clc;close all

global visualize_opt;

%% Option设置部分

demo_forward_one = 1;                   % 单组正运动学解算

% demo模式检查
if demo_forward_one > 1
    error('一次只能运行一个选项')
end

% 可视化工具选择：1为自己写的，0为使用工具箱函数
visualize_opt = 0;

%% demo演示

if demo_forward_one
    fk_1;
end
