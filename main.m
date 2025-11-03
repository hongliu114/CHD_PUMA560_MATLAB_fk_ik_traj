%% 
clear;clc;close all

global visualize_opt;

%% Option设置部分
% 在本部分设置后直接F5运行

demo_dh = 0;                            % 打印DH表
demo_forward_one = 0;                   % 单组正运动学解算
demo_forward_sorts = 0;                 % 多组正运动学解算（随机生成）
demo_forward_video = 0;                 % 正运动学运动演示
demo_inverse_one = 0;                   % 单点逆运动学解算
demo_inverse_sort = 0;                  % 多点逆运动学解算
demo_inverse_show = 0;                  % 更完善的多点逆运动学解算
demo_showtime = 1;

% demo模式检查
if demo_dh+demo_forward_one+demo_forward_sorts+demo_forward_video+demo_inverse_one+demo_inverse_sort+demo_inverse_show+demo_showtime > 1
    error('一次只能运行一个选项')
end

% 可视化工具选择：1为自己写的，0为使用RTB工具箱函数（自己写的仅测试了单组正运动学，其他可能有BUG）
visualize_opt = 0;

%% demo演示

if demo_dh
    dh_1;
end
if demo_forward_one
    fk_1;
end
if demo_forward_sorts
    fk_2;
end
if demo_forward_video
    fk_3;
end
if demo_inverse_one
    ik_1;
end
if demo_inverse_sort
    ik_2;
end
if demo_inverse_show
    ik_3;
end

if demo_showtime
    showtime();
end
