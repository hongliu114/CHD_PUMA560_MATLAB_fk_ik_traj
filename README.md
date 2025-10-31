# **CHD_PUMA560_MATLAB_fk_ik_traj**

## 概述-Abstract

本仓库以长安大学《机器人建模与仿真综合实验》-选题**六连杆机械臂运动学建模与分析**为需求，使用MATLAB进行建模和仿真。

本项目中仅使用RTB机器人工具箱进行可视化呈现，不使用工具箱进行正逆运动学解算。

## 功能-Functions

本项目实现的核心需求：

- ☑️已完成-基于PUMA560机械臂结构，通过D-H参数建模机械臂
- ☑️已完成-正运动学解算：输入一组或多组给定角度theta值，解算机械臂末端位姿，并可视化显示。此部分功能呈现为以下三个功能（MATLAB方法）：
  - ```demo_forward_one-fk_1()```：给定一组theta值，正运动学解算并输出；
  - ```demo_forward_sorts-fk_2()```：随机生成15组theta值，正运动学解算并输出；
  - ```demo_forward_video-fk_3()```：给定多组theta值作为路径点，插值计算theta路径，正运动学解算并以动画输出（见fk_continous.gif)；
- ☑️已完成-逆运动学解算：输入一组或多组空间点，解算可达的机械臂角度组，并可视化显示。此部分功能暂定通过下面功能展示：
  - ```demo_inverse_one-ik_1()```：单点逆运动学解算；ik_fk正逆运动学单点验证
  - ```demo_inverse_sort-ik_2()```：多点逆运动学解算：给定多点形成一个轨迹，计算每个点的逆运动学解算，并展示

项目详细需求见[任务书摘要](/docs/任务书摘要.pdf)（已删去无关信息）。

fk_3:

![fk_continous.gif](/fk_continous.gif)

ik_2:

![ik_traj.gif](/ik_traj.gif)

## 文件结构-Files

文件结构及重要文件如下：

```
├── demo/··············# 演示用方法
├── docs/··············# 相关文档
│   └── 任务书摘要.pdf··# 项目需求文档
├── forward_kine/······# 正运动学解算
├── inverse_kine/······# 逆运动学解算
├── robot/·············# 机械臂建模相关方法
├── visualize/·········# 可视化所用方法
├── main.m·············# 程序入口方法
└── README.md··········# 说明文档
```

## 依赖-Dependency

可视化部分使用了RTB机器人工具箱：

- https://github.com/petercorke/robotics-toolbox-matlab

## 参考资料-References

本项目制作过程中主要参考了以下资料：

- 书籍：
  - 《机器人学及其应用导论》- ISBN 978-7-302-56508-6
- 项目/文档：
  - https://github.com/howard789/puma560_trajectory_plan
  - https://github.com/0xBotCrafter/PUMA560-kinematics-and-trajectory_planning

## 致谢-Thanks

- Claude Sonnet 4.0
- Deepseek V3.1
- ChatGPT 4.0

```兄弟AI太好用了```

