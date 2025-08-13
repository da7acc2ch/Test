% 文件名: tricopter_pid_controller.m (修改后)
function [U, attitude_target] = tricopter_pid_controller(q, p_desired, h_desired, params, integral_e, dt)
    % =================== 我们只保留姿态控制的部分 ===================
    
    % 提取当前姿态和角速度
    current_rpy = q(10:12); % [roll; pitch; yaw]
    current_pqr = q(13:15); % [p; q; r] (机体坐标系下的角速度)

    % 目标姿态：在空中保持水平，所以目标 roll 和 pitch 都是 0
    target_rpy = [0; 0; current_rpy(3)]; % 保持当前偏航角不变

    % 计算姿态误差
    e_phi   = target_rpy(1) - current_rpy(1);
    e_theta = target_rpy(2) - current_rpy(2);
    e_psi   = 0; % 跳跃的飞行阶段，不主动控制偏航

    % 根据PD控制器计算输出力矩
    % 这完全模仿了 virtual_spring 中的姿态控制逻辑
    torque_L = params.Kp_att * e_phi   - params.Kd_att * current_pqr(1);
    torque_M = params.Kp_att * e_theta - params.Kd_att * current_pqr(2);
    torque_N = params.Kp_yaw * e_psi   - params.Kd_yaw * current_pqr(3);

    % =================== 最关键的修改 ===================
    % **不再计算任何用来对抗重力的推力！**
    % 我们告诉控制器，螺旋桨不负责升空，所以总推力设为0。
    force_Fz = 0;
    
    % 输出的U向量，第一个元素（总推力）为0，后面是姿态控制力矩
    U = [force_Fz; torque_L; torque_M; torque_N];

    % attitude_target 这个输出变量已经没用了，但为了保持函数接口不变，可以随便返回一个值
    attitude_target = [0;0;0;0];
end