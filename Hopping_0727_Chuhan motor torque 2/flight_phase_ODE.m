function dq = flight_phase_ODE(t,q,g,l0,T,p_desired,kr,klp,kpos,h_desired,params,integral_e,dt)
    robot_pos = q(1:3);
    robot_vel = q(4:6);
    leg_tilt = q(7:8);
    robot_orient = q(10:12);
    robot_angvel = q(13:15);

    v_desired = kpos*(p_desired-robot_pos);
    if norm(v_desired)>params.velocityLim
        v_desired = v_desired/norm(v_desired)*params.velocityLim;
    end
    input_asin1 = max(-1.0, min(1.0, (robot_vel(1)*T/2/l0-kr*(v_desired(1)-robot_vel(1)))));
    input_asin2 = max(-1.0, min(1.0, (robot_vel(2)*T/2/l0-kr*(v_desired(2)-robot_vel(2)))));
    leg_tilt_vel = [klp*(asin(input_asin1)-leg_tilt(1)); ...
                    klp*(asin(input_asin2)-leg_tilt(2))];

    relative_foot_body_approx = [-l0*sin(leg_tilt(1));
                                  l0*sin(leg_tilt(2));
                                 -l0*cos(leg_tilt(1))*cos(leg_tilt(2))];
    l0_cos_t1 = l0 * cos(leg_tilt(1));
    l0_cos_t2 = l0 * cos(leg_tilt(2));
    foot_vel_cartesian_body = [-l0_cos_t1 * leg_tilt_vel(1);
                                l0_cos_t2 * leg_tilt_vel(2);
                                l0*sin(leg_tilt(1))*cos(leg_tilt(2))*leg_tilt_vel(1) + l0*cos(leg_tilt(1))*sin(leg_tilt(2))*leg_tilt_vel(2)];
    
    D = params.D; d = params.d; r = params.r;
    J = inverse_jacobian(relative_foot_body_approx, D, d, r);
    actual_joint_vel = J * foot_vel_cartesian_body;
    
    joint_damping_torques = -params.jd * actual_joint_vel;
    
    axis1 = [1; 0; 0];
    axis2 = [-0.5; sqrt(3)/2; 0];
    axis3 = [-0.5; -sqrt(3)/2; 0];
    tau_leg1_on_body = -joint_damping_torques(1) * axis1;
    tau_leg2_on_body = -joint_damping_torques(2) * axis2;
    tau_leg3_on_body = -joint_damping_torques(3) * axis3;
    tau_legs_reaction = tau_leg1_on_body + tau_leg2_on_body + tau_leg3_on_body;

    tau_prop = [0; 0; 0];
    prop_force_world = [0; 0; 0];
    if params.propeller_switch == 1
        q_full = [q(1:8); 0; q(10:15); q(9)];
        [U, ~] = tricopter_pid_controller(q_full, p_desired, h_desired, params, integral_e, dt);
        motor_thrusts = tricopter_mixer(U, params);
        l_m=params.l; k_t=params.k_t;
        T1=motor_thrusts(1); T2=motor_thrusts(2); T3=motor_thrusts(3);
        tau_prop = [(T2 - T3) * l_m * cosd(30); (-T1 + (T2 + T3) * sind(30)) * l_m; k_t * (T2 - T1 - T3)];
        R_bw = eul2rotm(robot_orient', 'ZYX');
        prop_force_world = R_bw * [0; 0; sum(motor_thrusts)];
    end

    dq = zeros(15, 1);
    dq(1:3) = robot_vel;
    dq(4:6) = [0; 0; -g] + prop_force_world / params.m;
    dq(7:8) = leg_tilt_vel;
    phi = robot_orient(1); theta = robot_orient(2);
    W_inv = [1, sin(phi)*tan(theta), cos(phi)*tan(theta); 0, cos(phi), -sin(phi); 0, sin(phi)/cos(theta), cos(phi)/cos(theta)];
    dq(10:12) = W_inv * robot_angvel;
    I = params.I_matrix;
    omega = robot_angvel;
    tau_total = tau_legs_reaction + tau_prop;
    dq(13:15) = I \ (tau_total - cross(omega, I * omega));
    dq(9) = 0;
end