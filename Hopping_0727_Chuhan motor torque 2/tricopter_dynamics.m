function [d_rpy, d_pqr, d_uvw] = tricopter_dynamics(q, motor_thrusts, params)
    phi = q(10); theta = q(11);
    p = q(13); q_rot = q(14); r = q(15);
    u = q(4); v = q(5); w = q(6);
    
    m = params.m; g = params.g;
    Ix = params.Ix; Iy = params.Iy; Iz = params.Iz; Ixz = params.Ixz;
    l = params.l; k_t = params.k_t;
    T1 = motor_thrusts(1); T2 = motor_thrusts(2); T3 = motor_thrusts(3);
    Fz_prop = T1 + T2 + T3;
    L_prop = (T2 - T3) * l * cosd(30);
    
    L_prop = (T2-T3)*l_m*cosd(30);
    M_prop = (-T1 + (T2+T3)*sind(30))*l_m;
    N_prop = k_t*(T2 - T1 - T3);

    c_phi = cos(phi); s_phi = sin(phi); c_th = cos(theta); s_th = sin(theta);
    
    d_rpy = [p + q_rot*s_phi*tan(theta) + r*c_phi*tan(theta);
             q_rot*c_phi - r*s_phi;
             (q_rot*s_phi + r*c_phi)/c_th];
             
    I = [Ix, 0, -Ixz; 0, Iy, 0; -Ixz, 0, Iz];
    omega = [p; q_rot; r];
    tau = [L_prop; M_prop; N_prop];
    omega_dot = I \ (tau - cross(omega, I * omega));
    d_pqr = omega_dot;
    
    acc_body_x = r*v - q_rot*w - g*s_th;
    acc_body_y = p*w - r*u + g*c_th*s_phi;
    acc_body_z = q_rot*u - p*v + g*c_th*c_phi + Fz_prop/m;
    d_uvw = [acc_body_x; acc_body_y; acc_body_z];
end