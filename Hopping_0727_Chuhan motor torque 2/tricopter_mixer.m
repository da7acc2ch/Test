function motor_thrusts = tricopter_mixer(U, params)
    Fz_cmd = U(1); 
    L_cmd = U(2); 
    M_cmd = U(3); 
    N_cmd = U(4);
    l = params.l; 
    k_t = params.k_t;

    U_vec = [Fz_cmd; L_cmd; M_cmd; N_cmd];
    motor_thrusts = params.B_pinv * U_vec; 
    

    motor_thrusts = max(0, motor_thrusts); 
    
    if isfield(params, 'max_motor_thrust')
        motor_thrusts = min(params.max_motor_thrust, motor_thrusts); 
    end
end