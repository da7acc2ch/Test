clc; clear; close;
v=VideoWriter('sim.avi');
fprintf('\n001...\n')
%% =============================PARAMETERS=============================
l0 = 0.46; % spring resting length
m_robot = 2.73; % robot body mass
g = 9.81; %% gravity
h_desired = 0.55; % desired height
v_desired = [1;0;0]; % desired velocity
p_desired = [8;0;0]; % desired position
num_of_hop =15;
mode = '3D';
params.propeller_switch = 0; % 1 on; 0 off

params.random_force_magnitude = 0.2;
params.m_propeller = 0.5; % propeller mass
params.m = m_robot + params.m_propeller;
m = params.m;
params.g = g; 
params.Ix = 0.008;
params.Iy = 0.008;
params.Iz = 0.004;
params.Ixy = 0;
params.Ixz = 0;
params.Iyz = 0;
params.l = 1;
params.k_t = 0.000001;
params.max_motor_thrust = 3;
params.I_matrix = [  params.Ix,    -params.Ixy,  -params.Ixz;
                    -params.Ixy,   params.Iy,   -params.Iyz;
                    -params.Ixz,  -params.Iyz,   params.Iz  ];
B = [ 1,      1,                 1;
    0,      params.l*cosd(30), -params.l*cosd(30);
    -params.l, params.l*sind(30),  params.l*sind(30);
    -params.k_t, params.k_t,      -params.k_t];
params.B_pinv = pinv(B);
params.D = 0.158;
params.d = 0.398;
params.r = 0.02424;
params.upLim = 80*pi/180;
params.lowLim = -60*pi/180;
params.velocityLim = 1;
%% =============================PID SETTLE=============================
params.Kp_pos = 0;
params.Kd_pos = 0;
params.Kp_att = 5;
params.Ki_att = 0;
params.Kd_att = 1;
params.Kp_yaw = 5.5;
params.Ki_yaw = 0;
params.Kd_yaw = 1;
params.Kp_alt = 0;
params.Ki_alt = 0;
params.Kd_alt = 0;
params.integral_max = 5;
integral_e.phi = 0;
integral_e.theta = 0;
integral_e.psi = 0;
integral_e.z = 0;
dt = 0.01;
params.jd = 0.005;
%% =============================PATH DEFINE=============================
% num_hops_per_stage = 6;
% num_of_hop = num_hops_per_stage * 3;
% max_hopping = 100;
%%
% P0 = [0; 0; 0];
% P1 = [0; 0; 0];
% P2 = [0; 0; 0];
% P3 = [4; 0; 0];
% points = {P0, P1, P2, P3};
% arrival_error = 0.5; % m
% hop_length = 0.2;
%%
% P_start = [0; 0; 0];
% P_target_inplace = [0; 0; 0];
% points = {P_start, P_target_inplace}
%%


%% robot gains
ke = 5; k_spring = 800; b_spring = 10; kp = 500; kd = 1;  %%500>>>
klp = 10; kld = 1; kpos = 0.3; kr = 0.05;
%% floor gains
kf = 10000; bf = 0.1;
%% Simulation ODE
tspan = [0,1.5];
data = zeros([21,num_of_hop*1000]);
dataNum = 1;
totalTime = 0;
Xdoffset = 0;
q0 = [0;0;0.35; ... % body global position
    0;0;0; ... % body velocity
    0;0;0; ... % foot global position
    0;0;0; ... % orientation
    0;0;0; ... % angular velocity
    2];

%% =============================MAIN LOOP=============================
% for j = 2:length(points)
%     current_p = points{j};
%     hop_count = 0;
% while norm(q0(1:2) - current_p(1:2)) > arrival_error % q0(1:2) current coordinates
%     hop_count = hop_count + 1;
%     p_desired = current_p;
%
% while hop_count < max_hopping
%
%     hop_count = hop_count + 1;
%     p_desired = current_p;
for j = 1:num_of_hop
    Opt  = odeset('Events', @termination_liftoff);
    integral_e.phi   = 0;
    integral_e.theta = 0;
    integral_e.psi   = 0;
    integral_e.z     = 0;
    [t,q]=ode45(@(t,q) stance_phase_ODE(t,q,m,g,k_spring,l0,p_desired,h_desired,ke,kf,bf,kpos,kp,kd,params,integral_e,dt),tspan,q0,Opt);
    for i = 1:length(t)
        t(i) = t(i)+totalTime;
        if(t(i)>=0.001*dataNum)
            data(1,dataNum) = t(i);
            data(2,dataNum) = q(i,1); %% x
            data(3,dataNum) = q(i,2); %% y
            data(4,dataNum) = q(i,3); %% z
            data(5,dataNum) = q(i,4); %% xdot
            data(6,dataNum) = q(i,5); %% ydot
            data(7,dataNum) = q(i,6); %% zdot
            data(8,dataNum) = q(i,7); %% foot pos x from body
            data(9,dataNum) = q(i,8); %% foot pos y from body
            data(10,dataNum) = q(i,9); %% foot pos z from body
            data(11,dataNum) = q(i,10); %% orient
            data(12,dataNum) = q(i,11); %% orient
            data(13,dataNum) = q(i,12); %% orient
            data(14,dataNum) = q(i,13); %% angular velocity
            data(15,dataNum) = q(i,14); %% angular velocity
            data(16,dataNum) = q(i,15); %% angular velocity
            % stance energy = mgh+1/2mv^2+1/2k(l-l0)^2
            data(17,dataNum) = m*g*data(4,dataNum)+1/2*m*(data(5,dataNum)^2+data(6,dataNum)^2+data(7,dataNum)^2) ...
                +1/2*k_spring*(norm([data(2,dataNum);data(3,dataNum);data(4,dataNum)]-[data(8,dataNum);data(9,dataNum);data(10,dataNum)])-l0)^2;
            data(18,dataNum) = q(i,16);
            %% =============================MOTOR THRAUSTS=============================
            [U_from_pid, ~] = tricopter_pid_controller(q(i,:)', p_desired, h_desired, params, integral_e, dt);
            U_final = U_from_pid;
            motor_thrusts_real = tricopter_mixer(U_final, params);
            data(19:21,dataNum) = motor_thrusts_real;
            dataNum = dataNum+1;
            % =======================================================================
        end
    end
    disp("stance phase end at "); disp(data(1,dataNum-1));
    totalTime = t(length(t));
    T = t(length(t))-t(1);

    q0 = [data(2,dataNum-1); data(3,dataNum-1); data(4,dataNum-1);data(5,dataNum-1);data(6,dataNum-1);data(7,dataNum-1); ...
        atan((data(8,dataNum-1)-data(2,dataNum-1))/(data(4,dataNum-1)-data(10,dataNum-1))); ...
        atan((data(9,dataNum-1)-data(3,dataNum-1))/(data(4,dataNum-1)-data(10,dataNum-1))); ...
        1; data(11,dataNum-1);data(12,dataNum-1);data(13,dataNum-1); ...
        data(14,dataNum-1);data(15,dataNum-1);data(16,dataNum-1)];
    Opt2 = odeset('Events', @termination_touchdown);
    integral_e.phi   = 0;
    integral_e.theta = 0;
    integral_e.psi   = 0;
    integral_e.z     = 0;

    [t2,q2] = ode45(@(t2,q2) flight_phase_ODE(t2,q2,g,l0,T,p_desired+Xdoffset,kr,klp,kpos,h_desired,params,integral_e,dt),tspan,q0(1:15),Opt2);
    for i = 1:length(t2)
        t2(i) = t2(i)+totalTime;
        if(t2(i)>=0.001*dataNum)
            data(1,dataNum) = t2(i);
            data(2,dataNum) = q2(i,1);
            data(3,dataNum) = q2(i,2);
            data(4,dataNum) = q2(i,3);
            data(5,dataNum) = q2(i,4);
            data(6,dataNum) = q2(i,5);
            data(7,dataNum) = q2(i,6);
            data(8,dataNum) = q2(i,1)+l0*sin(q2(i,7));
            data(9,dataNum) = q2(i,2)+l0*sin(q2(i,8));
            data(10,dataNum) = data(4,dataNum)-sqrt(l0^2-(data(2,dataNum)-data(8,dataNum))^2-(data(3,dataNum)-data(9,dataNum))^2);
            data(11,dataNum) = q2(i,10); %% orient
            data(12,dataNum) = q2(i,11); %% orient
            data(13,dataNum) = q2(i,12); %% orient
            data(14,dataNum) = q2(i,13); %% angular velocity
            data(15,dataNum) = q2(i,14); %% angular velocity
            data(16,dataNum) = q2(i,15); %% angular velocity
            % flight energy = mgh+1/2mv^2
            data(17,dataNum) = m*g*data(4,dataNum)+1/2*m*(data(5,dataNum)^2+data(6,dataNum)^2+data(7,dataNum)^2);
            data(18,dataNum) = q2(i,9);
            %% =============================MOTOR THRAUSTS=============================
            q_current = q2(i,:)';
            q_full = [q_current(1:8); 0; q_current(10:12); q_current(13:15); q_current(9)];
            [U_from_pid, ~] = tricopter_pid_controller(q_full, p_desired, h_desired, params, integral_e, dt);
            U_final = U_from_pid;
            motor_thrusts = tricopter_mixer(U_final, params);
            data(19:21,dataNum) = motor_thrusts;
            dataNum = dataNum+1;
        end
    end
    disp("flight phase end at "); disp(data(1,dataNum-1));
    totalTime = t2(length(t2));

    q0 = [data(2,dataNum-1);data(3,dataNum-1);data(4,dataNum-1); ...
        data(5,dataNum-1);data(6,dataNum-1);data(7,dataNum-1); ...
        data(8,dataNum-1);data(9,dataNum-1);data(10,dataNum-1); ...
        data(11,dataNum-1);data(12,dataNum-1);data(13,dataNum-1); ...
        data(14,dataNum-1);data(15,dataNum-1);data(16,dataNum-1);2];
    % if hop_count > max_hopping
    %     disp('\nE001...\n');
    %     break;
end


fprintf('\n002...\n');


%% =============================DATA PLOT=============================


%% data to plot
D = 0.158;
d = 0.398;
r = 0.02424;
upLim = 80*pi/180;
lowLim = -60*pi/180;
robot_pos = zeros(3,dataNum-1);
foot_pos = zeros(3,dataNum-1);
robot_orient = zeros(3,3,dataNum-1);
for i=1:dataNum-1
    robot_orient(:,:,i) = eye(3);
end
joint_pos = zeros(3,dataNum-1);
joint = zeros(3,3,dataNum-1);
knee = zeros(3,3,dataNum-1);
up_leg = zeros(3,2);
low_leg = zeros(3,2);
triangle = zeros(3,3,dataNum-1);
k = 1;
stance_shade_vertices = zeros(8,num_of_hop);
stance_shade_vertices(1,k) = 0;
stance_shade_vertices(4,k) = 0;
foot_vel = zeros(3,dataNum-1);
foot_force = zeros(3,dataNum-1);
joint_torque = zeros(3,dataNum-1);
joint_vel = zeros(3,dataNum-1);

% 2D
arm_length = 1;
theta = linspace(0,2*pi,100);
circle_x = (arm_length+0.2)*cos(theta);
circle_y = (arm_length+0.2)*sin(theta);

for i = 1:dataNum-1
    % 2D
    if strcmp(mode,'2D')
        yaw_angle = data(2,i)/arm_length;
        while abs(yaw_angle)>=2*pi
            if yaw_angle>0
                yaw_angle = yaw_angle-2*pi;
            else
                yaw_angle = yaw_angle+2*pi;
            end
        end
        robot_orient(:,:,i) = [cos(yaw_angle) -sin(yaw_angle) 0; sin(yaw_angle) cos(yaw_angle) 0; 0 0 1];
        robot_pos(:,i) = robot_orient(:,:,i)*[0;arm_length;data(4,i)];
        relative_foot = robot_orient(:,:,i)*([data(2,i);data(3,i);data(4,i)]-[data(8,i);data(9,i);data(10,i)]);
        foot_yaw_angle = data(8,i)/arm_length;
        while abs(foot_yaw_angle)>=2*pi
            if foot_yaw_angle>0
                foot_yaw_angle = foot_yaw_angle-2*pi;
            else
                foot_yaw_angle = foot_yaw_angle+2*pi;
            end
        end
        foot_yaw_rotate = [cos(foot_yaw_angle) -sin(foot_yaw_angle) 0; sin(foot_yaw_angle) cos(foot_yaw_angle) 0; 0 0 1];
        foot_z = data(10,i);
        if foot_z<0
            foot_z = 0;
        end
        foot_pos(:,i) = foot_yaw_rotate*[0;arm_length;foot_z];
        [joint_pos(:,i),check,noSol] = inverse_kinematics(relative_foot,D,d,r,upLim,lowLim);
        phi = data(11, i);% roll
        theta = data(12, i);% pitch
        psi = data(13, i);% yaw
        Rx = [1, 0, 0; 0, cos(phi), -sin(phi); 0, sin(phi), cos(phi)];
        Ry = [cos(theta), 0, sin(theta); 0, 1, 0; -sin(theta), 0, cos(theta)];
        Rz = [cos(psi), -sin(psi), 0; sin(psi), cos(psi), 0; 0, 0, 1];
        robot_orient(:,:,i) = Rz * Ry * Rx;
        for j=1:3
            R = [cosd(120*(j-1)) -sind(120*(j-1)) 0; sind(120*(j-1)) cosd(120*(j-1)) 0; 0 0 1];
            joint(:,j,i) = robot_pos(:,i)+robot_orient(:,:,i)*R*[0;r;0];
            knee(:,j,i) = robot_pos(:,i)+robot_orient(:,:,i)*R*([0;r;0]+[0;D*cos(joint_pos(j,i));-D*sin(joint_pos(j,i))]);
        end


    else % 3D
        robot_pos(:,i) = data(2:4,i);
        foot_pos(:,i) = data(8:10,i);
        relative_foot = robot_pos(:,i) - foot_pos(:,i);
        phi = data(11, i);   % Roll
        theta = data(12, i); % Pitch
        psi = data(13, i);   % Yaw
        Rx = [1, 0, 0; 0, cos(phi), -sin(phi); 0, sin(phi), cos(phi)];
        Ry = [cos(theta), 0, sin(theta); 0, 1, 0; -sin(theta), 0, cos(theta)];
        Rz = [cos(psi), -sin(psi), 0; sin(psi), cos(psi), 0; 0, 0, 1];
        robot_orient(:,:,i) = Rz * Ry * Rx;
        relative_foot_body_frame = robot_orient(:,:,i)' * relative_foot;
        [joint_pos(:,i),check,noSol] = inverse_kinematics(relative_foot_body_frame,D,d,r,upLim,lowLim);

        for j_leg=1:3
            R_leg = [cosd(120*(j_leg-1)) -sind(120*(j_leg-1)) 0; sind(120*(j_leg-1)) cosd(120*(j_leg-1)) 0; 0 0 1];

            joint_local = R_leg * [0; r; 0];
            knee_local = R_leg * ([0;r;0] + [0; D*cos(joint_pos(j_leg,i)); -D*sin(joint_pos(j_leg,i))]);

            joint(:,j_leg,i) = robot_pos(:,i) + robot_orient(:,:,i) * joint_local;
            knee(:,j_leg,i)  = robot_pos(:,i) + robot_orient(:,:,i) * knee_local;
        end
    end

    if data(18,i) ~= 2 % if at flight phase
        foot_force(:,i) = [0;0;0];
        if (i + 1) <= (dataNum - 1)
            if data(18,i+1) == 2 % touchdown detected
                stance_shade_vertices(1,k) = data(1,i+1);
                stance_shade_vertices(4,k) = data(1,i+1);
            end
        end
    end
    if data(18,i) == 2 % if at stance phase
        foot_force(1,i) = (data(5,i+1)-data(5,i))/(data(1,i+1)-data(1,i))*m; %%%here
        foot_force(2,i) = (data(6,i+1)-data(6,i))/(data(1,i+1)-data(1,i))*m;
        foot_force(3,i) = ((data(7,i+1)-data(7,i))/(data(1,i+1)-data(1,i))+g)*m;
        if data(18,i+1) ~= 2 % liftoff detected
            stance_shade_vertices(2:3,k) = data(1,i);
            stance_shade_vertices(5:6,k) = [-100;-100];
            stance_shade_vertices(7:8,k) = [100;100];
            k = k+1;
        end
    end
    jacob = inverse_jacobian(relative_foot,D,d,r);
    joint_torque(:,i) = inv(jacob')*foot_force(:,i);
end
for i = 1:dataNum-2
    joint_vel(:,i) = (joint_pos(:,i+1)-joint_pos(:,i))/(data(1,i+1)-data(1,i));
end
joint_vel(:,dataNum-1) = joint_vel(:,dataNum-2);

%% video
lastT = 0;
f = figure(1);
f.Units = 'centimeters';
f.InnerPosition =[1,1,7.5*6,4*6];
loops = floor(dataNum/1000/60)+40;
F(loops) = struct('cdata',[],'colormap',[]);
frame = 1;
% f.Visible = 'off';
open(v);

body_polygon_vertice = zeros(3,6);
body_polygon_face = [1 2 3 NaN;4 5 6 NaN;1 3 6 4;1 2 5 4;2 3 6 5];
propeller_r = 0.15;
arm_length = params.l;
propeller_z = 0.02;
propeller_steps = 20;
propeller_angles = linspace(0, 2*pi, propeller_steps);
propeller_x = propeller_r * cos(propeller_angles);
propeller_y = propeller_r * sin(propeller_angles);
% ================================================================= 1.
%

for i = 1:dataNum-1
    clf(f);
    if data(1,i)-lastT>1/30
        disp(strcat("generating video ",num2str(i/dataNum*100),"%"));
        lastT = data(1,i);

        up_leg1 = [joint(1,1,i) knee(1,1,i);joint(2,1,i) knee(2,1,i);joint(3,1,i) knee(3,1,i)];
        up_leg2 = [joint(1,2,i) knee(1,2,i);joint(2,2,i) knee(2,2,i);joint(3,2,i) knee(3,2,i)];
        up_leg3 = [joint(1,3,i) knee(1,3,i);joint(2,3,i) knee(2,3,i);joint(3,3,i) knee(3,3,i)];
        low_leg1 = [knee(1,1,i) foot_pos(1,i);knee(2,1,i) foot_pos(2,i);knee(3,1,i) foot_pos(3,i)];
        low_leg2 = [knee(1,2,i) foot_pos(1,i);knee(2,2,i) foot_pos(2,i);knee(3,2,i) foot_pos(3,i)];
        low_leg3 = [knee(1,3,i) foot_pos(1,i);knee(2,3,i) foot_pos(2,i);knee(3,3,i) foot_pos(3,i)];

        % polyhedron for robot body, triangular prism
        body_h = 0.1;
        % x,y,z coords are 1,2,3 rows, each column is a point
        for j=1:3
            R = [cosd(120*(j-1)) -sind(120*(j-1)) 0; sind(120*(j-1)) cosd(120*(j-1)) 0; 0 0 1];
            body_polygon_vertice(:,j) = R*[0;-0.1;body_h/2];
            body_polygon_vertice(:,j+3) = R*[0;-0.1;-body_h/2];
        end
        body_polygon_vertice = robot_pos(:,i)+robot_orient(:,:,i)*body_polygon_vertice;

        subplot(4,4,[1 2 5 6]);%%%%
        plot_body_pos = plot3(robot_pos(1,1:i),robot_pos(2,1:i),robot_pos(3,1:i),'b');
        hold on
        plot_foot_pos = plot3(foot_pos(1,1:i),foot_pos(2,1:i),foot_pos(3,1:i),'c');
        plot_up_leg1 = plot3(up_leg1(1,:),up_leg1(2,:),up_leg1(3,:),'r','linewidth',3);
        plot_up_leg2 = plot3(up_leg2(1,:),up_leg2(2,:),up_leg2(3,:),'g','linewidth',3);
        plot_up_leg3 = plot3(up_leg3(1,:),up_leg3(2,:),up_leg3(3,:),'b','linewidth',3);
        plot_low_leg1 = plot3(low_leg1(1,:),low_leg1(2,:),low_leg1(3,:),'k','Marker','.','MarkerSize',20,'linewidth',3);
        plot_low_leg2 = plot3(low_leg2(1,:),low_leg2(2,:),low_leg2(3,:),'k','Marker','.','MarkerSize',20,'linewidth',3);
        plot_low_leg3 = plot3(low_leg3(1,:),low_leg3(2,:),low_leg3(3,:),'k','Marker','.','MarkerSize',20,'linewidth',3);
        plot_body_polygon = patch('Faces',body_polygon_face,'Vertices',body_polygon_vertice','FaceColor','red');

        R1 = [cosd(0) -sind(0) 0; sind(0) cosd(0) 0; 0 0 1];
        R2 = [cosd(120) -sind(120) 0; sind(120) cosd(120) 0; 0 0 1];
        R3 = [cosd(240) -sind(240) 0; sind(240) cosd(240) 0; 0 0 1];
        base_direction = [0; -1; 0];

        arm_vectors_local = [R1*base_direction, R2*base_direction, R3*base_direction];

        arm_start_1 = body_polygon_vertice(:,1);
        arm_start_2 = body_polygon_vertice(:,2);
        arm_start_3 = body_polygon_vertice(:,3);

        arm_end_1 = arm_start_1 + robot_orient(:,:,i) * (arm_vectors_local(:,1) * arm_length);
        arm_end_2 = arm_start_2 + robot_orient(:,:,i) * (arm_vectors_local(:,2) * arm_length);
        arm_end_3 = arm_start_3 + robot_orient(:,:,i) * (arm_vectors_local(:,3) * arm_length);

        plot3([arm_start_1(1), arm_end_1(1)], [arm_start_1(2), arm_end_1(2)], [arm_start_1(3), arm_end_1(3)], 'k', 'LineWidth', 2);
        plot3([arm_start_2(1), arm_end_2(1)], [arm_start_2(2), arm_end_2(2)], [arm_start_2(3), arm_end_2(3)], 'k', 'LineWidth', 2);
        plot3([arm_start_3(1), arm_end_3(1)], [arm_start_3(2), arm_end_3(2)], [arm_start_3(3), arm_end_3(3)], 'k', 'LineWidth', 2);

        prop_circle_rotated = robot_orient(:,:,i) * [propeller_x; propeller_y; zeros(1, propeller_steps)];
        patch(arm_end_1(1) + prop_circle_rotated(1,:), ...
            arm_end_1(2) + prop_circle_rotated(2,:), ...
            arm_end_1(3) + prop_circle_rotated(3,:) + propeller_z, ...
            'blue', 'FaceAlpha', 0.5, 'EdgeColor', 'none');

        patch(arm_end_2(1) + prop_circle_rotated(1,:), ...
            arm_end_2(2) + prop_circle_rotated(2,:), ...
            arm_end_2(3) + prop_circle_rotated(3,:) + propeller_z, ...
            'green', 'FaceAlpha', 0.5, 'EdgeColor', 'none');

        patch(arm_end_3(1) + prop_circle_rotated(1,:), ...
            arm_end_3(2) + prop_circle_rotated(2,:), ...
            arm_end_3(3) + prop_circle_rotated(3,:) + propeller_z, ...
            'red', 'FaceAlpha', 0.5, 'EdgeColor', 'none');

        plot_objects = [plot_body_pos,plot_foot_pos,plot_up_leg1,plot_up_leg2,plot_up_leg3,plot_low_leg1,plot_low_leg2,plot_low_leg3,plot_body_polygon];
        if strcmp(mode,'2D')
            plot_fixture = plot3([0 0],[0 0],[0 0.35],'color',[0.8 0.8 0.8],'linewidth',10);
            plot_arm = plot3([0 0.8*robot_pos(1,i)],[0 0.8*robot_pos(2,i)],[0.3 0.3],'color',[0.8 0.8 0.8],'linewidth',5);
            plot_short_arm = plot3([0.8*robot_pos(1,i) robot_pos(1,i)],[0.8*robot_pos(2,i) robot_pos(2,i)],[robot_pos(3,i) robot_pos(3,i)],'color',[0.8 0.8 0.8],'linewidth',5);
            plot_linear_guide = plot3([0.8*robot_pos(1,i) 0.8*robot_pos(1,i)],[0.8*robot_pos(2,i) 0.8*robot_pos(2,i)],[0 1],'color',[0.8 0.8 0.8],'linewidth',5);
            wheel_bar = [0 -1 0;1 0 0;0 0 1]*robot_orient(:,:,i)*[0;0.25;0];
            plot_wheel_bar = plot3([0.8*robot_pos(1,i)+wheel_bar(1) 0.8*robot_pos(1,i)-wheel_bar(1)],[0.8*robot_pos(2,i)+wheel_bar(2) 0.8*robot_pos(2,i)-wheel_bar(2)],[0 0],'color',[0.8 0.8 0.8],'linewidth',5);
            plot_ground = patch(circle_x,circle_y,zeros(1,length(circle_x)),[0.5 0.5 0.5]);
            plot_objects = [plot_objects,plot_fixture,plot_arm,plot_short_arm,plot_linear_guide,plot_wheel_bar,plot_ground];
        end

        xlim([robot_pos(1,i)-2,robot_pos(1,i)+2]);
        ylim([robot_pos(2,i)-2,robot_pos(2,i)+2]);
        zlim([0,0.9]);
        pbaspect([2 2 1]);
        if strcmp(mode,'2D')
            xlim([-arm_length-0.2,arm_length+0.2]);
            ylim([-arm_length-0.2,arm_length+0.2]);
            zlim([0,arm_length+0.2]);
            pbaspect([2 2 1]);
        end
        grid on
        xlabel('X(m)');
        ylabel('Y(m)');
        zlabel('Z(m)');

        current_roll_rad = data(11, i);
        current_pitch_rad = data(12, i);
        current_yaw_rad = data(13, i);
        current_roll_deg = current_roll_rad * 180/pi;
        current_pitch_deg = current_pitch_rad * 180/pi;
        current_yaw_deg = current_yaw_rad * 180/pi;
        display_title = sprintf('Time: %.2f s | Roll: %.1f | Pitch: %.1f | Yaw: %.1f (deg)', ...
            data(1,i), current_roll_deg, current_pitch_deg, current_yaw_deg);
        title(display_title);
        hold off

        % dashboard========================================================
        % % ax_side = subplot(3,4,[9 10]);
        % %
        % % motor_thrusts_for_plot = data(19:21, i);
        % % bar(ax_side, motor_thrusts_for_plot, 'FaceColor', 'flat');
        % % ax_side.Children.CData(1,:) = [0 0 1];
        % % ax_side.Children.CData(2,:) = [0 1 0];
        % % ax_side.Children.CData(3,:) = [1 0 0];
        % %
        % % title(ax_side, 'Motor Thrusts (Real)');
        % % ylabel(ax_side, 'Thrust (N)');
        % % set(ax_side, 'XTickLabel', {'Front', 'Rear-L', 'Rear-R'});
        % % ylim(ax_side, [0, params.max_motor_thrust + 0.5]);
        % % grid on;
        % =================================================================
        ax_side = subplot(4,4,[9 10]);
        copyobj(plot_objects,ax_side);
        view(ax_side,[0, 1, 0]);
        xlim(ax_side, [robot_pos(1,i)-2,robot_pos(1,i)+2]);
        ylim(ax_side, [robot_pos(2,i)-2,robot_pos(2,i)+2]);
        zlim(ax_side, [0,0.9]);
        pbaspect(ax_side, [2 2 1]);
        grid(ax_side, 'on');
        xlabel(ax_side, 'X(m)');
        ylabel(ax_side, 'Y(m)');
        zlabel(ax_side, 'Z(m)');
        title(ax_side, 'Side View (X-Z Plane)');
        ax_front = subplot(4,4,[13 14]);
        copyobj(plot_objects,ax_front);
        view(ax_front,[1, 0, 0]);
        xlim(ax_front, [robot_pos(1,i)-2,robot_pos(1,i)+2]);
        ylim(ax_front, [robot_pos(2,i)-2,robot_pos(2,i)+2]);
        zlim(ax_front, [0,0.9]);
        pbaspect(ax_front, [2 2 1]);
        grid(ax_front, 'on');
        xlabel(ax_front, 'X(m)');
        ylabel(ax_front, 'Y(m)');
        zlabel(ax_front, 'Z(m)');
        title(ax_front, 'Front View (Y-Z Plane)');
        stance_shade = zeros(8,num_of_hop);
        for j=1:num_of_hop
            if data(1,i)<stance_shade_vertices(1,j) % in flight phase
                stance_shade(1:8,1:j-1) = stance_shade_vertices(1:8,1:j-1);
                break;
            elseif data(1,i)<stance_shade_vertices(2,j) % in stance phase
                stance_shade(1:8,1:j) = stance_shade_vertices(1:8,1:j);
                stance_shade(2:3,j) = data(1,i);
                break;
            else
                if j == num_of_hop
                    stance_shade = stance_shade_vertices;
                end
            end
        end
        subplot(4,4,3);
        patch(stance_shade(1:4,:),stance_shade(5:8,:),[0.5 0.5 0.5],'FaceAlpha',0.3,'EdgeColor','none');
        hold on
        plot(data(1,1:i),joint_pos(1,1:i),'r','linewidth',2);
        plot(data(1,1:i),joint_pos(2,1:i),'g','linewidth',2);
        plot(data(1,1:i),joint_pos(3,1:i),'b','linewidth',2);
        hold off
        xlabel('Time (s)');
        ylabel('Joint Position (rad)');
        title('Joint Position');
        xlim([0,data(1,dataNum-1)]);
        ylim([-pi/2,pi/2]);

        smooth_joint_vel = zeros(3,length(joint_vel));
        smooth_joint_vel(:,1:i) = smoothdata(joint_vel(:,1:i));
        subplot(4,4,7);
        patch(stance_shade(1:4,:),stance_shade(5:8,:),[0.5 0.5 0.5],'FaceAlpha',0.3,'EdgeColor','none');
        hold on
        plot(data(1,isfinite(smooth_joint_vel(1,1:i))),smooth_joint_vel(1,isfinite(smooth_joint_vel(1,1:i))),'r','linewidth',2);
        plot(data(1,isfinite(smooth_joint_vel(2,1:i))),smooth_joint_vel(2,isfinite(smooth_joint_vel(2,1:i))),'g','linewidth',2);
        plot(data(1,isfinite(smooth_joint_vel(3,1:i))),smooth_joint_vel(3,isfinite(smooth_joint_vel(3,1:i))),'b','linewidth',2);
        hold off
        xlabel('Time (s)');
        ylabel('Joint Velocity (rad/s)');
        title('Joint Velocity');
        xlim([0,data(1,dataNum-1)]);
        ylim([-pi*4,pi*4]);

        subplot(4,4,11);
        patch(stance_shade(1:4,:),stance_shade(5:8,:),[0.5 0.5 0.5],'FaceAlpha',0.3,'EdgeColor','none');
        hold on
        plot(data(1,1:i),joint_torque(1,1:i),'r','linewidth',2);
        plot(data(1,1:i),joint_torque(2,1:i),'g','linewidth',2);
        plot(data(1,1:i),joint_torque(3,1:i),'b','linewidth',2);
        hold off
        xlabel('Time (s)');
        ylabel('Joint Torque (Nm)');
        title('Joint Toruqe');
        xlim([0,data(1,dataNum-1)]);
        ylim([-10 10]);

        subplot(4,4,4);
        hold on
        plot(smooth_joint_vel(1,i)*30/pi,joint_torque(1,i),'r','marker','o');
        plot(smooth_joint_vel(1,isfinite(smooth_joint_vel(1,1:i)))*30/pi,joint_torque(1,isfinite(smooth_joint_vel(1,1:i))),'r','linewidth',2);
        plot([280 160],[0 9],'--r','linewidth',2);
        plot([-280 -160],[0 9],'--r','linewidth',2);
        plot([280 160],[0 -9],'--r','linewidth',2);
        plot([-280 -160],[0 -9],'--r','linewidth',2);
        plot([-160 160],[9 9],'--r','linewidth',2);
        plot([-160 160],[-9 -9],'--r','linewidth',2);
        hold off
        xlabel('Joint1 Velocity (RPM)');
        ylabel('Joint1 Torque (Nm)');
        title('Joint1 Toruqe against Velocity');
        xlim([-300,300]);
        ylim([-10 10]);

        subplot(4,4,8);
        hold on
        plot(smooth_joint_vel(2,i)*30/pi,joint_torque(2,i),'g','marker','o');
        plot(smooth_joint_vel(2,isfinite(smooth_joint_vel(2,1:i)))*30/pi,joint_torque(2,isfinite(smooth_joint_vel(2,1:i))),'g','linewidth',2);
        plot([280 160],[0 9],'--g','linewidth',2);
        plot([-280 -160],[0 9],'--g','linewidth',2);
        plot([280 160],[0 -9],'--g','linewidth',2);
        plot([-280 -160],[0 -9],'--g','linewidth',2);
        plot([-160 160],[9 9],'--g','linewidth',2);
        plot([-160 160],[-9 -9],'--g','linewidth',2);
        hold off
        xlabel('Joint2 Velocity (RPM)');
        ylabel('Joint2 Torque (Nm)');
        title('Joint2 Toruqe against Velocity');
        xlim([-300,300]);
        ylim([-10 10]);

        subplot(4,4,12);
        hold on
        plot(smooth_joint_vel(3,i)*30/pi,joint_torque(3,i),'b','marker','o');
        plot(smooth_joint_vel(3,isfinite(smooth_joint_vel(3,1:i)))*30/pi,joint_torque(3,isfinite(smooth_joint_vel(3,1:i))),'b','linewidth',2);
        plot([280 160],[0 9],'--b','linewidth',2);
        plot([-280 -160],[0 9],'--b','linewidth',2);
        plot([280 160],[0 -9],'--b','linewidth',2);
        plot([-280 -160],[0 -9],'--b','linewidth',2);
        plot([-160 160],[9 9],'--b','linewidth',2);
        plot([-160 160],[-9 -9],'--b','linewidth',2);
        hold off
        xlabel('Joint3 Velocity (RPM)');
        ylabel('Joint3 Torque (Nm)');
        title('Joint3 Toruqe against Velocity');
        xlim([-300,300]);
        ylim([-10 10]);
        subplot(4,4,15);
        patch(stance_shade(1:4,:),stance_shade(5:8,:),[0.5 0.5 0.5],'FaceAlpha',0.3,'EdgeColor','none');
        hold on
        plot(data(1,1:i),data(19,1:i),'r','linewidth',2);
        plot(data(1,1:i),data(20,1:i),'g','linewidth',2);
        plot(data(1,1:i),data(21,1:i),'b','linewidth',2);
        hold off
        xlabel('Time (s)');
        ylabel('Motor Thrust (N)');
        title('Motor Thrusts');
        xlim([0,data(1,dataNum-1)]);
        ylim([0, params.max_motor_thrust + 0.5]);
        grid on;
        % =======================================================================
        subplot(4,4,16);
        vx = data(5, 1:i);
        vy = data(6, 1:i);
        speed_xy = sqrt(vx.^2 + vy.^2);
        patch(stance_shade(1:4,:),stance_shade(5:8,:),[0.5 0.5 0.5],'FaceAlpha',0.3,'EdgeColor','none'); hold on;
        plot(data(1,1:i), speed_xy, 'm', 'linewidth', 2);
        hold off;
        title('Horizontal Speed (X-Y)');
        xlabel('Time (s)');
        ylabel('Speed (m/s)');
        grid on;
        xlim([0,data(1,dataNum-1)]);
        if i > 1
            ylim([0, max(sqrt(data(5,1:dataNum-1).^2 + data(6,1:dataNum-1).^2)) + 0.5]);
        end

        F(frame)= getframe(f);
        writeVideo(v,F(frame));
        frame = frame+1;
        % ======================================================================================
    end
end

close(v);






figure(1);
subplot(3,4,[1 2 5 6 9 10]);
i = dataNum-1;

plot_body_pos = plot3(robot_pos(1,1:i),robot_pos(2,1:i),robot_pos(3,1:i),'b');
hold on
plot_foot_pos = plot3(foot_pos(1,1:i),foot_pos(2,1:i),foot_pos(3,1:i),'c');
plot_up_leg1 = plot3(up_leg1(1,:),up_leg1(2,:),up_leg1(3,:),'k','linewidth',3);
plot_up_leg2 = plot3(up_leg2(1,:),up_leg2(2,:),up_leg2(3,:),'k','linewidth',3);
plot_up_leg3 = plot3(up_leg3(1,:),up_leg3(2,:),up_leg3(3,:),'k','linewidth',3);
plot_low_leg1 = plot3(low_leg1(1,:),low_leg1(2,:),low_leg1(3,:),'k','Marker','.','MarkerSize',20,'linewidth',3);
plot_low_leg2 = plot3(low_leg2(1,:),low_leg2(2,:),low_leg2(3,:),'k','Marker','.','MarkerSize',20,'linewidth',3);
plot_low_leg3 = plot3(low_leg3(1,:),low_leg3(2,:),low_leg3(3,:),'k','Marker','.','MarkerSize',20,'linewidth',3);
plot_body_polygon = patch('Faces',body_polygon_face,'Vertices',body_polygon_vertice','FaceColor','red');
plot_objects = [plot_body_pos,plot_foot_pos,plot_up_leg1,plot_up_leg2,plot_up_leg3,plot_low_leg1,plot_low_leg2,plot_low_leg3,plot_body_polygon];
plot3([arm_start_1(1), arm_end_1(1)], [arm_start_1(2), arm_end_1(2)], [arm_start_1(3), arm_end_1(3)], 'k', 'LineWidth', 2);
plot3([arm_start_2(1), arm_end_2(1)], [arm_start_2(2), arm_end_2(2)], [arm_start_2(3), arm_end_2(3)], 'k', 'LineWidth', 2);
plot3([arm_start_3(1), arm_end_3(1)], [arm_start_3(2), arm_end_3(2)], [arm_start_3(3), arm_end_3(3)], 'k', 'LineWidth', 2);
prop_circle_rotated = robot_orient(:,:,i) * [propeller_x; propeller_y; zeros(1, propeller_steps)];
patch(arm_end_1(1) + prop_circle_rotated(1,:), ...
    arm_end_1(2) + prop_circle_rotated(2,:), ...
    arm_end_1(3) + prop_circle_rotated(3,:) + propeller_z, ...
    'blue', 'FaceAlpha', 0.5, 'EdgeColor', 'none');

patch(arm_end_2(1) + prop_circle_rotated(1,:), ...
    arm_end_2(2) + prop_circle_rotated(2,:), ...
    arm_end_2(3) + prop_circle_rotated(3,:) + propeller_z, ...
    'green', 'FaceAlpha', 0.5, 'EdgeColor', 'none');

patch(arm_end_3(1) + prop_circle_rotated(1,:), ...
    arm_end_3(2) + prop_circle_rotated(2,:), ...
    arm_end_3(3) + prop_circle_rotated(3,:) + propeller_z, ...
    'red', 'FaceAlpha', 0.5, 'EdgeColor', 'none');


if strcmp(mode,'2D')
    plot_fixture = plot3([0 0],[0 0],[0 0.35],'color',[0.8 0.8 0.8],'linewidth',10);
    plot_arm = plot3([0 0.8*robot_pos(1,i)],[0 0.8*robot_pos(2,i)],[0.3 0.3],'color',[0.8 0.8 0.8],'linewidth',5);
    plot_short_arm = plot3([0.8*robot_pos(1,i) robot_pos(1,i)],[0.8*robot_pos(2,i) robot_pos(2,i)],[robot_pos(3,i) robot_pos(3,i)],'color',[0.8 0.8 0.8],'linewidth',5);
    plot_linear_guide = plot3([0.8*robot_pos(1,i) 0.8*robot_pos(1,i)],[0.8*robot_pos(2,i) 0.8*robot_pos(2,i)],[0 1],'color',[0.8 0.8 0.8],'linewidth',5);
    wheel_bar = [0 -1 0;1 0 0;0 0 1]*robot_orient(:,:,i)*[0;0.25;0];
    plot_wheel_bar = plot3([0.8*robot_pos(1,i)+wheel_bar(1) 0.8*robot_pos(1,i)-wheel_bar(1)],[0.8*robot_pos(2,i)+wheel_bar(2) 0.8*robot_pos(2,i)-wheel_bar(2)],[0 0],'color',[0.8 0.8 0.8],'linewidth',5);
    plot_ground = patch(circle_x,circle_y,zeros(1,length(circle_x)),[0.5 0.5 0.5]);
    plot_objects = [plot_objects,plot_fixture,plot_arm,plot_short_arm,plot_linear_guide,plot_wheel_bar,plot_ground];
end
xlim([robot_pos(1,i)-2,robot_pos(1,i)+2]);
ylim([robot_pos(2,i)-2,robot_pos(2,i)+2]);
zlim([0,0.9]);
pbaspect([2 2 1]);
if strcmp(mode,'2D')
    xlim([-arm_length-0.2,arm_length+0.2]);
    ylim([-arm_length-0.2,arm_length+0.2]);
    zlim([0,arm_length+0.2]);
    pbaspect([2 2 1]);
end
grid on
xlabel('X(m)');
ylabel('Y(m)');
zlabel('Z(m)');
current_roll_rad = data(11, i);
current_pitch_rad = data(12, i);
current_yaw_rad = data(13, i);
current_roll_deg = current_roll_rad * 180/pi;
current_pitch_deg = current_pitch_rad * 180/pi;
current_yaw_deg = current_yaw_rad * 180/pi;
display_title = sprintf('Time: %.2f s | Roll: %.1f | Pitch: %.1f | Yaw: %.1f (deg)', ...
    data(1,i), current_roll_deg, current_pitch_deg, current_yaw_deg);
title(display_title);
hold off

subplot(4,4,3);
patch(stance_shade(1:4,:),stance_shade(5:8,:),[0.5 0.5 0.5],'FaceAlpha',0.3,'EdgeColor','none');
hold on
plot(data(1,1:i),joint_pos(1,1:i),'r','linewidth',2);
plot(data(1,1:i),joint_pos(2,1:i),'g','linewidth',2);
plot(data(1,1:i),joint_pos(3,1:i),'b','linewidth',2);
hold off
xlabel('Time (s)');
ylabel('Joint Position (rad)');
title('Joint Position');
xlim([0,data(1,dataNum-1)]);
ylim([-pi/2,pi/2]);

smooth_joint_vel = zeros(3,length(joint_vel));
smooth_joint_vel(:,1:i) = smoothdata(joint_vel(:,1:i));
subplot(4,4,7);
patch(stance_shade(1:4,:),stance_shade(5:8,:),[0.5 0.5 0.5],'FaceAlpha',0.3,'EdgeColor','none');
hold on
plot(data(1,isfinite(smooth_joint_vel(1,1:i))),smooth_joint_vel(1,isfinite(smooth_joint_vel(1,1:i))),'r','linewidth',2);
plot(data(1,isfinite(smooth_joint_vel(2,1:i))),smooth_joint_vel(2,isfinite(smooth_joint_vel(2,1:i))),'g','linewidth',2);
plot(data(1,isfinite(smooth_joint_vel(3,1:i))),smooth_joint_vel(3,isfinite(smooth_joint_vel(3,1:i))),'b','linewidth',2);
hold off
xlabel('Time (s)');
ylabel('Joint Velocity (rad/s)');
title('Joint Velocity');
xlim([0,data(1,dataNum-1)]);
ylim([-pi*4,pi*4]);

subplot(4,4,11);
patch(stance_shade(1:4,:),stance_shade(5:8,:),[0.5 0.5 0.5],'FaceAlpha',0.3,'EdgeColor','none');
hold on
plot(data(1,1:i),joint_torque(1,1:i),'r','linewidth',2);
plot(data(1,1:i),joint_torque(2,1:i),'g','linewidth',2);
plot(data(1,1:i),joint_torque(3,1:i),'b','linewidth',2);
hold off
xlabel('Time (s)');
ylabel('Joint Torque (Nm)');
title('Joint Toruqe');
xlim([0,data(1,dataNum-1)]);
ylim([-10 10]);

subplot(4,4,4);
hold on
plot(smooth_joint_vel(1,i)*30/pi,joint_torque(1,i),'r','marker','o');
plot(smooth_joint_vel(1,isfinite(smooth_joint_vel(1,1:i)))*30/pi,joint_torque(1,isfinite(smooth_joint_vel(1,1:i))),'r','linewidth',2);
plot([280 160],[0 9],'--r','linewidth',2);
plot([-280 -160],[0 9],'--r','linewidth',2);
plot([280 160],[0 -9],'--r','linewidth',2);
plot([-280 -160],[0 -9],'--r','linewidth',2);
plot([-160 160],[9 9],'--r','linewidth',2);
plot([-160 160],[-9 -9],'--r','linewidth',2);
hold off
xlabel('Joint1 Velocity (RPM)');
ylabel('Joint1 Torque (Nm)');
title('Joint1 Toruqe against Velocity');
xlim([-300,300]);
ylim([-10 10]);

subplot(4,4,8);
hold on
plot(smooth_joint_vel(2,i)*30/pi,joint_torque(2,i),'g','marker','o');
plot(smooth_joint_vel(2,isfinite(smooth_joint_vel(2,1:i)))*30/pi,joint_torque(2,isfinite(smooth_joint_vel(2,1:i))),'g','linewidth',2);
plot([280 160],[0 9],'--g','linewidth',2);
plot([-280 -160],[0 9],'--g','linewidth',2);
plot([280 160],[0 -9],'--g','linewidth',2);
plot([-280 -160],[0 -9],'--g','linewidth',2);
plot([-160 160],[9 9],'--g','linewidth',2);
plot([-160 160],[-9 -9],'--g','linewidth',2);
hold off
xlabel('Joint2 Velocity (RPM)');
ylabel('Joint2 Torque (Nm)');
title('Joint2 Toruqe against Velocity');
xlim([-300,300]);
ylim([-10 10]);

subplot(4,4,12);
hold on
plot(smooth_joint_vel(3,i)*30/pi,joint_torque(3,i),'b','marker','o');
plot(smooth_joint_vel(3,isfinite(smooth_joint_vel(3,1:i)))*30/pi,joint_torque(3,isfinite(smooth_joint_vel(3,1:i))),'b','linewidth',2);
plot([280 160],[0 9],'--b','linewidth',2);
plot([-280 -160],[0 9],'--b','linewidth',2);
plot([280 160],[0 -9],'--b','linewidth',2);
plot([-280 -160],[0 -9],'--b','linewidth',2);
plot([-160 160],[9 9],'--b','linewidth',2);
plot([-160 160],[-9 -9],'--b','linewidth',2);
hold off
xlabel('Joint3 Velocity (RPM)');
ylabel('Joint3 Torque (Nm)');
title('Joint3 Toruqe against Velocity');
xlim([-300,300]);
ylim([-10 10]);
subplot(4,4,15);
patch(stance_shade(1:4,:),stance_shade(5:8,:),[0.5 0.5 0.5],'FaceAlpha',0.3,'EdgeColor','none');
hold on

plot(data(1,1:i),data(19,1:i),'r','linewidth',2);
plot(data(1,1:i),data(20,1:i),'g','linewidth',2);
plot(data(1,1:i),data(21,1:i),'b','linewidth',2);
hold off

xlabel('Time (s)');
ylabel('Motor Thrust (N)');
title('Motor Thrusts');
xlim([0,data(1,dataNum-1)]);
ylim([0, params.max_motor_thrust + 0.5]);
grid on;
% =======================================================================
subplot(4,4,16);
vx = data(5, 1:i);
vy = data(6, 1:i);
speed_xy = sqrt(vx.^2 + vy.^2);
patch(stance_shade(1:4,:),stance_shade(5:8,:),[0.5 0.5 0.5],'FaceAlpha',0.3,'EdgeColor','none'); hold on;
plot(data(1,1:i), speed_xy, 'm', 'linewidth', 2);
hold off;
title('Horizontal Speed (X-Y)');
xlabel('Time (s)');
ylabel('Speed (m/s)');
grid on;
xlim([0,data(1,dataNum-1)]);
if i > 1
    ylim([0, max(sqrt(data(5,1:dataNum-1).^2 + data(6,1:dataNum-1).^2)) + 0.5]);
end

%% ===================== PLOT ====================
% figure(2)
% plot(data(1,1:dataNum-1),data(17,1:dataNum-1));
% title('Energy against Time');
% ylabel('energy(J)');
% xlabel('t(s)');

figure(2)
hold on
p_patch = patch(stance_shade(1:4,:),stance_shade(5:8,:),[0.5 0.5 0.5],'FaceAlpha',0.3,'EdgeColor','none');

p1 = plot(data(1,1:dataNum-1), data(11,1:dataNum-1)*(-180/pi), 'r-', 'LineWidth', 2);
p2 = plot(data(1,1:dataNum-1), data(12,1:dataNum-1)*(-180/pi), 'g-', 'LineWidth', 2);

hold off
xlim([0, data(1, dataNum-1)])
ylim([-20, 20])
title('Roll, Pitch vs. Time');
xlabel('t (s)');
ylabel('Angles (deg)');
grid on;
legend([p1, p2, p_patch], {'Roll', 'Pitch', 'Stance Phase'}, 'Location', 'best', 'Orientation', 'horizontal');


%
% figure(3)
% plot(data(1,1:dataNum-1),data(4,1:dataNum-1));
% hold on
% plot([0,data(1,dataNum-1)],[h_desired,h_desired]);
% title('Vertical Displacement against Time');
% ylabel('Height(m)');
% xlabel('t(s)');
% legend('Z', 'desired height');
%
%
% figure(4)
% plot(data(1,1:dataNum-1),data(5,1:dataNum-1));
% title('Forward Velocity (X-axis) against Time');%%we set the volatile expected velocity.
% ylabel('Velocity (m/s)');
% xlabel('t(s)');
% legend('Actual X-velocity');
%
%
% figure(5)
% plot(data(1,1:dataNum-1),(data(8,1:dataNum-1)-data(2,1:dataNum-1)));
% title('x foot pos from body');
% ylabel('distance(m/s)');
% xlabel('t(s)');
%
% smooth_joint_vel = zeros(3,length(joint_vel));
% smooth_joint_vel(1,1:dataNum-1) = smoothdata(joint_vel(1,1:dataNum-1));
% smooth_joint_vel(2,1:dataNum-1) = smoothdata(joint_vel(2,1:dataNum-1));
% smooth_joint_vel(3,1:dataNum-1) = smoothdata(joint_vel(3,1:dataNum-1));
% figure(6)
% plot(smooth_joint_vel(1,1:dataNum-1)*30/pi,joint_torque(1,1:dataNum-1),'r','linewidth',2);
% hold on
% plot([280 160],[0 9],'--r','linewidth',2);
% plot([-280 -160],[0 9],'--r','linewidth',2);
% plot([280 160],[0 -9],'--r','linewidth',2);
% plot([-280 -160],[0 -9],'--r','linewidth',2);
% plot([-160 160],[9 9],'--r','linewidth',2);
% plot([-160 160],[-9 -9],'--r','linewidth',2);
% hold off
% title('joint torque against joint velocity');
% ylabel('Torque(Nm)');
% xlabel('Velocity(RPM)');
%
% figure(7)
% plot(data(1,1:dataNum-1),joint_pos(1,1:dataNum-1), ...
%      data(1,1:dataNum-1),joint_pos(2,1:dataNum-1), ...
%      data(1,1:dataNum-1),joint_pos(3,1:dataNum-1),'linewidth',2);
% hold on
% patch(stance_shade_vertices(1:4,:),stance_shade_vertices(5:8,:),[0.5 0.5 0.5],'FaceAlpha',0.3,'EdgeColor','none')
% hold off
% title('joint torque');
% ylabel('torque(m/s)');
% xlabel('t(s)');
%
% figure(8)
% plot(data(1,1:dataNum-1),smooth_joint_vel(1,1:dataNum-1), ...
%      data(1,1:dataNum-1),smooth_joint_vel(2,1:dataNum-1), ...
%      data(1,1:dataNum-1),smooth_joint_vel(3,1:dataNum-1),'linewidth',2);
% hold on
% patch(stance_shade_vertices(1:4,:),stance_shade_vertices(5:8,:),[0.5 0.5 0.5],'FaceAlpha',0.3,'EdgeColor','none')
% hold off
% title('joint velocity');
% ylabel('Velocity(m/s)');
% xlabel('t(s)');
%
% figure(9)
% plot(data(1,1:dataNum-1),joint_torque(1,1:dataNum-1), ...
%      data(1,1:dataNum-1),joint_torque(2,1:dataNum-1), ...
%      data(1,1:dataNum-1),joint_torque(3,1:dataNum-1),'linewidth',2);
% hold on
% patch(stance_shade_vertices(1:4,:),stance_shade_vertices(5:8,:),[0.5 0.5 0.5],'FaceAlpha',0.3,'EdgeColor','none')
% hold off
% title('joint torque');
% ylabel('torque(m/s)');
% xlabel('t(s)');
% %
% figure(10)
% plot(data(1,1:dataNum-1),data(5,1:dataNum-1), ...
%      data(1,1:dataNum-1),data(6,1:dataNum-1),'linewidth',2);
% hold on
% patch(stance_shade_vertices(1:4,:),stance_shade_vertices(5:8,:),[0.5 0.5 0.5],'FaceAlpha',0.3,'EdgeColor','none')
% hold off
% title('joint torque');
% ylabel('torque(m/s)');
% xlabel('t(s)');