function main()
    close all;
    clc;

    seesawKind       = 2;
    regs             = struct;
    regs.pinvTol     = 1e-5;
    regs.pinvDamp    = 1e-2;
    
    seesaw           = struct;
    seesaw.h         = 0.1;
    seesaw.R         = 0.362;
    seesaw.delta     = seesaw.R - 0.5*seesaw.h;
    seesaw.inertia   = diag([7.6698599e-02, 3.7876787e-02, 1.0893139e-01]);
    seesaw.mass      = 4.2;
    seesaw.top       = seesaw.delta - (seesaw.R - seesaw.h) ;

    switch seesawKind
        case 1 %Spherical seesaw
            seesaw.iota      = seesaw.mass*inv(seesaw.inertia);
            seesaw.invIota   = inv(seesaw.iota);
        case 2 %Semi cylidrical seesaw
            seesaw.iota      = [1;0;0]*[1;0;0]'*seesaw.mass*inv(seesaw.inertia);
            seesaw.invIota   = 0;
    end
    
    model.seesaw = seesaw;
   
    quatIntegrationGain = 1;
       
    theta = 0/180*pi;
    
    wbm_modelInitialise('icubGazeboSim');
    [qj,~,~,~] = wholeBodyModel('get-state');
    model.robot.dofs = size(qj, 1);
    %%just get the size
%     robotPosition = zeros(size(qj, 1) + 7, 1);
    robotPosition =[ zeros(7,1);
                     0.00       % Torso
                     0.00
                     0.00
                    -0.61       % Left arm
                     0.51
                     0.00
                     0.87
                     0.00
                    -0.61       % Right arm
                     0.51
                     0.00
                     0.87
                     0.00
                     0.20       % Left leg
                     0.08
                     0.00
                    -0.17
                    -0.04
                    -0.08
                     0.20       % Right arm
                     0.08
                     0.00
                    -0.17
                    -0.04
                    -0.08];
    robotPosition(1:3) = [0;0;seesaw.R - seesaw.delta + seesaw.top];
    robotPosition(4:7) = quaternionFromEulerRotation(theta, [1; 0; 0]);
    robotVelocity      = zeros(size(qj, 1) + 6, 1);
    
    wbm_setWorldFrame(rotationFromQuaternion(robotPosition(4:7)), zeros(3,1), [0;0;-9.81]);
    wbm_updateState(robotPosition(8:end), robotVelocity(7:end), robotVelocity(1:6));
    solePos = wbm_forwardKinematics('l_sole');
    robotPosition(3) = robotPosition(3) - solePos(3);
    model.robot.lFootCentreDistance = solePos(2);
    model.robot.rFootCentreDistance = -solePos(2);
    
   
    tspan = [0 2];
    % Orientation is:
    % quaternion for the orientation
    % R^3 for the velocity (angular)
    %integrator state:
    %seesaw position
    %robot position
    %seesaw velocity
    %robot velocity
    w_Q_s = quaternionFromEulerRotation(theta, [1; 0; 0]);
    Rvec = -seesaw.R * [0;0;1]; 
    DeltaVec = seesaw.delta * rotationFromQuaternion(w_Q_s) * [0;0;-1];
    r_w = DeltaVec - Rvec;
    
    x0 = state_merger([r_w;
                       w_Q_s], ...
                      zeros(6,1), ...
                        robotPosition, ...
                        robotVelocity, ...
                        model.robot.dofs);
     
    % x0(1) = -10 / 180 * pi;
    % x0(5) = -10 / 180 * pi;
    lastTime = 0;

    wbm_setWorldFrame(rotationFromQuaternion(robotPosition(4:7)), robotPosition(1:3), [0;0;-9.81]);
    wbm_updateState(robotPosition(8:end), robotVelocity(7:end), robotVelocity(1:6));
    
    %initial com
    ctrlRefsAndGains.references.xcomDes = wbm_forwardKinematics('com');
    ctrlRefsAndGains.references.qDes = robotPosition(8:end);
    ctrlRefsAndGains.gains.posturalProp  = diag([60   60    10 ...
                                                  8    8     8      12     10 ...
                                                  8    8     8      12     10 ...
                                                 35   20    30     350     55   0,...
                                                 35   20    30     350     55   0]);
    
    ctrlRefsAndGains.references.DxcomDes  = zeros(3,1);
    ctrlRefsAndGains.references.DDxcomDes = zeros(3,1);
    ctrlRefsAndGains.gains.omegaGain    = 0;
    ctrlRefsAndGains.gains.xcomPGain    = 50 ;
    ctrlRefsAndGains.gains.xcomDGain    = 2*sqrt(ctrlRefsAndGains.gains.xcomPGain)*0;
    ctrlRefsAndGains.gains.Hw           = 1;
    ctrlRefsAndGains.gains.onSeesaw     = 1;
    ctrlRefsAndGains.gains.ssawControl  = 0;
    % options = odeset('MaxStep',1e-3);
    [T, Y] = ode15s(@(t, x) dyn(t, x), tspan, x0);%, options);
    figure()
    
    angles = zeros(length(T), 3);
    robAngles = zeros(length(T), 6);
    left_hand_pos = zeros(length(T), 3);
    right_hand_pos = zeros(length(T), 3);
    left_foot_pos = zeros(length(T), 3);
    right_foot_pos = zeros(length(T), 3);
    seesaw_com_pos = zeros(length(T), 3);
    
%     vp = zeros(length(T), 3);
%     vc = zeros(length(T), 3);
    for i = 1 : size(angles, 1)
        [ seesaw_pos, seesaw_vel, robot_pos, robot_velocity] = state_partitioning(Y(i,:), model.robot.dofs);
        [angles(i,1), angles(i,2), angles(i,3)] = rpyFromRotation(rotationFromQuaternion(seesaw_pos(4:7)));
        angles(i, :) = angles(i, :) .* 180/pi;

        [robAngles(i,4), robAngles(i,5), robAngles(i,6)] = rpyFromRotation(rotationFromQuaternion(robot_pos(4:7)));
        robAngles(i, :) = robAngles(i, :) .* 180/pi;
        robAngles(i,1:3) = robot_pos(1:3);
        
        
        
        %%Plot robot parts
        wbm_setWorldFrame(rotationFromQuaternion(robot_pos(4:7)), robot_pos(1:3)', [0;0;-9.81]);
        wbm_updateState(robot_pos(8:end)', robot_velocity(7:end)', robot_velocity(1:6)');
        
        temp = wbm_forwardKinematics('l_sole');
        left_foot_pos(i,:) = temp(1:3);
        temp = wbm_forwardKinematics('r_sole');
        right_foot_pos(i,:) = temp(1:3);
        temp = wbm_forwardKinematics('l_hand');
        left_hand_pos(i,:) = temp(1:3);
        temp = wbm_forwardKinematics('r_hand');
        right_hand_pos(i,:) = temp(1:3);
        
%         e3 = [0;0;1];
%         r_w = -seesaw.R * e3 + seesaw.delta * rotationFromQuaternion(Y(i, 1:4)) * e3;
%         vp(i,:) = Y(i, 11:13)' + S(Y(i, 8:10)) * r_w;
%         
%         vc(i,:) = S(Y(i,8:10)) * [0;0;seesaw.R] - ...
%          (Y(i,11:13)' + S(Y(i,8:10)) * seesaw.delta * rotationFromQuaternion(Y(i, 1:4)) * e3);
    end
%     plot(T, Y(:,4:7)');
%     title('Seesaw orientation (Quaternion)');
%     figure()
    plot(T, angles);
    title('Seesaw orientation (e1, e2, e3)');
    figure();    
    plot(T, Y(:,1:3)');
    title('Seesaw lin position (e1, e2, e3)');
    figure();    
    plot(T, robAngles(:,1:3)');
    title('Base lin (e1, e2, e3)');
%     figure();    
%     plot(T, robAngles(:,4:6)');
%     title('Base orientation (e1, e2, e3)');
%     
    figure();
    subplot(2,1,1)
    plot(T, left_foot_pos);
    title('left foot position');
    subplot(2,1,2)
    plot(T, right_foot_pos);
    title('right foot position');
%     figure();
%     plot(T, left_hand_pos);
%     title('left hand position');
%     figure();
%     plot(T, right_hand_pos);
%     title('right hand position');
    
    
%     figure();
%     plot(T, Y(:,8:10) .* (180/pi));
%     
%     figure();
%     plot(T, Y(:,5:7));
%     
%     figure();
%     plot(T, Y(:,11:13));
% 
%     figure();
%     plot(T, vp);
%     
%     figure();
%     plot(T, vc);
    
    function dx = dyn(t, x)
        clc
        fprintf('%f[s]\n', t);
        if (floor(t) > lastTime) 
            lastTime = floor(t);
            fprintf('%f[s]\n', lastTime);
        end
        
        %get state
        [ seesaw_position, seesaw_vel, robot_position, robot_vel] = state_partitioning(x, model.robot.dofs);
        robot_base_rotation = robot_position(4:7);
        robot_base_omega = robot_vel(4:6);
        seesaw_vel(4:6) = seesaw_vel(4:6);
        
        %update robot state
        wbm_setWorldFrame(rotationFromQuaternion(robot_base_rotation), robot_position(1:3), [0;0;-9.81]);
        wbm_updateState(robot_position(8:end), robot_vel(7:end), robot_vel(1:6));
        
        JdotNu = zeros(12, 1);
        J = zeros(12, model.robot.dofs + 6);
        JdotNu(1:6) = wbm_djdq('l_sole');
        JdotNu(7:end) = wbm_djdq('r_sole');
        J(1:6,:) = wbm_jacobian('l_sole');
        J(7:end,:) = wbm_jacobian('r_sole');
        J_CoM      = wbm_jacobian('com');
        Mass = wbm_massMatrix();
        genBiasForces = wbm_generalisedBiasForces();
        robotCoM = wbm_forwardKinematics('com');
        robotLeftFoot  = wbm_forwardKinematics('l_sole');
        robotRightFoot = wbm_forwardKinematics('r_sole');
        H =  wbm_centroidalMomentum();

        robotWBM.Mass = Mass;
        robotWBM.J = J;
        robotWBM.JdotNu = JdotNu;
        robotWBM.genBiasForces = genBiasForces;
        robotWBM.fwdkin.com = robotCoM;
        robotWBM.fwdkin.l_sole = robotLeftFoot;
        robotWBM.fwdkin.r_sole = robotRightFoot;
        
        control = balancingControl(t, x,J_CoM,H, ctrlRefsAndGains, model, robotWBM,regs);

%         Mass = wbm_massMatrix();
%         Mass = Mass(7:end, 7:end);
%         genBiasForces = wbm_generalisedBiasForces();
%         genBiasForces = genBiasForces(7:end);
        
%         control = genBiasForces;
%         control = zeros(25, 1);
        
        
        acc = robot_and_seesaw_dyn(t, x, control, model);
        
        omega_seesaw = rotationFromQuaternion(seesaw_position(4:7))' * seesaw_vel(4:6);
        robot_base_omega = rotationFromQuaternion(robot_base_rotation)' * robot_base_omega;
        dx = state_merger([seesaw_vel(1:3);
                           quaternionDerivative(seesaw_position(4:7), omega_seesaw, quatIntegrationGain)],...
                          acc(1:6), ...
                          [ robot_vel(1:3); ...
                              quaternionDerivative(robot_base_rotation, robot_base_omega, quatIntegrationGain);...
                              robot_vel(7:end)],...
                          acc(7:end),...
                          model.robot.dofs);
        
    end
    
end

%%old code
function simpleSeesawDyn
    seesaw = struct;
    seesaw.R = 1;
    seesaw.delta = 2/3 * seesaw.R;
    seesaw.inertia = eye(3);
    seesaw.mass = 1;
    seesaw.top = seesaw.R * 1/3 * 1/2;
    
    model.seesaw = seesaw;
    model.robot.lFootCentreDistance = 0.15;
    model.robot.rFootCentreDistance = 0.15;

    quatIntegrationGain = 1;
    w_R_s = eye(3);
    omega_w = zeros(3,1);
    wrench_ext = zeros(6 ,1);
    
    tspan = [0 30];
    % Orientation is:
    % quaternion for the orientation
    % R^3 for the velocity (angular)
    x0 = zeros(7 + 6, 1);
    theta = 10/180*pi;
    x0(1:4) = quaternionFromEulerRotation(theta, [1; 0; 0]);
    x0(5:7) = [0; ...
                seesaw.delta * sin(theta) - seesaw.R * theta; ...
                seesaw.R - seesaw.delta * cos(theta)];
    % x0(1) = -10 / 180 * pi;
    % x0(5) = -10 / 180 * pi;
    lastTime = 0;

    % options = odeset('MaxStep',1e-3);
    [T, Y] = ode113(@(t, x) dyn(t, x), tspan, x0);%, options);
    figure()
    
    angles = zeros(length(T), 3);
    vp = zeros(length(T), 3);
    vc = zeros(length(T), 3);
    for i = 1 : size(angles, 1)
        [angles(i,1), angles(i,2), angles(i,3)] = rpyFromRotation(rotationFromQuaternion(Y(i, 1:4)));
        angles(i, :) = angles(i, :) .* 180/pi;
        
        e3 = [0;0;1];
        r_w = -seesaw.R * e3 + seesaw.delta * rotationFromQuaternion(Y(i, 1:4)) * e3;
        vp(i,:) = Y(i, 11:13)' + S(Y(i, 8:10)) * r_w;
        
        vc(i,:) = S(Y(i,8:10)) * [0;0;seesaw.R] - ...
         (Y(i,11:13)' + S(Y(i,8:10)) * seesaw.delta * rotationFromQuaternion(Y(i, 1:4)) * e3);
    end
    plot(T, angles);
    figure();
    plot(T, Y(:,8:10) .* (180/pi));
    
    figure();
    plot(T, Y(:,5:7));
    
    figure();
    plot(T, Y(:,11:13));

    figure();
    plot(T, vp);
    
    figure();
    plot(T, vc);
    

    function dx = dyn(t, x)

        if (floor(t) > lastTime) 
            lastTime = floor(t);
%             disp(lastTime);
        end
        w_R_s = rotationFromQuaternion(x(1:4));
        omega_w = x(8:10);
        
        lin_pos = x(5:7);
        lin_vel = x(11:13);
        
        [acc, ~] = seesaw_dyn(w_R_s, omega_w, wrench_ext, seesaw, lin_vel, lin_pos);
        dx = [quaternionDerivative(x(1:4), omega_w, quatIntegrationGain); ...
              lin_vel; ...
              acc(4:6);  ...
              acc(1:3)];
    end 
end