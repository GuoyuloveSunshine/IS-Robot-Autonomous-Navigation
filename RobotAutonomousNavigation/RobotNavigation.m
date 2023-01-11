clc;
clear;
close all;
%% Robot navigation main script
addpath(genpath("ProfFunc"))
addpath(genpath("PathPlanning"))
addpath(genpath("Matching"))
addpath(genpath("Visualization"))
addpath(genpath("Control"))
%% Simulation preliminary configuration
map = rgb2gray(imread('navigation_map.bmp')); % Import the navigation map
mScale = 25; % Image scaling factor i.e. one physical metric unit corresponds to mScale pixels in the navigation map image 
idx = find(map>128); map(:,:) = 0; map(idx) = 255;
figure(1), imshow(map); set(gca, 'YDir', 'normal');
dt = 0.02; % Numerical computation step
pt_stt = ginput(1) % Select a start point randomly
robotState = [pt_stt(1)/  mScale; pt_stt(2)/mScale; random('uniform', -pi, pi)]%不能用robot state的信息
pt_end = ginput(1) % Select a destination point randomly
disp(["robotState: ",num2str(robotState(1)),num2str(robotState(2)),num2str(robotState(3))])
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% WRITE NAVIGATION PREPROCESSING CODE HERE
%% Path planning
distance_map = DistanceMap(map);
flag = true;
while(flag)
    edges = PlanningRRT(map,pt_stt,pt_end);
    avl_path = FindAvailablePath(edges,pt_stt);
    figure(1), imshow(map); set(gca, 'YDir', 'normal');
    hold on;
    plot(pt_stt(1), pt_stt(2), 'ob', pt_end(1), pt_end(2), 'or', 'MarkerSize', 12, 'LineWidth', 4);
    hold off;
    PlotEdges(avl_path);
    pause;

    opt_path = PathSmoothing(avl_path,distance_map)
    figure(1), imshow(map); set(gca, 'YDir', 'normal');
    hold on;
    plot(pt_stt(1), pt_stt(2), 'ob', pt_end(1), pt_end(2), 'or', 'MarkerSize', 12, 'LineWidth', 4);
    hold off;
    PlotEdges(opt_path);
    flag = IsBadPath(opt_path,distance_map,mScale);
    pause;
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Simulation of robot autonomous navigation
% Hyperparameter
T = 100;
count = 1;
x_est = zeros(3,T/dt+1);
ErSqrt = 1; Er = ErSqrt^2;
EvSqrt = 0.5; EwSqrt = 0.1; 
Eu = diag([EvSqrt^2; EwSqrt^2]);
point_idx = 2;

for t = 0:dt:T
    
    [xR, yR] = RobotRanging(robotState, map, mScale);

    cPt = ([cos(robotState(3)), -sin(robotState(3))]*[xR;yR]+robotState(1))*mScale;
    rPt = ([sin(robotState(3)), cos(robotState(3))]*[xR;yR]+robotState(2))*mScale;

    [xA, yA] = RobotAPS(robotState);
    cPtA = xA*mScale; 
    rPtA = yA*mScale; % large errors!
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Navigation Process
    %% Matching
    if t == 0
        [xA_rtf,yA_rtf,ang_rtf,cSensorw_rtf,rSensorw_rtf] = Matching(map,distance_map,xA,yA,xR,yR,mScale);
    elseif t*dt == round(t*dt) &&t ~=0
        tmp_idx = 1:2:length(xR);
        xR = xR(tmp_idx);
        yR = yR(tmp_idx);
        [xA_rtf,yA_rtf,ang_rtf,cSensorw_rtf,rSensorw_rtf] = MatchingAfterFirst(map,distance_map,xA_rtf,yA_rtf,ang_rtf,xR,yR,mScale);
    else
        tmp_idx = 1:5:length(xR);
        xR = xR(tmp_idx);
        yR = yR(tmp_idx);
        [xA_rtf,yA_rtf,ang_rtf,cSensorw_rtf,rSensorw_rtf] = MatchingAfterFirst(map,distance_map,xA_rtf,yA_rtf,ang_rtf,xR,yR,mScale);
    end
    disp(["after matching: ",num2str(xA_rtf),num2str(yA_rtf),num2str(ang_rtf)])

    
    %% EKF
    if t==0
        u =[0, 0]; %给的v 和omega
        x_kf = [xA_rtf;yA_rtf;ang_rtf];
        Ex_kf = diag([ErSqrt^2,ErSqrt^2,0.1^2]);
    end
    
    z_msr = zeros(length(cSensorw_rtf));
    for i=1:length(cSensorw_rtf)
        z_msr(i) = sqrt((xA_rtf*mScale-cSensorw_rtf(i)).^2 + (yA_rtf*mScale-rSensorw_rtf(i)).^2);
    end
    disp(["Before predict x_kf: ",num2str(x_kf(1)),num2str(x_kf(2)),num2str(x_kf(3))])
    v_msr = u(1);
    w_msr = u(2);

    % Prediction
    A = [1, 0, -v_msr*dt*sin(x_kf(3)+w_msr*dt/2);
         0, 1,  v_msr*dt*cos(x_kf(3)+w_msr*dt/2);
         0, 0, 1];
    B = [dt*cos(x_kf(3)+w_msr*dt/2), -v_msr*dt^2*sin(x_kf(3)+w_msr*dt/2)/2;
         dt*sin(x_kf(3)+w_msr*dt/2),  v_msr*dt^2*cos(x_kf(3)+w_msr*dt/2)/2;
         0,  dt];
    x_kf(1) = x_kf(1) + v_msr*dt*cos(x_kf(3)+w_msr*dt/2);
    x_kf(2) = x_kf(2) + v_msr*dt*sin(x_kf(3)+w_msr*dt/2);
    x_kf(3) = x_kf(3) + w_msr*dt;
    Ex_kf = A*Ex_kf*transpose(A) + B*Eu*transpose(B);
    disp(["After predict x_kf: ",num2str(x_kf(1)),num2str(x_kf(2)),num2str(x_kf(3))])
    
    % Update
    for j=1:length(cSensorw_rtf)
        xLtmp = cSensorw_rtf(j); yLtmp = rSensorw_rtf(j);
        z_x = sqrt((x_kf(1)*mScale-xLtmp)^2 + (x_kf(2)*mScale-yLtmp)^2);
        H_x = [(x_kf(1)*mScale-xLtmp)/z_x, (x_kf(2)*mScale-yLtmp)/z_x, 0];
        K = Ex_kf*H_x'/(H_x*Ex_kf*H_x'+Er);
        x_kf = x_kf +K*(z_msr(j)-z_x);
        Ex_kf = (eye(length(x_kf))-K*H_x)*Ex_kf;
    end
    x_est(:,count) = x_kf;
    count = count+1;
    disp(["After update x_kf: ",num2str(x_kf(1)),num2str(x_kf(2)),num2str(x_kf(3))])
    
    %% Control
    point_idx = PointCompare([x_kf(1)*mScale;x_kf(2)*mScale],opt_path,point_idx); % The point that the robot aim to reach
%     opt_path(:,point_idx);
    yawrateIn = CalculateOmega(opt_path, point_idx, x_kf, mScale);
    pxl = distance_map(round(x_kf(2)*mScale),round(x_kf(1)*mScale));
    speedIn = CalculateV(point_idx, yawrateIn, pxl,mScale);
    u =[speedIn yawrateIn]; % v and omega
    disp(["pxl: ",num2str(pxl), "u: ",num2str(speedIn),num2str(yawrateIn)])
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Collision checking
    if (IsCollision(robotState, map, mScale)) 
        speedIn = 0; fprintf('Robot state [%f, %f, %f]  Collision happens !! \n', robotState(1), robotState(2), robotState(3)); break;
    end
    if (pdist([[x_kf(1)*mScale,x_kf(2)*mScale];[pt_end(1),pt_end(2)]],"euclidean")) < 3/5*mScale
        speedIn = 0; fprintf('Robot state [%f, %f, %f]  Arrive !! \n', robotState(1), robotState(2), robotState(3)); break;
    end 
    
    % Robot dynamics simulation
    robotState = RobotDynamics(robotState, yawrateIn, speedIn, dt);
    disp("------------------------------------------------------------")
    disp(["robotState: ",num2str(robotState(1)),num2str(robotState(2)),num2str(robotState(3))])
     
    % Robot and map visualization
    figure(1), imshow(map);  set(gca, 'YDir', 'normal'); hold on; plot(pt_stt(1), pt_stt(2), 'ob', pt_end(1), pt_end(2), 'or', 'MarkerSize', 12, 'LineWidth', 4); hold off; 
    
    PlotEdges(opt_path);
    hold on; plot(cPt, rPt, '.r', 'MarkerSize', 12);
    hold on; plot(cPtA, rPtA, '+r', 'MarkerSize', 18, 'LineWidth', 2);

    hold on; plot(xA_rtf*mScale, yA_rtf*mScale,'+g', 'MarkerSize', 18, 'LineWidth', 2);
    DisplayRobot(robotState, mScale);
    hold on; plot(cSensorw_rtf,rSensorw_rtf,'.g','MarkerSize',12);
    hold off;
%     pause;
end
