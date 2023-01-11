% 
% Dynamics: robot
% robotState = [x; y; theta]

function robotState = RobotDynamics(robotState, yawrateIn, speedIn, dt)

yawrateIn = max(-1, yawrateIn); yawrateIn = min(1, yawrateIn);
speedIn = max(0, speedIn); speedIn = min(5, speedIn);

% Robot pose (position and orientation) evolution
robotState(1) = robotState(1) + speedIn*dt*cos(robotState(3)+0.5*yawrateIn*dt);
robotState(2) = robotState(2) + speedIn*dt*sin(robotState(3)+0.5*yawrateIn*dt);
robotState(3) = robotState(3) + yawrateIn*dt;
% if robotState(3)>pi
%     robotState(3) = robotState(3)-2*pi;
% end
% if robotState(3)<-pi
%     robotState(3) = robotState(3)+2*pi;
% end

end












