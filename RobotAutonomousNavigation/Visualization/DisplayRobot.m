% Visualize the robot

function DisplayRobot(robotState, mScale)

if(nargin<2) mScale = 25; end

% Robot shape is a circle of diameter 1 to be scaled by mScale
robotR = mScale/2;
xRobotOrg = mScale*robotState(1); yRobotOrg = mScale*robotState(2); theta = robotState(3);

ang = 0:(pi/18):(2*pi); ang = [ang, 0];
xRobot = xRobotOrg+robotR*cos(ang); yRobot = yRobotOrg+robotR*sin(ang);

cRobot = [[robotR; -0.1*robotR], [robotR; 0.1*robotR], [-0.6*robotR; 0.1*robotR], [-0.6*robotR; -0.1*robotR], [robotR; -0.1*robotR]]; % principal axis corners
cRobot = [cos(theta), -sin(theta); sin(theta), cos(theta)]*cRobot + [xRobotOrg; yRobotOrg];

hold on; patch(xRobot, yRobot, 'r'); patch(cRobot(1,:), cRobot(2,:), 'k'); hold off; axis equal;

end

