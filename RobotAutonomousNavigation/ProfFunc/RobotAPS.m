
function [xA, yA] = RobotAPS(robotState)

biasR = 5; noiseD = 1; ang = robotState(3) + random('norm',0,0.1);
xA = robotState(1) + random('norm', biasR*cos(ang), noiseD);
yA = robotState(2) + random('norm', biasR*sin(ang), noiseD);

end %% END RobotRanging
