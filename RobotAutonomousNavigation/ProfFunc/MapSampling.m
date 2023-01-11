
mScale = 25;

map = rgb2gray(imread('navigation_map.bmp'));
idx = find(map>128);
map(:,:) = 0; map(idx) = 255;

figure(1), imshow(map);  set(gca, 'YDir', 'normal');

sample_flg = 1;
while (sample_flg)
    figure(1);
    robotOrg = ginput(1);
    robotEnd = ginput(1);
    if (robotOrg(1)<0 || robotOrg(1)>size(map,2) || robotOrg(2)<0 || robotOrg(2)>size(map,1))
        sample_flg = 0;
        break;
    end
    theta = atan2(robotEnd(2)-robotOrg(2),robotEnd(1)-robotOrg(1));
    robotState = [robotOrg(1)/mScale; robotOrg(2)/mScale; theta];
    fprintf('Robot state [%f, %f, %f] | Collision status: %d \n', robotState(1), robotState(2), robotState(3), IsCollision(robotState, map, mScale));
    DisplayRobot(robotState, mScale);
    
    [xR, yR] = RobotRanging(robotState, map, mScale);
    figure(2); plot(xR, yR, '.b'); DisplayRobot([0;0;0], 1); grid on; axis equal;
    pause(2);
end

