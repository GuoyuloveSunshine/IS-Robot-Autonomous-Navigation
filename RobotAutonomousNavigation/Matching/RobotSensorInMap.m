
function [xR, yR] = RobotSensorInMap(xA, yA, theta, map, mScale)
% map: binary value 0 and non-0

if(nargin<3)== 25; end

szAug = size(map,1) + size(map,2);
rD = round(sqrt(size(map,1)^2 + size(map,2)^2));
mapAug = zeros(3*szAug, 3*szAug); 
mapAug((szAug+1):(szAug+size(map,1)), (szAug+1):(szAug+size(map,2))) = map;

xOrg = xA*mScale+szAug; yOrg = yA*mScale+szAug;

theta = theta; rotMInv = [cos(theta), sin(theta); -sin(theta), cos(theta)];
angRange = (pi/36):(pi/36):(2*pi);
ptRange = zeros(2,length(angRange));
rScan = 1:szAug;
for angIdx = 1:length(angRange)
    ang = angRange(angIdx);
    xScan = round(xOrg+rScan*cos(ang+theta)); yScan = round(yOrg+rScan*sin(ang+theta));
    idxScan = (xScan-1)*size(mapAug,1)+yScan;
    idxMin = idxScan(min(find(mapAug(idxScan)==0)));
    if (~isempty(idxMin))        
        xTmp = floor((idxMin-1)/size(mapAug,1))+1;
        yTmp = idxMin-(xTmp-1)*size(mapAug,1);
        ptRange(:,angIdx) = rotMInv*([xTmp;yTmp]-[xOrg;yOrg])/mScale;
    end
end

xR = ptRange(1,:); yR = ptRange(2,:);


end %% END RobotRanging
