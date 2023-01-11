
function ret = IsCollision(robotState, map, mScale)
% map: binary value 0 and non-0

if(nargin<3) mScale = 25; end

pts = round(mScale*(repmat(robotState(1:2),1,5)+[[0;0], [0.5;0], [0;0.5], [-0.5;0], [0;-0.5]]));

for idx = 1:5
    r = pts(2,idx); c = pts(1,idx);
    if (r<=1 || r>=size(map,1) || c<=1 || c>=size(map,2) || map(r,c)==0)
        ret = 1;
        return
    end
end

ret = 0;

end %% END RobotRanging
