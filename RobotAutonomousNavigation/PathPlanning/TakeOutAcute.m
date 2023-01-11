function optPath=TakeOutAcute(path)
    flag = true;
    while(flag)
        flag = false;
        optPath = path;
        for idx=2:length(optPath)-1
            a = norm(optPath(:,idx)-optPath(:,idx-1));
            b = norm(optPath(:,idx+1)-optPath(:,idx));
            c = norm(optPath(:,idx+1)-optPath(:,idx-1));
            theta = acos((a^2+b^2-c^2)/(2*a*b));
            if theta<=pi/9*5.5
                optPath(:,idx) = [];
                flag = true;
                break
            end
        end
        path = optPath;
    end
end