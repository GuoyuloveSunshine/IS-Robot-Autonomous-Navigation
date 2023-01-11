function ret = IsBadPath(path,distance_map,mScale)
    for i=1:length(path)-1
        point_now = path(:,i);
        point_next = path(:,i+1);
        tmp = abs(point_next-point_now);
        max_sep = max(tmp(1),tmp(2))+1;
        c = round(linspace(point_now(1),point_next(1),max_sep));
        r = round(linspace(point_now(2),point_next(2),max_sep));
        for j=1:length(r)
            distance_map(r(j),c(j))
            if distance_map(r(j),c(j)) <3/5*mScale
                ret = 1;
                return
            end
        end
    end
    ret = 0;
end