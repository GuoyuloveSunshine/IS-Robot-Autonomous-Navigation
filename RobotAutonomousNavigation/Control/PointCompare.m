function point_idx_nxt = PointCompare(car_position,path,point_idx)
    if (point_idx==length(path))
        point_idx_nxt = point_idx;
        return;
    end
    %%%%%%%%%%%%%A、B点构成向量AB;C为任一点
    A = path(:,point_idx);%向量1A 点
    B = path(:,point_idx-1);%向量1B 点
    C = car_position;
    %%%%%%%%%%%%%A、B点构成向量AB;C为任一点
     
    AngBAC = acosd((norm(A-B)^2+norm(A-C)^2-norm(B-C)^2)/(2*(norm(A-B)*norm(A-C))));%角A
    AngABC = acosd((norm(B-A)^2+norm(B-C)^2-norm(A-C)^2)/(2*(norm(B-A)*norm(B-C))));%角B
    if AngBAC>AngABC || AngBAC==AngABC
        AA = B; 
        BB = A; 
        D = [cosd(0),sind(0); -sind(0),cosd(0)]...
            *((BB - AA).* ((norm(AA-C) * cosd(AngABC)) / norm(A-B))) + AA;
    else
        D = [cosd(0),sind(0); -sind(0),cosd(0)]...
            *((B - A).* ((norm(A-C) * cosd(AngBAC)) / norm(A-B))) + A;
    end
    clear AA BB AngBAC AngABC ;
    if (sign(D(2)-A(2))==sign(A(2)-B(2))) && (sign(D(1)-A(1))==sign(A(1)-B(1)))
        point_idx_nxt = point_idx+1;
    else
        point_idx_nxt = point_idx;
    end
end