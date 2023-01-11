function [xA_rtf,yA_rtf,ang_rtf,cSensorw,rSensorw] = MatchingAfterFirst(map,distance_map,xA,yA,ang_init,xR,yR,mScale)
    flag = true;
    section_x = 1;
    section_y = 1;
    section_pi = pi/12;
    interval_x = 0.2;
    interval_y = 0.2;
    interval_pi = pi/36;
    count = 0;
    while(flag)
        distance = inf;
        for c=max(1/mScale,xA-section_x):interval_x:min(xA+section_x,size(distance_map,2)/mScale)
            for r =max(1/mScale,yA-section_y):interval_y:min(yA+section_y,size(distance_map,1)/mScale)
                for ang = ang_init-section_pi:interval_pi:ang_init+section_pi %-pi-pi
                    cSensorw_tmp = ([cos(ang), -sin(ang)]*[xR;yR]+c)*mScale;
                    rSensorw_tmp = ([sin(ang), cos(ang)]*[xR;yR]+r)*mScale;
                    %matching
                    cPt_final = round(cSensorw_tmp);
                    rPt_final = round(rSensorw_tmp);
                    if(sum(cPt_final<=0)+sum(rPt_final<=0)>0||max(rPt_final)>size(distance_map,1)||max(cPt_final)>size(distance_map,2))
                        continue;
                    end
                    distance_tmp = 0;
                    for i =1:size(cPt_final,2)
                        distance_tmp = distance_tmp + distance_map(rPt_final(i),cPt_final(i));
                    end
                    if distance_tmp < distance
                        distance = distance_tmp;
                        xA_rtf = c;
                        yA_rtf = r;
                        rSensorw = rSensorw_tmp;
                        cSensorw = cSensorw_tmp;
                        ang_rtf = ang;
                    end
                end
            end
        end
        if(distance~=inf)
            flag = false;
        elseif(count<=2)
            count = count+1;
            section_x = section_x+1;
            section_y = section_y+1;
            interval_pi = interval_pi/3*2;
        else
            flag = false;
            xA_rtf = xA;
            yA_rtf = yA;
            ang_rtf = ang_init;
            cSensorw = ([cos(ang_rtf), -sin(ang_rtf)]*[xR;yR]+xA)*mScale;
            rSensorw = ([sin(ang_rtf), cos(ang_rtf)]*[xR;yR]+yA)*mScale;
        end
    end
end