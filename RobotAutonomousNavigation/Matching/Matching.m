function [xA_rtf,yA_rtf,ang_rtf,cSensorw,rSensorw] = Matching(map,distance_map,xA,yA,xR,yR,mScale)
    flag = true;
    interval_pi = pi/18;
    interval_x = 0.2;
    interval_y = 0.2;
    while(flag)
        distance = inf;
        for c=max(1/mScale,xA-6):interval_x:min(xA+6,size(distance_map,2)/mScale)
            for r =max(1/mScale,yA-6):interval_y:min(yA+6,size(distance_map,1)/mScale)
                for ang = -pi:interval_pi:pi
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
                        angle = ang;
                    end
                end
            end
        end
        if(distance~=inf)
            flag = false;
            [xRGPS, yRGPS] = RobotSensorInMap(xA_rtf, yA_rtf, angle, map, mScale);
            cSensorwInMap = ([cos(angle), -sin(angle)]*[xRGPS;yRGPS]+xA_rtf)*mScale;
            rSensorwInMap = ([sin(angle), cos(angle)]*[xRGPS;yRGPS]+yA_rtf)*mScale;
            fixed = [rSensorwInMap;cSensorwInMap];

            cSensorw_tmp = ([cos(angle), -sin(angle)]*[xR;yR]+xA_rtf)*mScale;
            rSensorw_tmp = ([sin(angle), cos(angle)]*[xR;yR]+yA_rtf)*mScale;
            moving = [rSensorw_tmp;cSensorw_tmp];
            %matching
            [Ricp,Ticp] = ICPMatching(fixed,moving);
            Dicp = Ricp * moving + repmat(Ticp, 1, size(moving,2));
            
            tmp_cord = (Ticp + [yA_rtf*mScale;xA_rtf*mScale])/mScale;
            xA_rtf = tmp_cord(2);
            yA_rtf = tmp_cord(1);
            cSensorw = Dicp(2,:);
            rSensorw = Dicp(1,:);

            tmp = [cos(angle),-sin(angle);sin(angle),cos(angle)];
            tmp_mat = Ricp*tmp;
            tmp_ang = acos(tmp_mat(1,1));
            if tmp_mat(2,1)>0
                ang_rtf = tmp_ang;
            else
                ang_rtf = -tmp_ang;
            end
        else
            a = rand();
            if a<0.33
                interval_pi = interval_pi/2;
            elseif 0.33<=a||a<=0.66
                interval_x = interval_x/2;
            else
                interval_y = interval_y/2;
            end
        end
    end
end