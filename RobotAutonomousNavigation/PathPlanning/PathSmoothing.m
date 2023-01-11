function optPath_norepeat=PathSmoothing(path,distance_map)
    smoothPath = path(:,1);
    section = 40;
    tmp_idx = 2;
    now_x = path(1,1);
    while(tmp_idx<=length(path))
%         tmp_sign = sign(path(1,tmp_idx)-path(1,tmp_idx-1));
        total_length = sqrt((path(2,tmp_idx)-path(2,tmp_idx-1)).^2+(path(1,tmp_idx)-path(1,tmp_idx-1)).^2);
        num = ceil(total_length/section);
        seg_x = (path(1,tmp_idx)-path(1,tmp_idx-1))/num;
        k = (path(2,tmp_idx)-path(2,tmp_idx-1))/(path(1,tmp_idx)-path(1,tmp_idx-1));
        for i=1:num
            now_x = now_x+seg_x;
            now_y = k* (now_x-path(1,tmp_idx-1))+path(2,tmp_idx-1);
            smoothPath = [smoothPath,[now_x;now_y]];
        end
        tmp_idx = tmp_idx+1;
    end
    
    optPath = zeros(size(smoothPath));
    optPath(:,1) = smoothPath(:,1);
    optPath(:,length(smoothPath)) = smoothPath(:,length(smoothPath));
    for point_idx = 2:length(smoothPath)-1
        init_x = round(smoothPath(1,point_idx)); 
        init_y = round(smoothPath(2,point_idx)); 
        now_x = round(smoothPath(1,point_idx)); %%colonne
        now_y = round(smoothPath(2,point_idx)); %%row
        
        interval = 40;
        if(abs(smoothPath(1,point_idx)-smoothPath(1,point_idx-1))<=abs(smoothPath(2,point_idx)-smoothPath(2,point_idx-1))) 
            %%augmentation selon x plus que selon y
            for c =max(1,init_x-interval):min(init_x+interval,size(distance_map,2))
                if distance_map(init_y,c) > distance_map(init_y,now_x)
                    now_y = init_y;
                    now_x = c;
                end
            end
        else
            for r =max(1,init_y-interval):min(init_y+interval,size(distance_map,1))
                if distance_map(r,init_x) > distance_map(now_y,init_x)
                    now_y = r;
                    now_x = init_x;
                end
            end
        end
        optPath(:,point_idx) = [now_x,now_y];
    end
    
    [optPath_norepeat,~,~]=unique(optPath','rows','stable');
    optPath_norepeat = optPath_norepeat';
    optPath_norepeat = TakeOutAcute(optPath_norepeat);
end


% function optPath=PathSmoothing(path,distance_map)
%     smoothPath = path(:,1);
%     section = 50;
%     tmp_idx = 2;
%     now_x = path(1,1);
%     while(tmp_idx<=length(path))
%         tmp_sign = sign(path(1,tmp_idx)-path(1,tmp_idx-1));
%         now_x = now_x+tmp_sign*section;
%         now_y = (path(2,tmp_idx)-path(2,tmp_idx-1))/(path(1,tmp_idx)-path(1,tmp_idx-1))* (now_x-path(1,tmp_idx-1))+path(2,tmp_idx-1);
%         smoothPath = [smoothPath,[now_x;now_y]];
%         if((now_x-path(1,tmp_idx))*tmp_sign>=0)
%             smoothPath(:,length(smoothPath)) = path(:,tmp_idx);
%             now_x = path(1,tmp_idx);
%             tmp_idx = tmp_idx+1;
%         end
%     end
% 
%     optPath = zeros(size(smoothPath));
%     optPath(:,1) = smoothPath(:,1);
%     optPath(:,length(smoothPath)) = smoothPath(:,length(smoothPath));
%     for point_idx = 2:length(smoothPath)-1
%         init_x = round(smoothPath(1,point_idx)); 
%         init_y = round(smoothPath(2,point_idx)); 
%         now_y = round(smoothPath(2,point_idx)); %%row
%         
%         interval = 50;
%         for r =max(1,init_y-interval):min(init_y+interval,size(distance_map,1))
%             if distance_map(r,init_x) > distance_map(now_y,init_x)
%                 now_y = r;
% %                 disp([num2str(point_idx),":      ",num2str(now_y),"\n"])
%             end
%         end
%         optPath(:,point_idx) = [init_x,now_y];
%     end
% 
% end