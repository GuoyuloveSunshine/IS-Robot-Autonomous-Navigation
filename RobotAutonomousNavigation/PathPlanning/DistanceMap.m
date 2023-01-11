function distance_map = DistanceMap(map)
    imgW = size(map,2); imgH = size(map,1);
    distance_map = zeros(imgH,imgW);
    distance_map(:,:)= inf;
    idx = (map==0);
    distance_map(idx)=0;

    for idx=2:imgH
        for idy = 2:imgW
            if(distance_map(idx,idy)~=0)
                distance_map(idx,idy) = min(min(distance_map(idx,idy),distance_map(idx-1,idy)+1),distance_map(idx,idy-1)+1);
            end
        end
    end
    
    for idx=1:imgH-1
        for idy = 1:imgW-1
            if(distance_map(imgH-idx,imgW-idy)~=0)
                distance_map(imgH-idx,imgW-idy) = min(min(distance_map(imgH-idx,imgW-idy),distance_map(imgH-idx+1,imgW-idy)+1),distance_map(imgH-idx,imgW-idy+1)+1);
            end
        end
    end
end