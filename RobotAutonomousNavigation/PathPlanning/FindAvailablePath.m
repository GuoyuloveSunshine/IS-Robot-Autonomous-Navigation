
function avl_path = FindAvailablePath(edges, pt_stt)
    pt_stt = reshape(pt_stt, [], 1);
%     pt_end = reshape(pt_end, [], 1);

    avl_path = [edges(:,end),edges(:,end-1)];
    now_point = avl_path(:,end);
    for idx=1:2:size(edges,2)-2
        if (edges(:,idx) == now_point)
            avl_path = [avl_path, edges(:,idx+1)];
            now_point = avl_path(:,end);
        elseif (abs((edges(2,idx)-edges(2,idx+1))/(edges(1,idx)-edges(1,idx+1))-(now_point(2)-edges(2,idx+1))/(now_point(1)-edges(1,idx+1)))<=0.0001)
            avl_path = [avl_path, edges(:,idx+1)];
            now_point = avl_path(:,end);
        end
        if (now_point==pt_stt)
            avl_path = flip(avl_path,2);
            break
        end
    end
end