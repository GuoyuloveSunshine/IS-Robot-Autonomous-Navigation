
function edges = PlanningRRT(img, pt_stt,pt_end)

    imgW = size(img,2); imgH = size(img,1);
    figure(1), imshow(img); set(gca, 'YDir', 'normal');
    pt_stt = reshape(pt_stt, [], 1);
    pt_end = reshape(pt_end, [], 1);
    % Judge whether connection of two points is feasible
    function ret = IsLineFeasible(pt1, pt2, img)
        ret = 1;
        for nmda = 0:0.0001:1
            pt = round(pt1*nmda + pt2*(1-nmda));
            if (img(pt(2), pt(1))==0)
                ret = 0;
                break;
            end
        end
    end

    % Judge whether the destination point is connectible to current RRT graph 
    function edges_o = IsEndConnectible(edges_i, pt_end, img)
        edges_o = edges_i;
        for idx = 1:size(edges_i,2)
            if (IsLineFeasible(edges_i(:,idx), pt_end, img))
                edges_o = [edges_i, edges_i(:,idx), pt_end];
                break;
            end
        end
    end

    function ret = GetDist(pt1, pt2)
        ret = sqrt((pt1-pt2)'*(pt1-pt2));
    end
    function pt_closest = GetClosestPtOnLine(pt, pt1, pt2)
        % pt_ret = nmda*(pt1-pt2)+pt2
        % min || nmda*(pt1-pt2)+pt2-pt ||
        base_dist = (pt1-pt2)'*(pt1-pt2);
        if (base_dist==0)
            pt_closest = pt1;
            return
        end
        nmda = (pt1-pt2)'*(pt-pt2)/base_dist;
        if (nmda>1)
            nmda = 1;
        elseif (nmda<0)
            nmda = 0;
        end
        pt_closest = nmda*(pt1-pt2)+pt2;
    end
    % Expand the RRT graph by adding a new feasible edge
    function [pt_ret, edges_o] = FindConnectiblePoint(edges_i, img)
        imgW = size(img,2); imgH = size(img,1);        
        is_connectible_flg = 0;
        while (is_connectible_flg == 0)
            pt_ret = rand(2,1);
            pt_ret = [5;5] + diag([imgW-10,imgH-10])*pt_ret;
            dist_min = imgW^2+imgH^2;
            for idx = 1:2:size(edges_i,2)
                pt_tmp = GetClosestPtOnLine(pt_ret, edges_i(:,idx), edges_i(:,idx+1));
                if (IsLineFeasible(pt_tmp, pt_ret, img))
                    dist_tmp = GetDist(pt_tmp, pt_ret);
                    if (dist_tmp<dist_min)
                        dist_min = dist_tmp;
                        pt_min = pt_tmp;
                        is_connectible_flg = 1;
                    end
                end
            end
        end         
        edges_o = [pt_ret, pt_min, edges_i];        
    end

    function PlotEdges(edges)
        hold on;
        for idx = 1:2:size(edges,2)
            line(edges(1,idx:idx+1), edges(2,idx:idx+1), 'Color', 'b','LineWidth', 2);
        end
        hold off;
    end

edges = [pt_stt, pt_stt];

while(edges(1,end)~=pt_end(1) || edges(2,end)~=pt_end(2))   
    [pt_new, edges] = FindConnectiblePoint(edges, img);
    edges = IsEndConnectible(edges, pt_end, img);
    figure(1), imshow(img); set(gca, 'YDir', 'normal');
    hold on; plot(pt_stt(1), pt_stt(2), 'ob', pt_end(1), pt_end(2), 'or', 'MarkerSize', 12, 'LineWidth', 4); hold off; PlotEdges(edges);
end
end %% END PlanningRRT
