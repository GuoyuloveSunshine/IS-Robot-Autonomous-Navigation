function PlotEdges(edges)
    hold on;
    for idx = 1:size(edges,2)-1
        line(edges(1,idx:idx+1), edges(2,idx:idx+1), 'Color', 'b','LineWidth', 2);
    end
    hold off;
end