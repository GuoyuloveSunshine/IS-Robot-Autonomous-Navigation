function v = CalculateV(point_idx, omega, pxl, mScale)
    if pxl<= mScale
        if (-0.4<=omega && omega<=-0.1)||(0.1<=omega && omega<=0.4) 
                v = 0.2;
        elseif (-0.1<omega && omega <0.1)  
                v = 0.3;
        elseif (omega==1 || omega==-1) 
                v = 0;
        else  
                v = 0.1;
        end
    elseif  pxl>mScale && pxl <=mScale*1.4
        if (-0.4<=omega && omega<=-0.1)||(0.1<=omega && omega<=0.4)
            if (point_idx==2)   
                v = 0.3;
            else
                v = 0.5;
            end
        elseif (-0.1<omega && omega <0.1)
            if (point_idx==2)   
                v = 0.5;
            else
                v = 1;
            end
        elseif (omega==1 || omega==-1)
            if (point_idx==2)   
                v = 0;
            else
                v = 0.1;
            end
        else
            if (point_idx==2)   
                v = 0.1;
            else
                v = 0.3;
            end
        end
    else
        if (-0.7<=omega && omega<=-0.4)||(0.4<=omega && omega<=0.7)
            v = 3;
        elseif (-0.4<omega && omega <0.4)
            v = 5;
        elseif (omega==1 || omega==-1)
            if (point_idx==2)   
                v = 0;
            else
                v = 1;
            end
        else
            v = 1.5;
        end
    end
end