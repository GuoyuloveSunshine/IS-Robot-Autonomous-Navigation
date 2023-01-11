function omega = CalculateOmega(path, p_idx, x_kf, mScale)
    k = 1;
    R_w2c = [cos(-x_kf(3)), -sin(-x_kf(3));
                sin(-x_kf(3)), cos(-x_kf(3))];
    P_c = R_w2c*[path(1, p_idx) - x_kf(1)*mScale;path(2,p_idx) - x_kf(2)*mScale];
    d_ang = atan2(P_c(2),P_c(1));
    omega = k*d_ang;
    omega = max(-1, omega); omega = min(1, omega);
end