function fitness = ga_fitness_jsc_lowdim( ...
    x, H_comm, sigmasq_ue, sensing_beam, ...
    sigmasq_radar_rcs, ...
    P_total, SINR_min_req, ...
    F_comm_ref, F_sensing_ref, rho)

alpha = x(1);
beta  = x(2);

[F_comm, F_sensing] = decode_lowdim( ...
    [alpha, beta], F_comm_ref, F_sensing_ref, P_total, rho);

%% Communication SINR
SINR_u = compute_SINR(H_comm, F_comm, F_sensing, sigmasq_ue);
SINR_min = min(SINR_u);

if SINR_min < SINR_min_req
    fitness = 1e5 + (SINR_min_req - SINR_min)^2;
    return;
end

%% Sensing objective
SSNR = compute_sensing_SNR( ...
    sigmasq_radar_rcs, sensing_beam, F_comm, F_sensing);

fitness = -SSNR;

end
