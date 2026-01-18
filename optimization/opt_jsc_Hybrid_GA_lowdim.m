function [F_comm_opt, F_sensing_opt, Hybrid_info] = ...
    opt_jsc_Hybrid_GA_lowdim( ...
    H_comm, sigmasq_ue, sensing_beam, sigmasq_radar_rcs, ...
    P_total, SINR_min_req, ...
    F_comm_ref, F_sensing_ref, ...
    rho, GA)

% =====================================================
% HYBRID GA-based JSC Beamforming (Low-dimensional)
% Step 1: GA for (alpha, beta)
% Step 2: Local power refinement
% =====================================================

%% ===== Step 1: GA (reuse existing GA-JSC) =====
[F_comm_GA, F_sensing_GA, GA_info] = opt_jsc_GA_lowdim( ...
    H_comm, sigmasq_ue, sensing_beam, sigmasq_radar_rcs, ...
    P_total, SINR_min_req, ...
    F_comm_ref, F_sensing_ref, ...
    rho, GA);

%% Normalize beam directions
v_comm = F_comm_GA / norm(F_comm_GA(:));
v_sens = F_sensing_GA / norm(F_sensing_GA(:));

%% ===== Step 2: Local power refinement =====
% Optimize p_comm \in [0, P_total]
p_low  = 0.01 * P_total;
p_high = 0.99 * P_total;

best_SSNR = -inf;
best_p = rho * P_total;

for p = linspace(p_low, p_high, 50)

    F_comm = sqrt(p) * v_comm;
    F_sens = sqrt(P_total - p) * v_sens;

    % Check SINR constraint
    SINR_u = compute_SINR(H_comm, F_comm, F_sens, sigmasq_ue);
    if min(SINR_u) < SINR_min_req
        continue;
    end

    % Compute sensing SNR
    SSNR = compute_sensing_SNR( ...
        sigmasq_radar_rcs, sensing_beam, F_comm, F_sens);

    if SSNR > best_SSNR
        best_SSNR = SSNR;
        best_p = p;
    end
end

%% ===== Final Hybrid solution =====
F_comm_opt    = sqrt(best_p) * v_comm;
F_sensing_opt = sqrt(P_total - best_p) * v_sens;

Hybrid_info.GA = GA_info;
Hybrid_info.best_p_comm = best_p;
Hybrid_info.best_SSNR   = best_SSNR;
Hybrid_info.type = 'Hybrid-GA-LowDim';

end
