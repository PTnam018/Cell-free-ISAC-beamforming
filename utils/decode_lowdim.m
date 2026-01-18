function [F_comm, F_sensing] = decode_lowdim( ...
    x, F_comm_ref, F_sensing_ref, P_total, rho)

alpha = x(1);
beta  = x(2);

P_comm = rho * P_total;
P_sens = (1 - rho) * P_total;

F_comm = alpha * F_comm_ref;
F_sensing = beta * F_sensing_ref;

F_comm = F_comm / norm(F_comm(:)) * sqrt(P_comm);
F_sensing = F_sensing / norm(F_sensing(:)) * sqrt(P_sens);

end
