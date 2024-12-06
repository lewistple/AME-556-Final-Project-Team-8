function [sigma_l, sigma_r] = gait_schedule(N, k)

sigma_l0 = [ones(1, N/2) zeros(1, N/2)];
sigma_r0 = [zeros(1, N/2) ones(1, N/2)];

sigma_l = [sigma_l0(k+1: N) sigma_l0(1: k)];
sigma_r = [sigma_r0(k+1: N) sigma_r0(1: k)];

end