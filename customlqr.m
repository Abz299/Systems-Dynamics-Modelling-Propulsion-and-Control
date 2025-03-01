function [K, L, A_sys, B_sys, C_sys, D_sys, A_kalman, B_kalman, C_kalman, D_kalman] = lqr_lqe_psi_fixed()
    % System parameters
    m = 1.1;
    M = 3.57;
    g = 9.81;
    K_D = 0;
    Ixx = 0.036;
    l_p = (0.02 + 0.2) / 2;
    K_v = 0.01;
    l_h = 0.177;
    K_tau = 4.25e-3;

    V_coll_range = 5; % Replace range with a single value
    l_theta = 0.014;
    theta_rest = deg2rad(-25);
    Iyy = 0.93;
    M_q = (0.1 + 0.9) / 2;
    l_boom = 0.66;
    Izz = 0.93;

    % Define state-space matrices (A, B, C, D)
    A_sys = [
        0, 1, 0, 0, 0, 0;
        -m * g * l_theta / Ixx, -l_p / Ixx, 0, 0, 0, (-K_v * l_h) / Ixx;
        0, 0, 0, 1, 0, 0;
        0, 0, -M * g * l_theta * cos(theta_rest) / Iyy, -M_q / Iyy, 0, 0;
        0, 0, 0, 0, 0, 1;
        90 * K_tau * V_coll_range * l_boom / Izz, 0, 0, 0, -K_D / Izz, 0
    ];

    B_sys = [
        0, 0;
        130 * K_tau * l_h / Ixx, 0;
        0, 0;
        0, 90 * K_tau * l_boom / Iyy;
        0, 0;
        0, 0
    ];

    % Modify C_sys to ensure observability
    C_sys = eye(size(A_sys)); % Measure all states

    D_sys = [
        0, 0;
        0, 0
    ];

    % Check Observability
    obsv_matrix = obsv(A_sys, C_sys);
    rank_obsv = rank(obsv_matrix);
    disp('Rank of Observability Matrix:');
    disp(rank_obsv);

    if rank_obsv < size(A_sys, 1)
        error('System is NOT Observable. Adjust the C matrix or simplify the system.');
    end

    % Define weighting matrices for LQR
    Q_lqr = diag([10, 1, 10, 1, 10, 1]); % Penalize state deviations
    R_lqr = diag([0.1, 0.1]);           % Penalize control inputs

    % Compute LQR gain
    K = lqr(A_sys, B_sys, Q_lqr, R_lqr);

    % Define noise covariances for LQE
    Qn = diag([0.01, 0.01, 0.01, 0.01, 0.01, 0.01]); % Process noise covariance
    Rn = diag(0.1 * ones(size(C_sys, 1), 1));        % Measurement noise covariance

    % Compute LQE gain
    L = lqe(A_sys, eye(size(A_sys)), C_sys, Qn, Rn);

    % Modify state-space matrices for Kalman filter
    A_kalman = A_sys - L * C_sys;
    B_kalman = [B_sys, L];
    C_kalman = eye(size(A_sys)); % Outputs the estimated states
    D_kalman = zeros(size(C_kalman, 1), size(B_kalman, 2));

    % Display results
    disp('LQR Gain Matrix (K):');
    disp(K);

    disp('LQE Gain Matrix (L):');
    disp(L);


end
