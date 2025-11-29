% filename: MATLAB/GNC_Algorithms/Control/LQT-ObsAvd/offline_P08b_multi_traj_obs.m

function offline_P08b_multi_traj_obs()

    %quadcopter parameters
    px4_config = get_x500_params();

    % NMPC setup
    Ts = 1/3;                            % sampling time
    N_horizon = 30;                      % prediction horizon length
    T_sim = 60;                           % total simulation time

    % build NPMC with obstacle as constraints
    nmpc = p08b_setup_mpc_multi_tracj_obs(px4_config, Ts, N_horizon);

    
end