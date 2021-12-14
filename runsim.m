function [ traj ] = runsim( simpar, verbose, seed)
rng(seed);

%RUNSIM Runs a single trajectory given the parameters in simparams
tic;

%%===============================================================================
%% Prelims
%Derive the number of steps in the simulation and the time
nstep     = ceil(simpar.general.tsim/simpar.general.dt + 1);
nstep_aid = ceil(simpar.general.tsim/simpar.general.dt_kalmanUpdate);
t         = (0:nstep-1)'*simpar.general.dt;
t_kalman  = (0:nstep_aid)'.*simpar.general.dt_kalmanUpdate;
nstep_aid = length(t_kalman);

%If you are computing the nominal star tracker or other sensor orientations
%below is an example of one way to do this
qz                            = rotv2quat(simpar.general.thz_st,[0,0,1]');
qy                            = rotv2quat(simpar.general.thy_st,[0,1,0]');
qx                            = rotv2quat(simpar.general.thx_st,[1,0,0]');
simpar.general.q_b2st_nominal = qmult(qx,qmult(qy,qz));
qz                            = rotv2quat(simpar.general.thz_c,[0,0,1]');
qy                            = rotv2quat(simpar.general.thy_c,[0,1,0]');
qx                            = rotv2quat(simpar.general.thx_c,[1,0,0]');

simpar.general.q_b2c_nominal = qmult(qx,qmult(qy,qz));

%%===============================================================================
%% Pre-allocate buffers for saving data
% Truth, navigation, and error state buffers
x_buff          = zeros(simpar.states.nx,nstep);
xhat_buff       = zeros(simpar.states.nxf,nstep);
delx_buff       = zeros(simpar.states.nxfe,nstep);

% Navigation covariance buffer
P_buff = zeros(simpar.states.nxfe,simpar.states.nxfe,nstep);

% Continuous measurement buffer
ytilde_buff = zeros(simpar.general.n_inertialMeas,nstep);

% Residual buffers (star tracker is included as an example)
res_cam    = zeros(6,nstep_aid);
resCov_cam = zeros(6,6,nstep_aid);
K_cam_buff = zeros(simpar.states.nxfe,6,nstep_aid);

%%===============================================================================
%% Initialize the navigation covariance matrix
P_buff(:,:,1) = initialize_covariance(simpar);

%%===============================================================================
%% Initialize the truth state vector
x_buff(:,1) = initialize_truth_state(simpar);

%%===============================================================================
%% Initialize the navigation state vector
%% TODO: Change when P_buff is implemented
xhat_buff(:,1) = initialize_nav_state(x_buff(:,1), P_buff(:,1), simpar);
% xhat_buff(:,1) = initialize_nav_state(x_buff(:,1), simpar);

%%===============================================================================
%% Miscellaneous calcs
% Initialize values
s                 = extract_state(x_buff(:,1), simpar, 'truth');
input_init.Tib    = Ti2b(s.pos, s.vel, simpar);
input_init.n_a    = zeros(3,1);
input_init.t      = 1;
input_init.simpar = simpar;

% Synthesize continuous sensor data at t_n-1
% ytilde_buff(:,1) = contMeas(x_buff(:,1), input_init);

%Initialize the measurement counter
k = 1;

%Check that the error injection, calculation, and removal are all
%consistent if the simpar.sim.checkErrDefConstEnable is enabled.
if simpar.sim.checkErrDefConstEnable
    input
    checkErrorDefConsistency(xhat_buff(:,1), x_buff(:,1), simpar)
end

% Inject errors if the simpar.sim.errorPropTestEnable flag is enabled
if simpar.sim.errorPropTestEnable
    fnames = fieldnames(simpar.errorInjection);
    for i=1:length(fnames)
        delx_buff(i,1) = simpar.errorInjection.(fnames{i});
    end
    xhat_buff(:,1) = injectErrors(truth2nav(x_buff(:,1), simpar), delx_buff(:,1), simpar);
end

%%===============================================================================
%% Define PSD processes
Q_a = 2*simpar.truth.params.sig_accel_ss^2/simpar.general.tau_b;
Q_c = 2*simpar.truth.params.sig_c_ss^2/simpar.general.tau_c;
Q_g = simpar.truth.params.Q_grav;

%%===============================================================================
%% Loop over each time step in the simulation
for i=2:nstep
    %----------------------------------------------------------------------------
    % Update truth state structure for easy access
    s     = extract_state(x_buff(:,i-1), simpar, 'truth');
    s_nav = extract_state(xhat_buff(:,i-1), simpar, 'nav');

    %----------------------------------------------------------------------------
    % Propagate truth states to t_n
    %   Realize a sample of process noise (don't forget to scale Q by 1/dt!)
    %   Define any inputs to the truth state DE
    %   Perform one step of RK4 integration
    input_truth    = inputTruth(x_buff, ytilde_buff, t, i, simpar);
    input_truth.wa = sqrt(Q_a /simpar.general.dt) * randn(3,1);
    input_truth.wc = sqrt(Q_c /simpar.general.dt) * randn(3,1);
    input_truth.wg = sqrt(Q_g /simpar.general.dt) * randn(3,1);

    x_buff(:,i) = rk4('truthState_de', x_buff(:,i-1), input_truth, simpar.general.dt);

    % Synthesize continuous sensor data at t_n
    ytilde_buff(:,i) = contMeas(x_buff(:,i), x_buff(:,i-1), input_truth);

    %----------------------------------------------------------------------------
    % Propagate navigation states to t_n using sensor data from t_n-1
    %   Assign inputs to the navigation state DE
    %   Perform one step of RK4 integration

    input_nav      = inputNav(ytilde_buff(:,i), t, i, input_truth, simpar);
    xhat_buff(:,i) = rk4('navState_de', xhat_buff(:,i-1), input_nav, simpar.general.dt);

    % Propagate the covariance to t_n
    input_cov.ytilde = input_truth.a_meas;
    input_cov.simpar = simpar;
    input_cov.xhat   = xhat_buff(:,i);
    input_cov.Tib    = input_truth.Tib;

    P_buff(:,:,i)    = rk4('navCov_de', P_buff(:,:,i-1), input_cov, simpar.general.dt);

    %----------------------------------------------------------------------------
    % Propagate the error state from tn-1 to tn if errorPropTestEnable == 1

    if simpar.sim.errorPropTestEnable
        input_delx.xhat   = xhat_buff(:,i-1);
        input_delx.t      = t(i);
        input_delx.ytilde = ytilde_buff(:,i);
        input_delx.simpar = simpar;
        input_delx.Tib    = input_truth.Tib;
        delx_buff(:,i)    = rk4('errorState_de', delx_buff(:,i-1), input_delx, simpar.general.dt);
    end

    %----------------------------------------------------------------------------
    % If discrete measurements are available, perform a Kalman update
    if abs(t(i)-t_kalman(k+1)) < simpar.general.dt*0.01
        %   Check error state propagation if simpar.general.errorPropTestEnable = true
        if simpar.sim.errorPropTestEnable
            checkErrorPropagation(x_buff(:,i), xhat_buff(:,i), delx_buff(:,i), simpar);
        end

        %Adjust the Kalman update index
        k = k + 1;

        %   For each available measurement
        %       Synthesize the noisy measurement, ztilde
        %       Predict the measurement, ztildehat
        %       Compute the measurement sensitivity matrix, H
        %       If simpar.general.measLinerizationCheckEnable == true
        %           Check measurement linearization
        %       Compute and save the residual
        %       Compute and save the residual covariance
        %       Compute and save the Kalman gain, K
        %       Estimate the error state vector
        %       Update and save the covariance matrix
        %       Correct and save the navigation states

        % Create input compools for synthesize and predict measurement
        input_synthesize = inputSynthesize(x_buff, i, s, simpar);
        input_predict    = inputPredict(xhat_buff, i, s, simpar);

        % Synthesize and predict measurement
        exp_meas     = cam.synthesize_measurement(input_synthesize);
        act_meas     = cam.predict_measurement(input_predict);
        res_cam(:,k) = cam.compute_residual(exp_meas, act_meas);
        % cam.validate_linearization(x_buff(:,i), input_synthesize, s, simpar, i);

        % Initialize kalman matricies
        sig_cu = simpar.truth.params.sig_cu;
        sig_cv = simpar.truth.params.sig_cv;
        R      = diag([sig_cu, sig_cv, sig_cu, sig_cv, sig_cu, sig_cv])^2;
        G      = 1;

        % H_cam             = cam.compute_H(xhat_buff(:,i), input_predict);
        % resCov_cam(:,:,k) = compute_residual_cov(H_cam, P_buff(:,:,i), R);
        % K_cam_buff(:,:,k) = compute_Kalman_gain(H_cam, P_buff(:,:,i), resCov_cam(:,:,k));
        % del_x             = estimate_error_state_vector(res_cam(:,k), K_cam_buff(:,:,k));
        % P_buff(:,:,i)     = update_covariance(P_buff(:,:,i), K_cam_buff(:,:,k), H_cam, R, G, simpar);
        % xhat_buff(:,i)    = correctErrors(xhat_buff(:,i), del_x, simpar);

    end

    if verbose && mod(i,100) == 0
        fprintf('%0.1f%% complete\n',100 * i/nstep);
    end
end

if verbose
    fprintf('%0.1f%% complete\n',100 * t(i)/t(end));
end

T_execution = toc;

% Package up residuals
navRes.cam      = res_cam;
navResCov.cam   = resCov_cam;
kalmanGains.cam = K_cam_buff;

% Package up outputs
traj = struct(                                ...
'navState'                    , xhat_buff   , ...
    'navCov'                  , P_buff      , ...
    'navRes'                  , navRes      , ...
    'navResCov'               , navResCov   , ...
    'truthState'              , x_buff      , ...
    'time_nav'                , t           , ...
    'time_kalman'             , t_kalman    , ...
    'executionTime'           , T_execution , ...
    'continuous_measurements' , ytilde_buff , ...
    'kalmanGain'              , kalmanGains , ...
    'simpar'                  , simpar);
end
