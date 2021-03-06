function [ hfigs ] = plotMonteCarlo(errors, traj_ref, traj, simpar)
%PLOTMONTECARLO_GPSINS creates hair plots of the estimation error for each
%state.  I left example code so you can see how I normally make these
%plots.
[n, ~, ~] = size(errors);
hfigs = [];

%% Plot estimation errors
ylabels = {'X_i Position Est Err (m)',...
    'Y_i Position Est Err (m)',...
    'Z_i Position Est Err (m)',...
    'X_i Velocity Est Err (m/s)',...
    'Y_i Velocity Est Err (m/s)',...
    'Z_i Velocity Est Err (m/s)',...
    'X_c Camera Misalign Est Err (rad)',...
    'Y_c Camera Misalign Est Err (rad)',...
    'Z_c Camera Misalign Est Err (rad)',...
    'X_b Accel Bias Est Err (m/s^s)',...
    'Y_b Accel Bias Est Err (m/s^s)',...
    'Z_b Accel Bias Est Err (m/s^s)',...
    'F1x Attitude Est Err (rad)',...
    'F1y Attitude Est Err (rad)',...
    'F1z Attitude Est Err (rad)',...
    'F2x Attitude Est Err (rad)',...
    'F2y Attitude Est Err (rad)',...
    'F2z Attitude Est Err (rad)',...
    'F3x Attitude Est Err (rad)',...
    'F3y Attitude Est Err (rad)',...
    'F3z Attitude Est Err (rad)'};

for i=1:n

    hfigs(end + 1) = figure('Name',sprintf('est_err_%d',i)); %#ok<*AGROW>
    hold on;
    grid on;
    ensemble = squeeze(errors(i,:,:));
    filter_cov = squeeze(traj_ref.navCov(i,i,:));
    h_hair = stairs(traj_ref.time_nav, ensemble,'Color',[0.8 0.8 0.8]);
    h_filter_cov = stairs(traj_ref.time_nav, ...
        [3*sqrt(filter_cov) -3*sqrt(filter_cov)],'--r');
    legend([h_hair(1), h_filter_cov(1)],'MC run','EKF cov')
    xlabel('time(s)')
    ylabel(ylabels{i})
end

%% Create estimation error plots in LVLH frame
nsamp = length(traj_ref.time_nav);
%Transform
P_lvlh = zeros(simpar.states.nxfe, simpar.states.nxfe, nsamp);
errors_lvlh = zeros(simpar.states.nxfe,nsamp,...
    simpar.general.n_MonteCarloRuns);
for i=1:nsamp
    r_i = traj_ref.truthState(simpar.states.ix.pos,i);
    v_i = traj_ref.truthState(simpar.states.ix.vel,i);
    T_i2lvlh = inertial2lvlh(r_i, v_i);
    A = blkdiag(T_i2lvlh, T_i2lvlh, ...
        eye(simpar.states.nxfe-6, simpar.states.nxfe-6));
    P_lvlh(:,:,i) = A*traj_ref.navCov(:,:,i)*A';
    for j=1:simpar.general.n_MonteCarloRuns
        errors_lvlh(:,i,j) = A*errors(:,i,j);
    end
end
ylabels = {'$X_{LVLH}$ Position Error $(m)$',...
    '$Y_{LVLH}$ Position Error $(m)$',...
    '$Z_{LVLH}$ Position Error $(m)$',...
    '$X_{LVLH}$ Velocity Error $(m/s)$',...
    '$Y_{LVLH}$ Velocity Error $(m/s)$',...
    '$Z_{LVLH}$ Velocity Error $(m/s)$'};
for i=1:6
    hfigs(end+1) = figure('Name',sprintf('est_err_lvlh_%d',i)); %#ok<*AGROW>
    hold on;
    grid on;
    ensemble = squeeze(errors_lvlh(i,:,:));
    filter_cov = squeeze(P_lvlh(i,i,:));
    h_hair = stairs(traj_ref.time_nav, ensemble,'Color',[0.8 0.8 0.8]);
    h_filter_cov = stairs(traj_ref.time_nav, ...
        [3*sqrt(filter_cov) -3*sqrt(filter_cov)],'--r');
    legend([h_hair(1), h_filter_cov(1)],'MC run','EKF cov')
    xlabel('time$\left(s\right)$','Interpreter','latex');
    ylabel(ylabels{i},'Interpreter','latex');
end
end
