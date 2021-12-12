clearvars
close all
clc
totalTimeId = tic;
set(groot, 'defaultAxesTickLabelInterpreter','latex');

%%===============================================================================
%% Setup paths and matlab file object for saving data
filename = getDateTimeStringFilename( 'sim' );

% If the computer is a linux machine
if computer() == 'GLNXA64'
    savedir   = strcat('/tmp/', filename, '/');
% Of if the computer is a windows machine
elseif computer() == 'PCWIN64'
    savedir = strcat('.\sims\',filename,'\');
end

paramfile = 'config.xlsx';
mkdir(savedir)
copyfile(paramfile,strcat(savedir,filename,'_',paramfile));

[~,scriptname,~] = fileparts(mfilename('fullpath'));
try
    copyfile(strcat(mfilename('fullpath'),'.m'),...
        strcat(fullfile(savedir,scriptname),'.m'));
catch
end
filenamepath      = [savedir,filename,'.mat'];
savefile          = matfile(filenamepath);
savefile.savedir  = savedir;
savefile.filename = filename;

%%===============================================================================
%% Read in the simulation parameters
% Define the simparams
[ simpar, ~ ]       = createSimParams( paramfile );
checkProp           = simpar.sim.checkProp;
runSingleMonteCarlo = simpar.sim.runSingleMonteCarlo;
runMonteCarlo       = simpar.sim.runMonteCarlo;
savefigs            = simpar.sim.savefigs;

%%===============================================================================
%% Ensure certain flags are not enabled for certain runs
if simpar.sim.measLinerizationCheckEnable
    assert(runSingleMontedeCarlo == 0,...
        'Measurement linearization check is only valid for checkProp == 1')
    assert(runMonteCarlo == 0,...
        'Measurement linearization check is only valid for checkProp == 1')
    assert(checkProp == 1,...
        'Measurement linearization check is only valid for checkProp == 1')
end

%%===============================================================================
%% Check propagation and nonlinear measurement modeling
if checkProp
    [ ~, simpar_ref ]                              = createSimParams( paramfile );
    simpar_ref.general.processStarTrackerEnable    = 0;
    simpar_ref.general.processVisualOdometryEnable = 0;

    traj_propcheck          = runsim(simpar_ref,1,1);
    savefile.traj_propcheck = traj_propcheck;
    h_figs_prop_check       = plotNavPropErrors(traj_propcheck);
    if savefigs
        disp('Saving "CheckProp" plots...')
        for i = 1:length(h_figs_prop_check)
            h           = h_figs_prop_check(i);
            ha          = gca;
            filesubstr  = matlab.lang.makeValidName(ha.YLabel.String);
            figfilename = sprintf('checkProp_%d___%s',i,filesubstr);

            set(h,'renderer','Painters');
            saveas(h,fullfile(savedir,figfilename),'fig');
            saveas(h,fullfile(savedir,figfilename),'png');
            saveas(h,fullfile(savedir,figfilename),'espc');
        end
        disp('Plots saved.')
    end
end

%%===============================================================================
%% Run single Monte Carlo
if runSingleMonteCarlo
    [ simpar, ~ ] = createSimParams( paramfile );
    traj_single_mc = runsim(simpar,1,1);
    savefile.traj_single_mc = traj_single_mc;
    h_figs_single_mc = plotEstimationErrors(traj_single_mc, simpar);
    if savefigs
        disp('Saving Single Monte Carlo plots...')
        for i = 1:length(h_figs_single_mc)
            h = h_figs_single_mc(i);
            filesubstr = matlab.lang.makeValidName(get(h,'Name'));
            figfilename = sprintf('singleMonteCarlo_%d__%s',i,filesubstr);
            set(h,'renderer','Painters');
            saveas(h,fullfile(savedir,figfilename),'fig');
            saveas(h,fullfile(savedir,figfilename),'png');
            saveas(h,fullfile(savedir,figfilename),'espc');
        end
        disp('Plots saved.')
    end
end

%%===============================================================================
%% Run Multiple Monte Carlos
if runMonteCarlo
    [ simpar, simpar_ref] = createSimParams( paramfile );
    % Simulate reference trajectory
    tic_ref  = tic;
    traj_ref = runsim(simpar_ref,0,1);
    dt_ref   = toc(tic_ref);

    % Preallocated error buffer to enable parallel processing
    nstep  = length(traj_ref.time_nav);
    errors = zeros(simpar.states.nxfe, nstep, simpar.general.n_MonteCarloRuns);

    % Run Monte Carlo simulation
    tic_mc = tic;

    parfor i = 1:simpar.general.n_MonteCarloRuns
        traj(i)       = runsim(simpar, 0, i);
        errors(:,:,i) = calcErrors( traj(i).navState, traj(i).truthState, simpar);
        fprintf('%d/%d complete\n',i, simpar.general.n_MonteCarloRuns);
    end

    dt_mc = toc(tic_mc);

    % Create Monte Carlo plots
    disp('Monte Carlo simulation complete.  Saving data...')
    savefile.errors   = errors;
    savefile.traj     = traj;
    savefile.traj_ref = traj_ref;
    savefile.simpar   = simpar;

    disp('Generating Monte Carlo plots...')

    hfigs = plotMonteCarlo(errors, traj_ref, traj, simpar);

    if savefigs
        disp('Saving Monte Carlo plots...')

        for i = 1:length(hfigs)
            h = hfigs(i);
            filesubstr  = matlab.lang.makeValidName(get(h,'Name'));
            figfilename = sprintf('monteCarlo_%d__%s',i,filesubstr);

            saveas(h,fullfile(savedir,figfilename),'fig');
            saveas(h,fullfile(savedir,figfilename),'png');
            saveas(h,fullfile(savedir,figfilename),'espc');
        end

        disp('Plots saved.')
    end

    fprintf('MC_time = %g\n',dt_ref + dt_mc);
end

%%===============================================================================
%% Final stuff
totalTime          = toc(totalTimeId)
savefile.totalTime = totalTime;
