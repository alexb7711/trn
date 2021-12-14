%truthState_de computes the derivative of the truth state
%
% Inputs:
%   x = truth state (mixed units)
%   u = input (mixed units)
%
% Outputs
%   xdot = truth state derivative (mixed units)
%
function xdot = truthState_de(x, input)
    %%---------------------------------------------------------------------------
    %% Unpack the inputs
    simpar = input.simpar;
    Tbi    = input.Tib';
    t      = input.t;
    wa     = input.wa;
    wc     = input.wc;
    wg     = input.wg;

    %%---------------------------------------------------------------------------
    %% Compute parameters
    % Convert state into structure
    s = extract_state(x, simpar, 'truth');

    % Create moon quaternion
    q_moon  = [0; 0; 0; simpar.general.W_MOON];

    %%---------------------------------------------------------------------------
    %% Assign to output
    % Initialize xdot
    xdot = zeros(simpar.states.nx,1);

    % Make assignments
    xdot(simpar.states.ix.pos, 1)      = s.vel;
    % xdot(simpar.states.ix.vel, 1)      = calc_grav(s.pos, simpar) + Tbi*input.thrust + wg;
    xdot(simpar.states.ix.vel, 1)      = Tbi*input.thrust + wg;
    xdot(simpar.states.ix.cam, 1)      = ecrv(s.cam, simpar.general.tau_c, wc);
    xdot(simpar.states.ix.acc_bias, 1) = ecrv(s.bias, simpar.general.tau_b, wa);
    xdot(simpar.states.ix.pos_f1, 1)   = zeros(3,1);
    xdot(simpar.states.ix.pos_f2, 1)   = zeros(3,1);
    xdot(simpar.states.ix.pos_f3, 1)   = zeros(3,1);
    xdot(simpar.states.ix.mcmf_att, 1) = 1/2*quatmult(q_moon, s.qm);
end
