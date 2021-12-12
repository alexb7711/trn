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
    omega  = input.u;
    w      = input.w;

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
    xdot(simpar.states.ix.pos)      = s.vel;
    xdot(simpar.states.ix.vel)      = calc_grav(s.pos, simpar) + input.Tib'*(input.thrust) + w;
    xdot(simpar.states.ix.cam)      = ecrv(s.cam, simpar.general.tau_c, w);
    xdot(simpar.states.ix.acc_bias) = ecrv(s.bias, simpar.general.tau_b, w);
    xdot(simpar.states.ix.pos_f1)   = zeros(3,1);
    xdot(simpar.states.ix.pos_f2)   = zeros(3,1);
    xdot(simpar.states.ix.pos_f3)   = zeros(3,1);
    xdot(simpar.states.ix.mcmf_att) = 1/2*quatmult(q_moon, s.qm);
end
