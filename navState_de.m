%navState_de computes the derivative of the nav state
%
% Inputs:
%   xhat = nav state (mixed units)
%   input = input (mixed units)
%
% Outputs
%   xhatdot = nav state derivative (mixed units)
%
function xdot = navState_de(x,input)

    %%---------------------------------------------------------------------------
    % Unpack the inputs
    simpar = input.simpar;
    Tbi    = input.Tib';

    %%---------------------------------------------------------------------------
    % Compute individual elements of x_dot
    % Convert state into structure
    s  = extract_state(x, simpar, 'nav');
    z3 = zeros(3,1);

    %%---------------------------------------------------------------------------
    % Assign to output
    xdot(simpar.states.ix.pos, 1)      = s.vel;
    xdot(simpar.states.ix.vel, 1)      = calc_grav(s.pos, simpar) + Tbi*(input.a_meas - s.bias);
    % xdot(simpar.states.ix.vel, 1)      = calc_grav(s.pos, simpar) + Tbi*(input.thrust);
    xdot(simpar.states.ix.cam, 1)      = ecrv(s.cam, simpar.general.tau_c, z3);
    xdot(simpar.states.ix.acc_bias, 1) = ecrv(s.bias, simpar.general.tau_b, z3);
    xdot(simpar.states.ix.pos_f1, 1)   = zeros(3,1);
    xdot(simpar.states.ix.pos_f2, 1)   = zeros(3,1);
    xdot(simpar.states.ix.pos_f3, 1)   = zeros(3,1);
end
