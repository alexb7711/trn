% Input:
%   x          : State vector
%   s          : Simulation parameters
%   state_type : Define to extract nav state parameters or truth
%
% Output:
%   state: Structure form of the state vector
%
function [state] = extract_state(x, s, state_type)
    state.pos    = x(s.states.ix.pos);
    state.vel    = x(s.states.ix.vel);
    state.cam    = x(s.states.ix.cam);
    state.bias   = x(s.states.ix.acc_bias);
    state.f1     = x(s.states.ix.pos_f1);
    state.f2     = x(s.states.ix.pos_f2);
    state.f3     = x(s.states.ix.pos_f3);

    % If we are extracting the truth states
    if strcmp(state_type, 'truth')
        state.qm = x(s.states.ix.mcmf_att);
    end
end
