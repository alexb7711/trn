% Input:
%   x          : State vector
%   s          : Simulation parameters
%   state_type : Define to extract nav state parameters or truth
%
% Output:
%   state: Structure form of the state vector
%
function [state] = extract_state(x, s, state_type)
    state.pos    = x(s.states.ix.pos,1);
    state.vel    = x(s.states.ix.vel,1);
    state.cam    = x(s.states.ix.cam,1);
    state.bias   = x(s.states.ix.acc_bias,1);
    state.f1     = x(s.states.ix.pos_f1,1);
    state.f2     = x(s.states.ix.pos_f2,1);
    state.f3     = x(s.states.ix.pos_f3,1);

    % If we are extracting the truth states
    if strcmp(state_type, 'truth')
        state.qm = x(s.states.ix.mcmf_att);
    end
end
