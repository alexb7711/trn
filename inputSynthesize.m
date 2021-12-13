% Input:
%   x_buff: Truth state buffer
%   i     : Current simulation index
%   s     : Strcture of the current truth state (used to get rotation of moon)
%   simpar: Simulation parameters
%
% Output
%   input_synthesize: Input values for truth state
%
function [input_synthesize] = inputSynthesize(x_buff, i, s, simpar)
    input_synthesize.simpar = simpar;
    input_synthesize.Tib    = Ti2b(s.pos, s.vel, simpar);
    input_synthesize.Tbc    = Tb2c(x_buff(:,i), simpar);
    input_synthesize.Tmi    = Tm2i(s.qm);
    input_synthesize.x      = x_buff(:,i);
end
