% Input:
%   x_buff: Nav state buffer
%   i     : Current simulation index
%   s     : Strcture of the current truth state (used to get rotation of moon)
%   simpar: Simulation parameters
%
% Output
%   input_predict: Input values for truth state
%
function [input_predict] = inputPredict(x_buff, i, s, simpar)
    s_nav                = extract_state(x_buff(:,i), simpar, 'nav');

    input_predict.simpar = simpar;
    input_predict.Tib    = Ti2b(s_nav.pos, s_nav.vel, simpar);
    input_predict.Tbc    = Tb2c(x_buff(:,i), simpar);
    input_predict.Tmi    = Tm2i(s.qm);
    input_predict.x      = x_buff(:,i);
end

