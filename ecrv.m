% Input:
%   v: Value of the ecrv
%   t: Tau value of ecrv
%   w: Noise
%
% Output:
%   x: ECRV value
%
function x = ecrv(v, t, w)
    x = -1/t * v + w
end
