% Input:
%   s: Structure to be converted to an array
%
% Output:
%   a: Array representation of s
%
function [a] = convert_struct_to_array(s)
    a = reshape(struct2array(s), [], numel(fieldnames(s)));
end
