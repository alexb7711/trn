function [ output_args ] = validate_linearization_example( input_args )
%validate_linearization_example validates the calculation of the
%measurement sensitivity matrix
%
% Inputs:
%   Input1 = description (units)
%   Input2 = description (units)
%
% Outputs
%   Output1 = description (units)
%   Output2 = description (units)
%
% Example Usage
% [ output_args ] = validate_linearization_example( input_args )
%
% See also FUNC1, FUNC2

% Author: Randy Christensen
% Date: 31-Aug-2020 16:01:19
% Reference: 
% Copyright 2020 Utah State University

%% Inject Error
fnames = fieldnames(simpar.errorInjection);
dele_injected = zeros(numel(fnames),1);
for j=1:length(fnames)
    dele_injected(j) = simpar.errorInjection.(fnames{j});
end
xhat = injectErrors(truth2nav(x), dele_injected, simpar);
%% Calculate residual
z_tilde = example.synthesize_measurement(x, phat_m, t, simpar);
delz_nl = example.compute_residual(xhat, z_tilde, phat_m, t, simpar);
H = example.compute_H(xhat, phat_m, t, simpar);
delz_l = H*dele_injected;
%% Compare linear and nonlinear residuals
measLinTable.delz_nl = delz_nl;
measLinTable.delz_l = delz_l;
measLinTable.difference = delz_nl - delz_l;
disp(struct2table(measLinTable));
end
