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
function [ ] = validate_linearization(x, input_synthesize, s, simpar, i)
    simpar = input_synthesize.simpar;

    %%---------------------------------------------------------------------------
    %% Inject Error
    fnames        = fieldnames(simpar.errorInjection);
    dele_injected = zeros(numel(fnames),1);

    for j = 1:length(fnames)
        dele_injected(j) = simpar.errorInjection.(fnames{j});
    end

    xhat = injectErrors(truth2nav(x, simpar), dele_injected, simpar);

    %%---------------------------------------------------------------------------
    % Calculate residual

    % Calculate input_predict
    s_hat                = extract_state(xhat, simpar, 'nav');
    input_predict.simpar = simpar;
    input_predict.Tib    = Ti2b(s_hat.pos, s_hat.vel, simpar);
    input_predict.Tbc    = Tb2c(xhat, simpar);
    input_predict.Tmi    = Tm2i(s.qm);
    input_predict.x      = xhat;

    exp_meas             = cam.synthesize_measurement(input_synthesize);
    act_meas             = cam.predict_measurement(input_predict);
    delz_nl              = cam.compute_residual(exp_meas, act_meas);

    H                    = cam.compute_H(xhat, input_predict);
    delz_l               = H*dele_injected;

    %%---------------------------------------------------------------------------
    % Compare linear and nonlinear residuals
    measLinTable.delz_nl    = delz_nl;
    measLinTable.delz_l     = delz_l;
    measLinTable.difference = delz_nl - delz_l;

    disp(struct2table(measLinTable));
end
