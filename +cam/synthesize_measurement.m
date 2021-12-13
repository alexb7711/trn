%synthesize_measurement_example synthesizes the discrete measurement
%corrupted by noise
%
% Inputs:
%   input: Compool of parameters used to syntheize measurement
%
% Outputs
%   z: Pin-Hole camera model measurement
%
% Example Usage
% [ output_args ] = synthesize_measurement_example( input_args )
%
function [ z ] = synthesize_measurement( input )
    %----------------------------------------------------------------------------
    % Local variables
    simpar = input.simpar;
    s      = extract_state(input.x, simpar, 'nav');
    Tbc    = input.Tbc;
    Tib    = input.Tib;
    Tmi    = input.Tmi;
    rcb    = [simpar.general.rcbx; simpar.general.rcby; simpar.general.rcbz];

    %----------------------------------------------------------------------------
    % Model feature locations
    lc1 = Tbc * (Tib * (Tmi * s.f1 - s.pos) - rcb);
    lc2 = Tbc * (Tib * (Tmi * s.f2 - s.pos) - rcb);
    lc3 = Tbc * (Tib * (Tmi * s.f3 - s.pos) - rcb);

    %----------------------------------------------------------------------------
    % Synthesize measurement

    z = [lc1(1)/lc1(3); lc1(2)/lc1(3);
         lc2(1)/lc2(3); lc2(2)/lc2(3);
         lc3(1)/lc3(3); lc3(2)/lc3(3)];
end
