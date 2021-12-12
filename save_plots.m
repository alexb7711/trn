function [ success ] = save_plots(h_figs, prefix, savedir, varargin)
%save_plots saves a list of plots with a prefix, number, and figure name in
%the file name
%
% Inputs:
%   h_figs = array of figure handles
%   prefix = prefix to add to all file names
%   savedir = save directory
%
% Outputs
%   success = flag to determine if the save was successful
%
% Example Usage
% [ success ] = save_plots(h_figs, prefix, savedir)

% Author: Randy Christensen
% Date: 09-Jun-2020 15:28:32
% Reference:
% Copyright 2020 Utah State University

if length(varargin) >= 1
    save_mfig = varargin{1};
else
    save_mfig = false;
end

fprintf('Saving plots ...\n');
try
    for i=1:length(h_figs)
        h = h_figs(i);
        filesubstr = matlab.lang.makeValidName(get(h,'Name'));
        figfilename = sprintf('%s_%d_%s',prefix,i,filesubstr);
        saveas(h,fullfile(savedir,figfilename),'png');
        if save_mfig
            saveas(h,fullfile(savedir,figfilename),'fig');
        end
        %"Print" a pdf
        set(h,'Units','inches');
        screenposition = get(h,'Position');
        set(h,...
            'PaperPosition',[0 0 screenposition(3:4)],...
            'PaperSize',screenposition(3:4));
        print(h,fullfile(savedir,figfilename),'-dpdf','-r600')
        print(h,fullfile(savedir,figfilename),'-dpng','-r600')
    end
    success = true;
    fprintf('Plots saved\n');
catch
    fprintf('Save unsuccessful!\n')
    success = false;
end
end
