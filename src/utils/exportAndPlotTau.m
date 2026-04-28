function exportAndPlotTau(labelOrTau, tauOrDt, varargin)
% EXPORTANDPLOTTAU  Export joint torques to Excel and generate plots.
%
% Flexible interface — first argument can be either the tau matrix or a
% string label. Supports 'all' (single figure) and 'separate' (per-joint).
%
% USAGE:
%   exportAndPlotTau(tau, dt)
%   exportAndPlotTau(tau, dt, 'myFile.xlsx')
%   exportAndPlotTau(tau, dt, 'myFile.xlsx', 'separate')
%   exportAndPlotTau(tau, dt, 'myFile.xlsx', 'all', 'ArmA')
%
% INPUTS:
%   tau  – N×7 or 7×N joint torque matrix [N·m]
%   dt   – sample period [s]
%   (optional) excelFileName – output .xlsx path (default: 'tau_outputs.xlsx')
%   (optional) plotOption    – 'all' | 'separate'            (default: 'all')
%   (optional) label         – legend/title prefix string

if isnumeric(labelOrTau)
    tau=labelOrTau; dt=tauOrDt;
    label=inputname(1); if isempty(label), label='tau'; end
    excelFileName='tau_outputs.xlsx'; plotOption='all';
    if nargin>=3 && ~isempty(varargin{1}), excelFileName=varargin{1}; end
    if nargin>=4 && ~isempty(varargin{2}), plotOption=varargin{2};    end
    if nargin>=5 && ~isempty(varargin{3}), label=varargin{3};         end
else
    label=labelOrTau; tau=tauOrDt;
    if nargin<3, error('Not enough inputs.'); end
    dt=varargin{1}; excelFileName='tau_outputs.xlsx'; plotOption='all';
    if nargin>=4 && ~isempty(varargin{2}), excelFileName=varargin{2}; end
    if nargin>=5 && ~isempty(varargin{3}), plotOption=varargin{3};    end
end

if size(tau,1)==7 && size(tau,2)>7, tau=tau'; end
N=size(tau,1); time=(0:N-1)'*dt;

header=[{'Time'}, arrayfun(@(k)sprintf('%s%d',label,k),1:7,'UniformOutput',false)];
writetable(array2table([time,tau],'VariableNames',header), excelFileName);
fprintf('Saved: %s\n', excelFileName);

switch lower(plotOption)
    case 'all'
        figure('Name',[label ' – All Joints'],'NumberTitle','off');
        plot(time,tau,'LineWidth',1.6); grid on;
        xlabel('Time (s)'); ylabel('Torque (N·m)');
        title([label ' – Joint Torques']); legend(header(2:end),'Location','best');
    case 'separate'
        colors=lines(7);
        for k=1:7
            figure('Name',sprintf('%s J%d',label,k),'NumberTitle','off');
            plot(time,tau(:,k),'Color',colors(k,:),'LineWidth',1.6); grid on;
            xlabel('Time (s)'); ylabel(sprintf('%s%d (N·m)',label,k));
            title(sprintf('%s – Joint %d Torque',label,k));
        end
    otherwise
        error('plotOption must be ''all'' or ''separate''.');
end
end
