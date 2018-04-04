% This is a helper function and may be changed or removed without notice.

%   Copyright 2016 The MathWorks, Inc.

% Plots detections collected using helperCollectScenarioMetrics during
% simulation of a driving scenario.
%
% metrics:  struct array of metrics collected from a driving scenario using
%           helperCollectScenarioMetrics
% plotMetrics:  Type of metrics to plot. Can be one of: position, velocity,
%               snr
% indepAxis:    Specifies the values to use for the plot's independent
%               axis. Can be one of: range, reverse range, or time.
% axisLimits:   Optional. Specify the limits to apply to the axes of the
%               plots. Either a 1-by-2 or a 2-by-2 vector where the first
%               row defines the limits of the dependent axis and (when
%               provided) the second row defines the limits of the
%               independent axis
% showMerged:   Optional. When true, plot merged detections using a unique
%               color. This only applies to detections generated from a
%               radar sensor. Otherwise, do not plot merged detections
%               using a unique color.
function helperPlotSensorDemoDetections(metrics,plotMetrics,indepAxis,axisLimits,showMerged)

% Parse optional inputs
if nargin<4
    axisLimits = [];
end
if nargin<5
    showMerged = false;
end

% Create figure of detection plot. If figure already exists, close it and
% create a new one.
figMetrics = findall(0,'Type','Figure','Tag','SensorDemoMetrics');
if ~isempty(figMetrics)
    close(figMetrics);
end
figMetrics = figure('Name','Sensor Metrics','Tag','SensorDemoMetrics');

% For figures with subplots, set width to the maximum without truncation
% when printed, keep figure centered on screen
if ~strcmpi(plotMetrics(1),'s')
    % This is the maximum figure width that can be used for publishing
    % without clipping the subplots
    maxWidth = 1150;
    pos = figMetrics.Position;
    width = pos(3);
    figMetrics.Position = [pos(1)-(maxWidth-width)/2 pos(2) maxWidth pos(4)];
end

% Dispatch call according to the requested plot type
switch lower(plotMetrics(1))
    case 'p' % position
        plotPosition(metrics,indepAxis,axisLimits,showMerged);
    case 'v' % velocity
        plotVelocity(metrics,indepAxis,axisLimits);
    case 's' % snr
        plotSNR(metrics,indepAxis,axisLimits,showMerged);
end
end

% Computes longitudinal and lateral position errors from detections and
% ground truth metrics collected from a driving scenario simulation.
% Creates a subplot for the longitudinal errors and a subplot for the
% lateral errors and plots the errors in each subplot. Each target's errors
% are plotted using a unique color.
function plotPosition(metrics,indepAxis,axisLimits,showMerged)

% Create subplot for longitudinal position errors
hLong = subplot(1,2,1);
setupYAxis(hLong,indepAxis);
xlabel(hLong,'Error (m)');
title(hLong,'Longitudinal Position');
grid(hLong,'on');

% Create subplot for lateral position errors
hLat = subplot(1,2,2);
setupYAxis(hLat,indepAxis);
xlabel(hLat,'Error (m)');
title(hLat,'Lateral Position');
grid(hLat,'on');

% Get unique colors for each target's errors
numTgts = numel(metrics);
if showMerged
    % Last color is for merged detections
    clrs = getColors(numTgts+1);
else
    clrs = getColors(numTgts);
end

% Plot longitudinal position errors and longitudinal 2-sigma noise for each target
hold(hLong,'on');
for m = 1:numTgts
    groundTruth = metrics(m).GroundTruth;
    meas = metrics(m).Measurement;
    measSigma = metrics(m).MeasurementSigma;
    posError = meas-groundTruth;
    yVals = getIndepAxisVals(metrics(m),indepAxis);
    plot(hLong,posError(:,1),yVals,'.','Color',clrs(m,:));
    if numTgts>1 && mod(m,2)
        lineWidth = 3;
    else
        lineWidth = 2;
    end
    plot(hLong,2*[measSigma(:,1);NaN;-measSigma(:,1)],[yVals;NaN;yVals],...
        '-','Color',clrs(m,:),'LineWidth',lineWidth);
end
hold(hLong,'off');

% Plot lateral position errors and lateral 2-sigma noise for each target
hold(hLat,'on');
for m = 1:numTgts
    groundTruth = metrics(m).GroundTruth;
    meas = metrics(m).Measurement;
    measSigma = metrics(m).MeasurementSigma;
    posError = meas-groundTruth;
    yVals = getIndepAxisVals(metrics(m),indepAxis);
    plot(hLat,posError(:,2),yVals,'.','Color',clrs(m,:));
    if numTgts>1 && mod(m,2)
        lineWidth = 3;
    else
        lineWidth = 2;
    end
    plot(hLat,2*[measSigma(:,2);NaN;-measSigma(:,2)],[yVals;NaN;yVals],...
        '-','Color',clrs(m,:),'LineWidth',lineWidth);
end
hold(hLat,'off');

% Highlight merged detections
if showMerged
    
    groundTruth = metrics(1).GroundTruth;
    meas = metrics(1).Measurement;
    measSigma = metrics(1).MeasurementSigma;
    posError = meas-groundTruth;
    yVals = getIndepAxisVals(metrics(1),indepAxis);
    
    isMerged = findMerged(metrics);
    
    % Longitudinal errors
    hold(hLong,'on');
    plot(hLong,posError(isMerged,1),yVals(isMerged),'.','Color',clrs(end,:));
    plot(hLong,2*[measSigma(isMerged,1);NaN;-measSigma(isMerged,1)],[yVals(isMerged);NaN;yVals(isMerged)],...
        '--','Color',clrs(end,:),'LineWidth',2);
    hold(hLong,'off');
    
    % Lateral errors
    hold(hLat,'on');
    plot(hLat,posError(isMerged,2),yVals(isMerged),'.','Color',clrs(end,:));
    plot(hLat,2*[measSigma(isMerged,2);NaN;-measSigma(isMerged,2)],[yVals(isMerged);NaN;yVals(isMerged)],...
        '--','Color',clrs(end,:),'LineWidth',2);
    hold(hLat,'off');
end

% Sync error axis limits, apply dependent axis limits if specified
if isempty(axisLimits) % auto
    elim1 = xlim(hLong);
    elim2 = xlim(hLat);
    elim = [min([elim1 elim2]) max([elim1 elim2])];
else
    elim = axisLimits(1,:);
end
xlim(hLong,elim);
xlim(hLat,elim);

% Apply independent axis limts if specified
if size(axisLimits,1)>1
    ylim(hLong,axisLimits(2,:));
    ylim(hLat,axisLimits(2,:));
end
end

% Computes longitudinal and lateral velocity errors from detections and
% ground truth metrics collected from a driving scenario simulation.
% Creates a subplot for the longitudinal errors and a subplot for the
% lateral errors and plots the errors in each subplot. Each target's errors
% are plotted using a unique color.
function plotVelocity(metrics,indepAxis,axisLimits)

% Create subplot for longitudinal velocity errors
hLong = subplot(1,2,1);
setupYAxis(hLong,indepAxis);
xlabel(hLong,'Error (m/s)');
title(hLong,'Longitudinal Velocity');
grid(hLong,'on');

% Create subplot for lateral velocity errors
hLat = subplot(1,2,2);
setupYAxis(hLat,indepAxis);
xlabel(hLat,'Error (m/s)');
title(hLat,'Lateral Velocity');
grid(hLat,'on');

% Get unique colors for each target's errors
numTgts = numel(metrics);
clrs = getColors(numTgts);

% Plot longitudinal velocity errors and longitudinal 2-sigma noise for each target
hold(hLong,'on');
for m = 1:numTgts
    groundTruth = metrics(m).GroundTruth;
    meas = metrics(m).Measurement;
    measSigma = metrics(m).MeasurementSigma;
    posError = meas-groundTruth;
    yVals = getIndepAxisVals(metrics(m),indepAxis);
    plot(hLong,posError(:,4),yVals,'.','Color',clrs(m,:));
    if numTgts>1 && mod(m,2)
        lineWidth = 3;
    else
        lineWidth = 2;
    end
    plot(hLong,2*[measSigma(:,4);NaN;-measSigma(:,4)],[yVals;NaN;yVals],...
        '-','Color',clrs(m,:),'LineWidth',lineWidth);
end
hold(hLong,'off');

% Plot lateral velocity errors and longitudinal 2-sigma noise for each target
hold(hLat,'on');
for m = 1:numTgts
    groundTruth = metrics(m).GroundTruth;
    meas = metrics(m).Measurement;
    measSigma = metrics(m).MeasurementSigma;
    posError = meas-groundTruth;
    yVals = getIndepAxisVals(metrics(m),indepAxis);
    plot(hLat,posError(:,5),yVals,'.','Color',clrs(m,:));
    if numTgts>1 && mod(m,2)
        lineWidth = 3;
    else
        lineWidth = 2;
    end
    plot(hLat,2*[measSigma(:,5);NaN;-measSigma(:,5)],[yVals;NaN;yVals],...
        '-','Color',clrs(m,:),'LineWidth',lineWidth);
end
hold(hLat,'off');

% Sync error axis limits, apply dependent axis limits if specified
if isempty(axisLimits) % auto
    elim1 = xlim(hLong);
    elim2 = xlim(hLat);
    elim = [min([elim1 elim2]) max([elim1 elim2])];
else
    elim = axisLimits(1,:);
end
xlim(hLong,elim);
xlim(hLat,elim);

% Apply independent axis limts if specified
if size(axisLimits,1)>1
    ylim(hLong,axisLimits(2,:));
    ylim(hLat,axisLimits(2,:));
end
end

% Plot the signal-to-noise ratio (SRN) from the detections collected from a
% driving scenario simulation. Each target's SNR is plotted using a unique
% color.
function plotSNR(metrics,indepAxis,axisLimits,showMerged)

% Create plot for SNR. Set xlabel according to the values requested for the
% independent axis.
switch indepAxis(1)
    case 'r'
        xStr = 'Ground Truth Range (m)';
    case 't'
        xStr = 'Scenario Time (s)';
end
xlabel(xStr);
ylabel('SNR (dB)');
title('Detection Sensitivity');
grid on;

% Get unique colors for each target's errors
numTgts = numel(metrics);
if showMerged
    % Last color is for merged detections
    clrs = getColors(numTgts+1);
else
    clrs = getColors(numTgts);
end

% Plot each target's SNR
hold on;
for m = 1:numTgts
    detSNR = metrics(m).SNR;
    xVals = getIndepAxisVals(metrics(m),indepAxis);
    plot(xVals,detSNR,'.','Color',clrs(m,:));
end
hold off;

% Highlight merged detections
if showMerged
    hold on;

    detSNR = metrics(1).SNR;
    xVals = getIndepAxisVals(metrics(1),indepAxis);
    
    isMerged = findMerged(metrics);
    plot(xVals(isMerged),detSNR(isMerged),'.','Color',clrs(end,:));
    
    hold off;
end

% If specified, set the limits of the dependent axis
if ~isempty(axisLimits)
    xlim(axisLimits(1,:));
end

% If specified, set the limits of the independent axis
if size(axisLimits,1)>1
    ylim(axisLimits(2,:));
end
end

% Returns a unique color for each target
function clrs = getColors(numTgts)
clrs = lines(numTgts+1);
clrs = clrs(2:end,:); % First color is reserved for the ego vehicle
end

% Sets up the y-axis for the position and velocity error subplots according
% to the data type requested for the y-axis data.
function setupYAxis(hax, yType)

yDir = 'normal';
if contains(lower(yType),'reverse')
    yDir = 'reverse';
end

yType = strtrim(strrep(yType,'reverse',''));
switch yType(1)
    case 'r' % Ground truth range
        yStr = 'Ground Truth Range (m)';
    case 't' % Time
        yStr = 'Scenario Time (s)';
end

set(hax,'YDir',yDir);
ylabel(hax,yStr);
end

% Returns the requested values for the independent axis. The independent
% axis can be set to either ground truth range or time.
function vals = getIndepAxisVals(metrics,indepAxis)

indepAxis = strtrim(strrep(indepAxis,'reverse',''));
switch indepAxis(1)
    case 't' % Time
        vals = metrics.Time;
    case 'r' % Ground truth range
        vals = sqrt(sum(metrics.GroundTruth(:,1:3).^2,2));
end
end

% Finds merged detections by comparing SNR values. Assumes all merged
% detections are assigned to the first target, uses the 2nd target to
% generate the SNR v Range relationship. Also assumes both targets have the
% same RCS
function isMerged = findMerged(metrics)
    groundTruth = metrics(2).GroundTruth;
    snr = metrics(2).SNR;
    rg = sqrt(sum(groundTruth(:,1:3).^2,2)); % ground truth range
    coeffs = polyfit(rg,snr,4);
    
    groundTruth = metrics(1).GroundTruth;
    rg = sqrt(sum(groundTruth(:,1:3).^2,2)); % ground truth range
    snr = metrics(1).SNR;
    thrsh = polyval(coeffs,rg)+1;
    isMerged = snr>thrsh;
end