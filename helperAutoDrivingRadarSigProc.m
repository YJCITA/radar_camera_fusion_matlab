% This is a helper function and may be changed or removed without notice.

%   Copyright 2017 The MathWorks, Inc.

function varargout = helperAutoDrivingRadarSigProc(method,varargin)

persistent figHandle BEP rangeImageHandles metrics

varargout = {};
switch method
    case 'Setup Scenario'
        [scenario,egoCar,radarParams,tgts] = setupScenario(varargin{:});
        varargout = {scenario,egoCar,radarParams,tgts};
    case 'Initialize Display'
        [figHandle, BEP, rangeImageHandles] = initDisplay(varargin{:});
    case 'Update Display'
        updateDisplay(varargin{:},BEP, rangeImageHandles);
    case 'Run Simulation'
        clear metrics
        metrics = runSimulation(varargin{:});
        varargout = {metrics};
    case 'Plot Channels'
        plotChannels(varargin{:});
    case 'Collect Metrics'
        metrics = collectMetrics(metrics,varargin{:});
        if nargout>0
            varargout = {metrics};
        end
    case 'Publish Snapshot'
        publishSnapshot(figHandle,varargin{:});
    case 'Cluster Detections'
        clusterIDs = clusterDetections(varargin{:});
        varargout = {clusterIDs};
    case 'Estimate Angle'
        [angest, angvar, snrdB] = doaestimator(varargin{:});
        varargout = {angest, angvar, snrdB};
    case 'Array Beamwidth'
        bw = arraybeamwidth(varargin{:});
        varargout = {bw};
end
end

function [scenario,egoCar,radarParams,tgts]= setupScenario(c,fc)

% Create a driving scenario
scenario = drivingScenario;
scenario.SampleTime = 0.05; % s
scenario.StopTime = 2;

% Create road
laneWidth = 3.6;
roadCenters = [0 0; 50 0; 100 0; 250 20; 500 40];

% Six lanes, each 3.6 meters
dy = laneWidth*1.05;
for m = -2:3
    road(scenario, roadCenters+[0 m*dy], laneWidth);
end

initialdist = 100+[0; -1; 20; 60];
lane = [0; 3; 1; -2]*dy;
initialspd = [80; 110; 100; 130]*1000/3600; % km/hr -> m/s
egoCar = addCars(scenario,roadCenters,initialdist,lane,initialspd);

radarPos = [3.4 0 0.2]';
radarYaw = 0;
radarPitch = 0;
radarRoll = 0;
radarAxes = rotz(radarYaw)*roty(radarPitch)*rotx(radarRoll);
radarParams = struct( ...
    'Frame', drivingCoordinateFrameType.Spherical, ...
    'OriginPosition',radarPos, ...
    'OriginVelocity',zeros(3,1), ...
    'Orientation', radarAxes, ...
    'HasElevation', false, ...
    'HasVelocity', true, ...
    'RMSBias',[0 0.25 0.05]);

tgts = createPointTargets(scenario,c,fc);
end

function [hFigure, BEP, hndls] = initDisplay(egoCar, radarParams, rx_array, fc, vmax, frontLim)

% Make a figure
hFigure = findobj('Tag',mfilename);
if isempty(hFigure)
    hFigure = figure('Name', 'Automated Driving', 'Tag', mfilename, 'Visible','off');
    set(hFigure,'Position', [0, 0, 1032, 600]); % This is the maximum figure size that will print without being clipped
    movegui(hFigure , [0 -1]) ;
end
clf(hFigure);
hFigure.Visible = 'off';

% Add a car plot that follows the ego vehicle from behind
hCarViewPanel = uipanel(hFigure, 'Position', [0 2/3 1/2 1/3], 'Title', 'Chase Camera View');
hCarPlot = axes(hCarViewPanel);
chasePlot(egoCar, 'Parent', hCarPlot);

% Add a panel for a bird's-eye plot
hBEVPanel = uipanel(hFigure, 'Position', [1/2 0 1/2 1], 'Title', 'Bird''s-Eye Plot');

% Add a panel for range doppler plot
hDRPanel = uipanel(hFigure, 'Position', [0 1/3 1/2 1/3], 'Title', 'Range-Doppler ');
hdrange = axes(hDRPanel);

% Add a panel for range-angle of arrival plot
hRAOAPanel = uipanel(hFigure, 'Position', [0 0 1/2 1/3], 'Title', 'Range-Angle of Arrival');
hraoa= axes(hRAOAPanel);

% Create bird's-eye plot for the ego car and sensor coverage
% frontLim = 100;
backLim = -20;
hBEVPlot = axes(hBEVPanel);
BEP = birdsEyePlot('Parent', hBEVPlot, 'Xlimits', [-10 frontLim], 'Ylimits', [-35 35]);

% Combine all radar detections into one entry and store it for later update
detectionPlotter(BEP,'DisplayName','Radar Detections','MarkerEdgeColor','red','MarkerFaceColor','red');

trackPlotter(BEP,'DisplayName','Tracks','HistoryDepth',10);

% Add road borders to plot
laneBoundaryPlotter(BEP, 'DisplayName','road', 'Color', 0.75*[1 1 1]);
xlim(BEP.Parent, [backLim frontLim]);
ylim(BEP.Parent, [-20 20]);

% Add an outline plotter for ground truth
outlinePlotter(BEP, 'Tag', 'Ground truth');

% Add receive array radiation pattern
theta = [-90:90]';
ant_patt = patternAzimuth(rx_array, fc,0 , 'Azimuth', theta');
ant_patt_pow = db2pow(ant_patt) ;
hold on (hBEVPlot)

scale = 8;
plot(hBEVPlot,radarParams.OriginPosition(1)+scale*ant_patt_pow.*cosd(theta),...
    radarParams.OriginPosition(2)+scale*ant_patt_pow.*sind(theta), 'r')
hold off

% Initialize plot parameters for Range-Doppler response
hi_rng_dopp = imagesc(hdrange,[-50 50],[backLim frontLim], zeros(2,2));
hdrange.YDir = 'normal';
ylim(hdrange,[backLim frontLim])
xlim(hdrange,[-vmax vmax]*3600/1000); % m/s -> km/hr
caxis(hdrange,[10 60])
ylabel(colorbar('peer',hdrange),'SNR (dB)');
xlabel(hdrange,'Radial Speed (km/hr)')
ylabel(hdrange,'Range (m)')
title(hdrange,'Range-Doppler Image')

% Initialize plot parameters for Doppler-Angle of Arrival response
hi_rng_aoa = imagesc(hraoa,[backLim frontLim],[-60 60], zeros(2,2));
hraoa.YDir = 'normal';
ylim(hraoa,[backLim frontLim])
xlim(hraoa,[-80 80])
caxis(hraoa,[10 60])
ylabel(colorbar('peer',hraoa),'SNR (dB)');
xlabel(hraoa,'Angle of Arrival (deg)')
ylabel(hraoa,'Range (m)')
title(hraoa,'Range-Angle Image' )

hndls = [hi_rng_dopp, hi_rng_aoa];
end

function updateDisplay(egoCar, dets, tracks, dopgrid, rnggrid, Xbf, beamscan, Xrngdop, BEP, rngImagHndls)

% Check if display is still open
if ~ishghandle(rngImagHndls(1))
    return % no, nothing to update
end

if ~isPublishing()
    hfig = get(get(get(rngImagHndls(1),'Parent'),'Parent'),'Parent');
    hfig.Visible = 'on';
end

% Update road boundaries and their display
rb = roadBoundaries(egoCar);
plotLaneBoundary(findPlotter(BEP,'DisplayName','road'),rb)

% Update ground truth data
[position, yaw, length, width, originOffset, color] = targetOutlines(egoCar);
plotOutline(findPlotter(BEP,'Tag','Ground truth'), position, yaw, length, width, 'OriginOffset', originOffset, 'Color', color);

% Get detections in ego coordinate frame for bird's-eye plot
numDets = numel(dets);
pos = NaN(numDets,2);
lbls = cell(1,numDets);
for iDet = 1:numDets
    meas = dets{iDet}.Measurement;
    params = dets{iDet}.MeasurementParameters{1};
    [x,y,z] = sph2cart(meas(1)*pi/180,0,meas(2));
    Xradar = [x;y;z];
    Xego = params.Orientation*Xradar+params.OriginPosition;
    pos(iDet,:) = Xego(1:2);
    lbls{iDet} = sprintf('   %1.1f dB\n', dets{iDet}.ObjectAttributes{1}.SNR);
end

if numDets>0
    plotDetection(findPlotter(BEP,'DisplayName','Radar Detections'),pos,lbls);
end

positionSelector = [1 0 0 0 0 0; 0 0 1 0 0 0]; % Position selector
velocitySelector = [0 1 0 0 0 0; 0 0 0 1 0 0]; % Velocity selector
[tracksPos, tracksCov] = getTrackPositions(tracks, positionSelector);
tracksVel = getTrackVelocities(tracks, velocitySelector);
trackIDs = {tracks.TrackID};
labels = cellfun(@(x)sprintf('\n   ID%i',x), trackIDs, 'UniformOutput', false);
plotTrack(findPlotter(BEP,'DisplayName','Tracks'), tracksPos, tracksVel, tracksCov, labels);

Xpow = abs(Xbf).^2;
noisefloor = db2pow(-85);
Xsnr = pow2db(Xpow/noisefloor);

hRngDopPlot = rngImagHndls(1);
hRngDopPlot.XData = dopgrid*3600/1000; % m/s -> km/hr
hRngDopPlot.YData = rnggrid;
hRngDopPlot.CData = Xsnr;

[Nr,Ne,Nd] = size(Xrngdop);
Xrngdop = permute(Xrngdop,[1 3 2]); % Nr x Nd x Ne
Xrngdop = reshape(Xrngdop,[],Ne);
win = taylorwin(Ne,5,-60);
win = win/norm(win);
Xrngang = beamscan(Xrngdop.*win');
Xrngang = reshape(Xrngang,Nr,Nd,[]); % Nr x Nd x Ns
Xrngang = permute(Xrngang,[1 3 2]); % Nr x Ns x Nd
Xpow = abs(Xrngang).^2;
Xpow = max(Xpow,[],3);
Xsnr = pow2db(Xpow/noisefloor);

hRngAngPlot = rngImagHndls(2);
hRngAngPlot.XData = beamscan.Direction(1,:);
hRngAngPlot.YData = rnggrid;
hRngAngPlot.CData = Xsnr;
end

function metrics = runSimulation(c,fc,rangeMax,vMax,waveform,Nsweep,...
    transmitter,radiator,collector,receiver,...
    rngdopresp,beamformer,cfar,idxCFAR,...
    rngestimator,dopestimator,doaest,beamscan,tracker,...
    txchannel,rxchannel)

rng(2017);

reset(transmitter);
reset(radiator);
reset(collector);
reset(receiver);
reset(rngestimator);
reset(dopestimator);
reset(doaest);
reset(beamscan);
reset(tracker);

fs = waveform.SampleRate;

if nargin==20
    if isprop(txchannel,'TwoWayPropagation') && txchannel.TwoWayPropagation
        txchannel.TwoWayPropagation = false;
    end
    rxchannel = clone(txchannel);
elseif nargin<20
    txchannel = phased.FreeSpace('PropagationSpeed',c,...
        'OperatingFrequency',fc,'SampleRate',fs,'TwoWayPropagation',false);
    rxchannel = clone(txchannel);
end
reset(txchannel);
reset(rxchannel);

[scenario,egoCar,radarParams,pointTgts] = helperAutoDrivingRadarSigProc('Setup Scenario',c,fc);

Nft = waveform.SweepTime*waveform.SampleRate;
Ne = collector.Sensor.NumElements;
Nr = rngdopresp.RangeFFTLength;
Nd = rngdopresp.DopplerFFTLength;

rxArray = collector.Sensor;

% Initialize display for driving scenario example
helperAutoDrivingRadarSigProc('Initialize Display', egoCar, radarParams, rxArray, fc, vMax, rangeMax);

tgtProfiles = actorProfiles(scenario);
tgtProfiles = tgtProfiles(2:end);
tgtHeight = [tgtProfiles.Height];

sweepTime = waveform.SweepTime;
while advance(scenario)
    
    % Get the current scenario time
    time = scenario.SimulationTime;
    
    % Get current target poses in ego car's reference frame
    tgtPoses = targetPoses(egoCar);
    tgtPos = reshape([tgtPoses.Position],3,[]);
    tgtPos(3,:) = tgtPos(3,:)+0.5*tgtHeight; % Point targets are positioned at half of each target's height
    tgtVel = reshape([tgtPoses.Velocity],3,[]);
    
    % Simulate the radar transmit and receive operation
    Xcube = zeros(Nft, Ne, Nsweep);
    
    for m = 1:Nsweep
        
        % Calculate the target angles as seen by the sensor
        tgtAngs = NaN(2,numel(tgtPoses));
        for iTgt = 1:numel(tgtPoses)
            tgtAxes = rotz(tgtPoses(iTgt).Yaw)*roty(tgtPoses(iTgt).Pitch)*rotx(tgtPoses(iTgt).Roll);
            [~,tgtAngs(:,iTgt)] = rangeangle(radarParams.OriginPosition,tgtPos(:,iTgt),tgtAxes);
        end
        
        % Transmit FMCW waveform
        sig = waveform();
        [~, txang] = rangeangle(tgtPos, radarParams.OriginPosition, radarParams.Orientation);
        txsig = transmitter(sig);
        txsig = radiator(txsig,txang);
        
        % Propagate the signal and reflect off the targets
        txsig = txchannel(txsig,radarParams.OriginPosition,tgtPos,radarParams.OriginVelocity,tgtVel);
        txsig = pointTgts(txsig,tgtAngs);
        txsig = rxchannel(txsig,radarParams.OriginPosition,tgtPos,radarParams.OriginVelocity,tgtVel);
        
        % Collect received target echos
        rxsig = collector(txsig, txang);
        rxsig = receiver(rxsig);
        
        % Dechirp the received signal
        rxsig = dechirp(rxsig, sig);
        
        % Save sweep to data cube
        Xcube(:,:,m) = rxsig;
        
        % Move targets forward for next sweep
        tgtPos = tgtPos+tgtVel*sweepTime;
    end
    
    % Calculate the range-doppler response
    [Xrngdop,rnggrid,dopgrid] = rngdopresp(Xcube);
    
    % Beamform received data
    Xbf = permute(Xrngdop,[1 3 2]);
    Xbf = reshape(Xbf,Nr*Nd,Ne);
    Xbf = beamformer(Xbf);
    Xbf = reshape(Xbf,Nr,Nd);
    
    % Detect targets
    Xpow = abs(Xbf).^2;
    [detidx,noisepwr] = cfar(Xpow,idxCFAR);
    
    % Cluster detections
    clusterIDs = helperAutoDrivingRadarSigProc('Cluster Detections', detidx);
    
    % Azimuth, range, and range rate measurements
    [azest,azvar,snrdB] = helperAutoDrivingRadarSigProc('Estimate Angle', doaest, conj(Xrngdop), Xbf, detidx, noisepwr, clusterIDs);
    azvar = azvar+radarParams.RMSBias(1)^2;
    
    [rngest,rngvar] = rngestimator(Xbf,rnggrid,detidx,noisepwr,clusterIDs);
    rngvar = rngvar+radarParams.RMSBias(2)^2;
    
    [rsest,rsvar] = dopestimator(Xbf,dopgrid,detidx,noisepwr,clusterIDs);
    
    % Radial speed is estimated, but the tracker expects range rate.
    % Convert radial speed to range rate.
    rrest = -rsest;
    rrvar = rsvar;
    rrvar = rrvar+radarParams.RMSBias(3)^2;
    
    % Assemble object detections for use by tracker
    numDets = numel(rngest);
    dets = cell(numDets,1);
    for iDet = 1:numDets
        dets{iDet} = objectDetection(time, [azest(iDet) rngest(iDet) rrest(iDet)]', ...
            'MeasurementNoise', diag([azvar(iDet) rngvar(iDet) rrvar(iDet)]), ...
            'MeasurementParameters', {radarParams}, ...
            'ObjectAttributes', {struct('SNR',snrdB(iDet))});
    end
    
    % Track detections
    tracks = tracker(dets,time);
    
    % Update bird's-eye plot
    helperAutoDrivingRadarSigProc('Update Display', egoCar, dets, tracks, dopgrid, rnggrid,...
        Xbf, beamscan, Xrngdop);
    
    % Publish snapshot
    helperAutoDrivingRadarSigProc('Publish Snapshot', time>=1.1);
    
    % Collect channel metrics
    metrics = helperAutoDrivingRadarSigProc('Collect Metrics', radarParams, tgtPos, tgtVel, dets);
end
end

function plotChannels(metricsFS, metrics2Ray)
hfig = findobj('Tag','Compare Propagation Channels');
if isempty(hfig)
    hfig = figure('Name','Channel Propagation','Tag','Channel Propagation');
end
figure(hfig);
clf(hfig); hold on;
title('Propagation Channels');
xlabel('Measured Radar Range (m)');
ylabel('CFAR SNR Estimate (dB)');
grid on;

% Only plot metrics for long range car
[~,y] = pol2cart(metricsFS.AzimuthEst*pi/180,metricsFS.RangeEst);
iFnd = y<0;
plot(metricsFS.RangeEst(iFnd),metricsFS.SNREst(iFnd),'o');

% Only plot metrics for long range car
[~,y] = pol2cart(metrics2Ray.AzimuthEst*pi/180,metrics2Ray.RangeEst);
iFnd = y<0;
plot(metrics2Ray.RangeEst(iFnd),metrics2Ray.SNREst(iFnd),'^');

legend('Free space','Two-ray');
end

function metrics = collectMetrics(metrics, radarParams, tgtPos, tgtVel, dets)
% Collects metrics from the simulated driving scenario. Returns the current
% collected metrics.

if isempty(metrics)
    metrics = struct( ...
        'Position',[],...
        'Velocity',[],...
        'Azimuth',[],...
        'Range',[],...
        'RadialSpeed',[],...
        'AzimuthEst',[],...
        'AzimuthVar',[],...
        'RangeEst',[],...
        'RangeVar',[],...
        'RadialSpeedEst',[],...
        'RadialSpeedVar',[],...
        'SNREst',[]);
end

% Compute ground truth metrics
gtspd = radialspeed(tgtPos,tgtVel);
[gtrng, gtang] = rangeangle(tgtPos, radarParams.OriginPosition, radarParams.Orientation);
gtaz = gtang(1,:)';

% Measurements
numDets = numel(dets);
azest = NaN(numDets,1);
azvar = NaN(numDets,1);
rngest = NaN(numDets,1);
rngvar = NaN(numDets,1);
rsest = NaN(numDets,1);
rsvar = NaN(numDets,1);
snrdB = NaN(numDets,1);
for m = 1:numDets
    azest(m) = dets{m}.Measurement(1);
    azvar(m) = dets{m}.MeasurementNoise(1,1);
    rngest(m) = dets{m}.Measurement(2);
    rngvar(m) = dets{m}.MeasurementNoise(2,2);
    rsest(m) = dets{m}.Measurement(3);
    rsvar(m) = dets{m}.MeasurementNoise(3,3);
    snrdB(m) = dets{m}.ObjectAttributes{1}.SNR;
end

metrics.Position = [metrics.Position; tgtPos];
metrics.Velocity = [metrics.Position; tgtVel];
metrics.Azimuth = [metrics.Azimuth; gtaz(:)];
metrics.Range = [metrics.Range; gtrng(:)];
metrics.RadialSpeed = [metrics.RadialSpeed; gtspd(:)];
metrics.AzimuthEst = [metrics.AzimuthEst; azest(:)];
metrics.AzimuthVar = [metrics.AzimuthVar; azvar(:)];
metrics.RangeEst = [metrics.RangeEst; rngest(:)];
metrics.RangeVar = [metrics.RangeVar; rngvar(:)];
metrics.RadialSpeedEst = [metrics.RadialSpeedEst; rsest(:)];
metrics.RadialSpeedVar = [metrics.RadialSpeedVar; rsvar(:)];
metrics.SNREst = [metrics.SNREst; snrdB(:)];
end

function publishSnapshot(fig, takeSnapshot)
% When publishing, function takes one snapshot of the figure fig when
% takeSnapshot is true and then closes the figure so that another snapshot
% will not be generated from it. Does nothing when not publishing.

if ishghandle(fig)
    hasSnapshot = takeSnapshot && isPublishing();
    if hasSnapshot
        figure(fig);
        fig.Visible = 'on';
        snapnow;
        close(fig);
    end
end
end

% Helper functions

function egoCar = addCars(scenario,roadcenters,initialdist,lane,initialspd)
% Adds all of the cars defined for the driving scenario and returns the
% vehicle object corresponding to the ego vehicle.

waypoints = interpWaypointsFromRoadCenters(roadcenters);
egoCar = addCar(scenario,waypoints,initialdist(1),lane(1),initialspd(1));
numCars = numel(initialspd);
for m = 2:numCars
    addCar(scenario,waypoints,initialdist(m),lane(m),initialspd(m));
end
end

function car = addCar(scenario,waypoints,initialdist,lane,initialspd)
% Adds a car and its path to the driving scenario. Returns the added
% vehicle object.

pathlength = [0;cumsum(sqrt(sum(diff(waypoints,1,1).^2,2)))];

car = vehicle(scenario);

ind = pathlength>=initialdist;
waypoints = waypoints(ind,:);
vecs = diff(waypoints,1,1);
nm = sqrt(sum(vecs.^2,2));
vecs = vecs./nm;
vecs = vecs([1 1:end],:);
vecs = [-vecs(:,2) vecs(:,1)];
waypoints = waypoints+vecs*lane;
path(car, waypoints, initialspd);
end

function tgts = createPointTargets(scenario,c,fc)
% Returns point targets modeled using the |phased.BackscatterRadarTarget|
% object. The point target models are loaded from the point target models
% stored in the driving scenario's actor profiles.

profiles = actorProfiles(scenario);
profiles = profiles(2:end); % First profile is the ego car, not a target
numTgts = numel(profiles);

azAngles = profiles(2).RCSAzimuthAngles;
elAngles = profiles(2).RCSElevationAngles;
rcsPatterns = NaN(numel(elAngles),numel(azAngles),numTgts);
for m = 1:numTgts
    rcsPatterns(:,:,m) = db2mag(profiles(m).RCSPattern);
end
tgts = phased.BackscatterRadarTarget( ...
    'RCSPattern', rcsPatterns, ...
    'AzimuthAngles', azAngles, ...
    'ElevationAngles', elAngles, ...
    'PropagationSpeed', c,...
    'OperatingFrequency', fc);
end

function waypoints = interpWaypointsFromRoadCenters(roadCenters)
% Returns waypoints corresponding to the interpolated road centers, where
% the interpolation between the road centers is performed by
% drivingScenario.

num = size(roadCenters,2);
s = drivingScenario;
road(s, roadCenters, 1e-6);
rb = roadBoundaries(s);
rb = rb{1};
waypoints = [];
while ~isempty(rb)
    pt = rb(1,:);
    d = sqrt(sum((rb-pt).^2,2));
    ind = d<1e-4;
    waypoints = [waypoints;mean(rb(ind,:),1)]; %#ok<AGROW>
    rb(ind,:) = [];
end

waypoints = waypoints(:,1:num);
end

function clusterIDs = clusterDetections(detidx)
% clusterIDs = clusterDetections(dets) clusters detections from a
% range-Doppler image. Detections which occur at adjacent range and Doppler
% cells within the image are associated to a single detection cluster. Each
% detection cluster is assigned a unique cluster ID which is used to
% identify the detections assigned to that cluster.
%
% detidx is a 2-by-L matrix of detection indices. Each column of detidx
% identifies the range and Doppler cell in the range-Doppler image where a
% detection was found as [rngidx; dopidx].
%
% clusterIDs is a 1-by-L vector of cluster IDs assigned to each detection
% index in detidx.

% Number of detections found in the range-Doppler image
numDet = size(detidx,2);
clusterIDs = NaN(1,numDet);

iGd = ~isnan(detidx(1,:));
if any(iGd(:))
    % Cluster adjacent points
    iRng = detidx(1,iGd);
    iDop = detidx(2,iGd);
    
    iGroups = cell(1,0);
    allCells = [iRng;iDop];
    while ~all(isnan(allCells(1,:)))
        % Select an unassigned range-Doppler cell and remove it from the set
        iFnd = find(~isnan(allCells(1,:)),1);
        thisCell = allCells(:,iFnd);
        allCells(:,iFnd) = NaN;
        
        % Find all cells adjacent to the selected cell and remove them from set
        [iAdjCells,allCells] = findAdjacentCells(allCells,thisCell);
        
        iGroups{end+1} = [iFnd iAdjCells]; %#ok<AGROW>
    end
    
    % Assign unique cluster IDs to each group of adjacent range-Doppler
    % cells
    ids = NaN(sum(iGd),1);
    for m = 1:numel(iGroups)
        ids(iGroups{m}) = m;
    end
    clusterIDs(iGd) = ids';
end
end

function [iAdjCells,allCells] = findAdjacentCells(allCells,thisCell)
% [iAdjCells,allCells] = findAdjacentCells(allCells,thisCell) finds the
% cells in the 2-by-L matrix of allCells that are adjacent to the 2-by-1
% vector for the current cell, thisCell. Both allCells and thisCell contain
% indices of the rows and columns of a matrix.
%
% iAdjCells is a 2-by-M matrix of cells in allCells that are adjacent to
% thisCell. M is less-than or equal-to L.
%
% allCells is returned as a 2-by-L matrix, where the adjacent cells
% returned in iAdjCells have been set to NaN.

% Find all cells adjacent to the current cell and remove it from the set
delta = allCells-repmat(thisCell,[1 size(allCells,2)]);
iAdjCells = find(all(abs(delta)<=2,1));
thisCell = allCells(:,iAdjCells);
allCells(:,iAdjCells) = NaN;

% Find call cells next to each of the adjacent cells that were just found
for m = 1:numel(iAdjCells)
    [iAdjNext,allCells] = findAdjacentCells(allCells,thisCell(:,m));
    iAdjCells = [iAdjCells iAdjNext]; %#ok<AGROW>
end
end

function [angest, angvar, snrdB] = doaestimator(doa,Xrngdop,Xbf,detidx,noisepwr,clusterIDs)
% [angest, angvar, snrdB] = doaestimator(doa,Xrngdop,Xbf,detidx,noisepwr,clusterIDs)
% returns the estimated angles for the detections generated from a linear
% phased array.
%
% doa is the phased.RootMUSICEstimator object used to estimate the
% direction-of-arrival of signals detected on a linear phased array.
%
% Xrngdop is a Nr-by-Ne-by-Nd date cube containing the range-Doppler
% processed data received by the Ne elements of the linear phased array.
%
% Xbf is the Nr-by-Nd beamformed range-Doppler image used to identify
% detections in the Xrngdop data cube.
%
% detidx is a 2-by-L matrix of detection indices generated from the
% beamformed range-Doppler image, Xbf. Each column of detidx identifies the
% range and Doppler cell in the range-Doppler image where a detection was
% found as [rngidx; dopidx].
%
% noisepwr is a 1-by-L vector of the estimated noise powers at each
% detection location reported in detidx.
%
% clusterIDs is a 1-by-L vector of cluster IDs assigned to each detection
% index in detidx.
%
% angest is an N-by-1 vector of angle estimates generated from the
% detection indices and clusterIDs. If clusterIDs is not provided, each
% detection is assigned its own angle estimate. Angle estimates are
% reported in degrees.
%
% angvar is an N-by-1 vector of variances corresponding to each of the
% angle estimates reported in angest. The angle variances are reported in
% degrees-squared (deg^2).
%
% snrdB is an N-by-1 vector of the signal-to-noise ratios (SNR) estimated
% of each detection in detidx. SNR is reported in decibels (dB).

persistent bw

if isempty(bw)
    bw = arraybeamwidth(doa);
end

% If not provided, assign each detection to its own cluster
numDet = size(detidx,2);
if nargin<4
    clusterIDs = (1:numDet);
    clusters = clusterIDs;
    numEst = numDet;
else
    clusters = unique(clusterIDs(~isnan(clusterIDs)));
    numEst = length(clusters);
end

angest = NaN(numEst,1);
angvar = NaN(numEst,1);
snrdB = NaN(numEst,1);

Nr = size(Xrngdop,1);

hasDoppler = ndims(Xrngdop)==3;
if hasDoppler
    Xrngdop = permute(Xrngdop,[1 3 2]); % Nr x Nd x Ne
    Nd = size(Xrngdop,2);
    Nelmnt = size(Xrngdop,3);
else
    Nd = 1;
    Nelmnt = size(Xrngdop,2);
end

Xrngdop = reshape(Xrngdop,[],Nelmnt);

for m = 1:numEst
    % Select detections in this cluster
    thisCluster = clusterIDs==clusters(m);
    iRng = detidx(1,thisCluster);
    iDop = detidx(2,thisCluster);
    idx = sub2ind([Nr Nd],iRng,iDop);
    
    y = Xrngdop(idx,:);
    angest(m) = doa(y);
    
    [pow,iMax] = max(abs(Xbf(idx)).^2);
    noise = noisepwr(thisCluster);
    snr = pow/noise(iMax);
    sigma = bw/(1.6*sqrt(2*snr));
    angvar(m) = sigma.^2;
    
    snrdB(m) = pow2db(snr);
end
end

function bw = arraybeamwidth(doa,c,fc)
% bw = arraybeamwidth(doa) returns the half-power (3dB) beamwidth estimated
% from the linear phased array used by doa.
%
% doa is the phased.RootMUSICEstimator object used to estimate the
% direction-of-arrival of signals detected on a linear phased array.
%
% bw is the estimated half power beamwidth in degrees.

if isprop(doa,'SensorArray')
    array = doa.SensorArray;
    fc = doa.OperatingFrequency;
    c = doa.PropagationSpeed;
else
    array = doa;
end

ang = linspace(-90,90);
pat = patternAzimuth(array,fc,0,'Type','powerdb','PropagationSpeed',c,'Azimuth',ang);
pk = max(pat);
iPk1 = find(pat>=pk-3,1,'first');
iPk2 = find(pat>=pk-3,1,'last');
bw = abs(ang(iPk2)-ang(iPk1));
end

function flag = isPublishing
% Returns true if the example is currently publishing
flag = ~isempty(snapnow('get'));
end
