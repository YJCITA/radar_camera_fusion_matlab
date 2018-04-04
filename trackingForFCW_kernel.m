function [confirmedTracks, egoLane, numTracks, mostImportantObject] = ...
    trackingForFCW_kernel(visionObjects, radarObjects, inertialMeasurementUnit, ...
    laneReports, egoLane, time, positionSelector, velocitySelector)
% This function is for demo purposes only and may be removed in the future

% trackingForFCW_kernel A function that contains all the tracking related
% functions required for the
% <matlab:web(fullfile(docroot,'driving','examples','forward-collision-warning-using-vision-and-radar-sensors.html'));
% Forward Collision Warning Using Vision and Radar Sensors> example.
% 
% Inputs:
%   visionObjects - a struct containing 10 vision objects.
%   radarObjects  - a struct containing 36 radar objects.
%   inertialMeasurementUnit - a struct with speed and yaw rate of ego car.
%   laneReports - a struct with parameters of the left and right lane boundaries.
%   egoLane - parameters of the left and right lane boundaries of the ego lane.
%   time - the time to which the tracker is updated.
%   positionSelector - a matrix that selects the position from the state vector
%   velocitySelector - a matrix that selects the velocity from the state vector
%
% Outputs:
%   confirmedTracks - a struct array with the tracks confirmed by the tracker.
%   egoLane - updated parameters of the ego lane boundaries.
%   numTracks - the total number of tracks maintained by the tracker
%   mostImportantObject - the track index and ID of the Most Important Object
%                      (MIO) and the level of Forward Collision Warning (FCW)

%#codegen
%   Copyright 2016 The MathWorks, Inc.

%% Tracker Construction and Update
% Define the tracker as a persistent variable to preserve its state between
% calls to this function
persistent tracker

% Create objectDetection inputs to the tracker. Please open the function to
% see memory allocation for code generation.
[detections, egoLane] = processDetections(visionObjects, radarObjects, ...
    inertialMeasurementUnit, laneReports, egoLane, time);

% On the first call to the function, tracker is still not defined. The
% following lines of code will define it. Note the use of the first
% detection as a SampleDetection. When generating code, the
% multiObjectTracker requires a SampleDetection to allocate memory for
% detections and tracks on construction.
if isempty(tracker)
    tracker = multiObjectTracker(...
        'FilterInitializationFcn', @initConstantAccelerationFilter, ...
        'AssignmentThreshold', 35, 'ConfirmationParameters', [2 3], ...
        'NumCoastingUpdates', 5);
end

% Using the list of objectDetections, return the confirmed tracks, updated
% to time.
confirmedTracks = updateTracks(tracker, detections, time);
numTracks = tracker.NumTracks;

% Find the most important object and calculate the forward collision warning    
mostImportantObject = findMostImportantObject(confirmedTracks, egoLane, positionSelector, velocitySelector);
end

%% Defining a Kalman Filter
function filter = initConstantAccelerationFilter(detection)
% This function shows how to configure a constant acceleration filter. The
% input is an objectDetection and the output is a tracking filter.
% For clarity, this function shows how to configure a trackingKF,
% trackingEKF, or trackingUKF for constant acceleration.
%
% Things to consider when creating a filter:
%   1. Defining the motion model and state
%   2. Defining the process noise
%   3. Defining the measurement model
%   4. Mapping the sensor measurements to an initial state vector
%   5. Mapping the sensor measurement noise to a state covariance
%   6. Creating the correct filter

    % Step 1: Defining the motion model and state
    % In this example, we want to use constant acceleration so:
    STF = @constacc;     % State-transition function, for EKF and UKF
    STFJ = @constaccjac; % State-transition function Jacobian, only for EKF
    % The motion model implies that the state is [x;vx;ax;y;vy;ay]
    % You could also use constvel and constveljac to setup a constant
    % velocity model, constturn and constturnjac to setup a constant turn
    % rate model, or write your own models.

    % Step 2: Defining the process noise
    dt = 0.05; % Known timestep size
    sigma = 1; % Magnitude of the unknown acceleration change rate
    % The process noise along one dimension
    Q1d = [dt^4/4, dt^3/2, dt^2/2; dt^3/2, dt^2, dt; dt^2/2, dt, 1] * sigma^2;
    Q = blkdiag(Q1d, Q1d); % 2-D process noise

    % Step 3: Defining the measurement model    
    MF = @fcwmeas;       % Measurement function, for EKF and UKF
    MJF = @fcwmeasjac;   % Measurement Jacobian function, only for EKF

    % Step 4: Initializing the state vector based on the measurement. 
    % The sensors measure [x;vx;y;vy] while the constant acceleration
    % model's state is [x;vx;ax;y;vy;ay], so the third and sixth elements
    % of the state vector are initialized to zero.
    state = [detection.Measurement(1); detection.Measurement(2); 0; detection.Measurement(3); detection.Measurement(4); 0];

    % Step 5: Initialize the state covariance based on the measurement
    % noise. The parts of the state that are not directly measured are
    % assigned a large measurement noise value to account for that.
    L = 100; % A large number relative to the measurement noise
    stateCov = blkdiag(detection.MeasurementNoise(1:2,1:2), L, detection.MeasurementNoise(3:4,3:4), L);

    % Step 6: Creating the correct filter. 
    % Use 'KF' for trackingKF, 'EKF' for trackingEKF, or 'UKF' for trackingUKF
    FilterType = 'EKF';

    % Creating the filter:
    switch FilterType
        case 'EKF'
            filter = trackingEKF(STF, MF, state,...
                'StateCovariance', stateCov, ...
                'MeasurementNoise', detection.MeasurementNoise(1:4,1:4), ...
                'StateTransitionJacobianFcn', STFJ, ...
                'MeasurementJacobianFcn', MJF, ...
                'ProcessNoise', Q ...            
                );
        case 'UKF'
            filter = trackingUKF(STF, MF, state, ...
                'StateCovariance', stateCov, ...
                'MeasurementNoise', detection.MeasurementNoise(1:4,1:4), ...
                'Alpha', 1e-1, ...            
                'ProcessNoise', Q ...
                );
        case 'KF' % The ConstantAcceleration model is linear and KF can be used                
            % Define the measurement model: measurement = H * state
            % In this case:
            %   measurement = [x;vx;y;vy] = H * [x;vx;ax;y;vy;ay]
            % So, H = [1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 0 1 0 0; 0 0 0 0 1 0]
            %
            % Note that ProcessNoise is automatically calculated by the
            % ConstantAcceleration motion model
            H = [1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 0 1 0 0; 0 0 0 0 1 0];
            filter = trackingKF('MotionModel', '2D Constant Acceleration', ...
                'MeasurementModel', H, 'State', state, ...
                'MeasurementNoise', detection.MeasurementNoise(1:4,1:4), ...
                'StateCovariance', stateCov);
    end
end

%% Memory Allocation, Detection Processing and Formatting
% The recorded information has to be processed and formatted before it can
% be used by the tracker. This includes the following steps:
%
% # Allocating memory for the list objectDetection inputs to the tracker.
%   Code generation requires that we define exactly the number of objects,
%   as well the data types and sizes of each property. We have to use the
%   data types and sizes in the recorded data to make sure that code
%   generation will be successful. 
% # Cleaning the radar detections from unnecessary clutter detections. The
%   radar reports many objects that correspond to fixed objects, which
%   include: guard-rails, the road median, traffic signs, etc. If these
%   detections are used in the tracking, they will create false tracks of
%   fixed objects at the edges of the road and therefore should be removed
%   prior to calling the tracker. Radar objects are considered non-clutter
%   if they are either stationary in front of the car or moving in its
%   vicinity.
% # Formatting the detections as input to the tracker, i.e., an array of
%   <matlab:doc('objectDetection'), objectDetection> elements. See the
%   processVideo and processRadar supporting functions below.
function [detections, egoLane] = processDetections...
        (visionFrame, radarFrame, IMUFrame, laneFrame, egoLane, time)
    % Inputs are:
    %   visionFrame - objects reported by the vision sensor for this time frame
    %   radarFrame  - objects reported by the radar sensor for this time frame
    %   IMUFrame    - inertial measurement unit data for this time frame
    %   laneFrame   - lane reports for this time frame
    %   egoLane     - the estimated ego lane
    %   time        - the time corresponding to the time frame

    % Remove clutter radar objects
    egoLane = processLanes(laneFrame, egoLane);
    realRadarObjects = findNonClutterRadarObjects(radarFrame.object,...
        radarFrame.numObjects, IMUFrame.velocity, egoLane);
    numRadarObjects = numel(realRadarObjects);
        
    % Memory allocation for code generation.
    % The recorded detections have several fields that the tracker does not
    % use itself, but may be of interest to the user. We store these fields
    % in a struct names ObjectAttributes. Since the recorded data uses a
    % variety of data types and sizes, we have to allocate the memory for
    % code generation using the exact same data types and sizes as the
    % recorded data as shown in objAttr. 
    % We define the set of all the fields we would like to maintain from
    % both sensor types. Each sensor will later fill out only the subset
    % that pertains to that sensor.
    objAttr = struct('VisionID', uint32(0), 'Size', zeros(1,3,'single'), ...
        'RadarID', uint16(0), 'Status', uint8(0), 'Amplitude', single(0), ...
        'RangeMode', uint8(0));
    
    % Use the object attributes in allocating an objectDetection. This
    % objectDetection captures the exact size and types required for
    % generating code.
    det = {objectDetection(time, zeros(4,1,'single'), 'SensorIndex', 1, ...
        'MeasurementNoise', eye(4,'single'), 'MeasurementParameters', {1}, ...
        'ObjectClassID', uint8(0), ...
        'ObjectAttributes', {objAttr})};
    
    % Finally, allocate the required number of objectDetection objects by
    % using the repmat command. It will create a cell array where each cell
    % is an objectDetection as defined by det. The allocated memory will
    % then be used by processRadar and processVideo that will enter the
    % recorded data into the allocated the memory.
    detections = repmat(det, [visionFrame.numObjects + numRadarObjects, 1]);
    
    if (visionFrame.numObjects + numRadarObjects) == 0        
        return;
    end

    %Process the remaining radar objects
    detections = processRadar(detections, realRadarObjects, time, 0);

    % Process video objects
    detections = processVideo(detections, visionFrame, time, numRadarObjects);
end

%% Processing the Lanes
% Maintans a persistent ego lane estimate
function egoLane = processLanes(laneReports, egoLane)
    % Lane boundaries are updated based on the laneReports from the recordings.
    % Since some laneReports contain invalid (isValid = false) reports or
    % impossible parameter values (-1e9), these lane reports are ignored and
    % the previous lane boundary is used.
    leftLane    = laneReports.left;
    rightLane   = laneReports.right;
    
    % Check the validity of the reported left lane
    cond = (leftLane.isValid && leftLane.confidence) && ...
        ~(leftLane.headingAngle == -1e9 || leftLane.curvature == -1e9);
    if cond
        egoLane.left = cast([leftLane.curvature, leftLane.headingAngle, leftLane.offset], 'double');
    end
    
    % Check the validity of the reported right lane
    cond = (rightLane.isValid && rightLane.confidence) && ...
        ~(rightLane.headingAngle == -1e9 || rightLane.curvature == -1e9);
    if cond
        egoLane.right  = cast([rightLane.curvature, rightLane.headingAngle, rightLane.offset], 'double');
    end
end

%% Removing Clutter from Radar Detections
% Removes radar objects that are considered part of the clutter
function realRadarObjects = findNonClutterRadarObjects(radarObject, numRadarObjects, egoSpeed, egoLane)
% The radar objects include many objects that belong to the clutter.
% Clutter is defined as a stationary object that is not in front of the
% car. The following types of objects pass as non-clutter:
%   
% # Any object in front of the car
% # Any moving object in the area of interest around the car, including
%   objects that move at a lateral speed around the car
    
    % Allocate memory
    normVs = zeros(numRadarObjects, 1);
    inLane = zeros(numRadarObjects, 1);
    inZone = zeros(numRadarObjects, 1);
    
    % Parameters
    LaneWidth = 3.6;            % What is considered in front of the car
    ZoneWidth = 1.7*LaneWidth;  % A wider area of interest
    minV = 1;                   % Any object that moves slower than that is stationary
    for j = 1:numRadarObjects
        [vx, vy] = calculateGroundSpeed(radarObject(j).velocity(1),radarObject(j).velocity(2),egoSpeed);
        normVs(j) = norm([vx,vy]);
        laneBoundariesAtObject = [polyval(egoLane.left, radarObject(j).position(1)), polyval(egoLane.right, radarObject(j).position(1))];
        laneCenter = mean(laneBoundariesAtObject);
        inLane(j) = (abs(radarObject(j).position(2) - laneCenter) <= LaneWidth/2);
        inZone(j) = (abs(radarObject(j).position(2) - laneCenter) <= max(abs(vy)*2, ZoneWidth));
    end         
    realRadarObjectsIdx = union(...
        intersect(find(normVs > minV), find(inZone == 1)), ...
        find(inLane == 1));
        
    realRadarObjects = radarObject(realRadarObjectsIdx);
end

%%%
% *calculateGroundSpeed*
% Calculates the true ground speed of a radar reported object from the
% relative speed and the ego car speed
function [Vx,Vy] = calculateGroundSpeed(Vxi,Vyi,egoSpeed)
% Inputs
%   (Vxi,Vyi) : relative object speed
%   egoSpeed  : ego vehicle speed
% Outputs
%   [Vx,Vy]   : ground object speed

    Vx = Vxi + egoSpeed; % calculate longitudinal ground speed
    theta = atan2(Vyi,Vxi); % calculate heading angle
    Vy = Vx * tan(theta); % calculate lateral ground speed

end

%% Process Vision Detections
% Converts reported vision objects to a list of |objectDetections|
function detections = processVideo(detections, visionFrame, t, startIndex)
% Process the video objects into objectDetections
numVisionObjects = visionFrame.numObjects;
if numVisionObjects
    classToUse = class(visionFrame.object(1).position);
    visionMeasCov = cast(diag([2,2,2,100]), classToUse);
    % Process Vision Objects:
    for i=1:numVisionObjects
        object = visionFrame.object(i);
        objAttr = struct('VisionID', uint32(object.id), 'Size', object.size, ...
            'RadarID', uint16(0), 'Status', uint8(0), 'Amplitude', single(0), ...
            'RangeMode', uint8(0));
        detections{startIndex+i} = objectDetection(t,...
            [object.position(1); object.velocity(1); object.position(2); 0], ...
            'SensorIndex', 1, 'MeasurementNoise', visionMeasCov, ...
            'MeasurementParameters', {1}, ...
            'ObjectClassID', object.classification, ...
            'ObjectAttributes', {objAttr});
    end
end
end

%% Process Radar Detections
% Converts reported radar objects to a list of |objectDetections|
function detections = processRadar(detections, realRadarObjects, t, startIndex)
% Process the radar objects into objectDetections
    numRadarObjects = numel(realRadarObjects);
    if numRadarObjects
        classToUse = class(realRadarObjects(1).position);
        radarMeasCov = cast(diag([2,2,2,100]), classToUse);
        % Process Radar Objects:
        for i=1:numRadarObjects
            object = realRadarObjects(i);
            objAttr = struct('VisionID', uint32(0), 'Size', zeros(1,3,'single'), ...
                'RadarID', object.id, 'Status', object.status, 'Amplitude', object.amplitude, ...
                'RangeMode', object.rangeMode);
            detections{startIndex+i} = objectDetection(t, ...
                [object.position(1); object.velocity(1); object.position(2); object.velocity(2)], ...
                'ObjectClassID', uint8(0), ...
                'SensorIndex', 2, 'MeasurementNoise', radarMeasCov, ...
                'MeasurementParameters', {2}, ...
                'ObjectAttributes', {objAttr});
        end
    end
end
%%
function measurement = fcwmeas(state, varargin)
% The demo measurements depend on the sensor type, which is reported by the
% objectDetection's MeasurementParameters property. The following two
% sensorID values are used:
%   sensorID=1: video objects, the measurement is [x;vx;y]. 
%   sensorID=2: radar objects, the measurement is [x;vx;y;vy]. 
% The state is:
%   Constant velocity       state = [x;vx;y;vy] 
%   Constant turn           state = [x;vx;y;vy;omega]
%   Constant acceleration   state = [x;vx;ax;y;vy;ay]

%#codegen
    measurement = zeros(4, 1, 'like', state);
    if isempty(varargin)
        sensorID = 1;
    else
        sensorID = varargin{1};
    end
    if numel(state) < 6 % Constant turn or constant velocity
        switch sensorID
            case 1 % video
                measurement = [state(1:3); 0];
            case 2 % radar
                measurement = state(1:4);
        end
    else % Constant acceleration
        switch sensorID
            case 1 % video
                measurement = [state(1:2); state(4); 0];
            case 2 % radar
                measurement = [state(1:2); state(4:5)];
        end
    end
end

%%
function jacobian = fcwmeasjac(state, varargin)
% The demo measurements depend on the sensor type, which is reported by the
% objectDetection's MeasurementParameters property. We choose sensorID=1
% for video objects and sensorID=2 for radar objects.  The following two
% sensorID values are used:
%   sensorID=1: video objects, the measurement is [x;vx;y]. 
%   sensorID=2: radar objects, the measurement is [x;vx;y;vy]. 
% The state is:
%   Constant velocity       state = [x;vx;y;vy] 
%   Constant turn           state = [x;vx;y;vy;omega]
%   Constant acceleration   state = [x;vx;ax;y;vy;ay]

%#codegen
    numStates = numel(state);
    jacobian = zeros(4, numStates, 'like', state);
    
    if isempty(varargin)
        sensorID = 1;
    else
        sensorID = varargin{1};
    end

    if numel(state) < 6 % Constant turn or constant velocity
        switch sensorID
            case 1 % video
                jacobian(1,1) = 1;
                jacobian(2,2) = 1;
                jacobian(3,3) = 1;
            case 2 % radar
                jacobian(1,1) = 1;
                jacobian(2,2) = 1;
                jacobian(3,3) = 1;
                jacobian(4,4) = 1;
        end
    else % Constant acceleration
        switch sensorID
            case 1 % video
                jacobian(1,1) = 1;
                jacobian(2,2) = 1;
                jacobian(3,4) = 1;
            case 2 % radar
                jacobian(1,1) = 1;
                jacobian(2,2) = 1;
                jacobian(3,4) = 1;
                jacobian(4,5) = 1;
        end
    end
end

%% findMostImportantObject
% Finding the Most Important Object and Issuing a Forward Collision Warning
% The most important object (MIO) is defined as the track that is in the
% ego lane and is closest in front of the car, i.e., with the smallest
% positive x value. To lower the probability of false alarms, only
% confirmed tracks are considered.
%
% Once the MIO is found, the relative speed between the car and MIO is
% calculated. The relative distance and relative speed determine the
% forward collision warning. There are 3 cases of FCW:
%
% # Safe: There is no car in the ego lane (no MIO), the MIO is moving away
%   from the car, or the distance is maintained constant. This case is
%   indicated by a green color.
% # Caution: The MIO is moving closer to the car, but is still at a 
%   distance above the FCW distance. FCW distance is calculated using the
%   Euro NCAP AEB Test Protocol. Note that this distance varies with the
%   relative speed between the MIO and the car, and is greater when the
%   closing speed is higher. This case is indicated by a yellow color.
% # Warn: The MIO is moving closer to the car, and its distance is less 
%   than the FCW distance. This case is indicated by a red color.
%
% Euro NCAP AEB Test Protocol defines the following distance calculation:
%%
% $d_{FCW} = 1.2 * v_{rel} + \frac{v_{rel}^2}{2a_{max}}$
%
% Where:
%
% $d_{FCW}$ is the forward collision warning distance
%
% $v_{rel}$ is the relative velocity between the two vehicles
%
% $a_{max}$ is the maximum deceleration defined to be 40% of the gravity acceleration
function mostImportantObject = findMostImportantObject(confirmedTracks,egoLane,positionSelector,velocitySelector)

% Initialize outputs and parameters
MIO = zeros(0, 'uint32');        % By default, there is no MIO
trackIndex = [];                 % By default, there is no MIO
FCW = 3;                         % By default, the FCW is 'safe'
threatColor = 'green';           % By default, the threat color is green
maxX = single(1000);             % Beyond sensor range. No track can exceed it
gAccel = 9.8;                    % Constant gravity acceleration, in m/s^2
maxDeceleration = 0.4 * gAccel;  % EuroNCAP AEB definition
delayTime = 1.2;                 % Delay time for a driver before starting to break, in seconds

for i = 1:numel(confirmedTracks)
    position = positionSelector * confirmedTracks(i).State;
    x = position(1);
    y = position(2);
    
    velocity = velocitySelector * confirmedTracks(i).State;
    relSpeed = velocity(1); % The relative speed between the cars, along the lane
    
    if x < maxX && x > 0 % No point checking otherwise
        yleftLane  = polyval(egoLane.left,  x);
        yrightLane = polyval(egoLane.right, x);
        if (yrightLane <= y) && (y <= yleftLane)
            maxX = x;
            trackIndex = i;
            MIO = confirmedTracks(i).TrackID;
            if relSpeed < 0 % Relative speed indicates object is getting closer
                % Calculate expected braking distance according to
                % Euro NCAP AEB Test Protocol
                d = abs(relSpeed) * delayTime + relSpeed^2 / 2 / maxDeceleration;
                if x <= d % 'warn'
                    FCW = 1;
                    threatColor = 'red';
                else % 'caution'
                    FCW = 2;
                    threatColor = 'yellow';
                end
            end
        end
    end
end
mostImportantObject = struct('ObjectID', MIO, 'TrackIndex', trackIndex, 'Warning', FCW, 'ThreatColor', threatColor);
end