%% Track Pedestrians from a Moving Car
%
% This example shows how to track pedestrians using a camera mounted in a
% moving car.
%
% Copyright 2016 The MathWorks, Inc.

%% Overview
% This example shows how to perform automatic detection and tracking of
% people in a video from a moving camera. It demonstrates the flexibility
% of a tracking system adapted to a moving camera, which is ideal for
% automotive safety applications. Unlike the vehicle tracking example,
% <VehicleTrackingExample.html Track Multiple Vehicles Using a Camera>,
% this example contains additional algorithmic steps. These steps include
% people detection and customized cost for assigning detections to tracks.
%
% The tracking workflow consists of the following steps:
%
% # Define camera intrinsics and camera mounting position.
% # Load and configure a pretrained people detector.
% # Set up a multi-object tracker.
% # Run the detector for each video frame.
% # Compute costs for detections-to-tracks assignments.
% # Update the tracker with detection results.
% # Display the tracking results in a video.

%% Configure People Detector and Multi-object Tracker
% In this example, we use a pretrained ACF people detector, and configure
% this detector to incorporate camera information. By default, the detector
% scans the entire image at multiple scales. By knowing the camera
% parameters, we can configure the detector to only detect people on the
% ground plane at reasonable scales.

% Load the monoCamera object that contains the camera information.
d = load('TrackingDemoMonoCameraSensor.mat', 'sensor');

% Load a pretrained ACF people detector. The ACF detector uses "Aggregate
% Channel Features", which is fast to compute in practice. The 'caltech'
% model is trained on caltech pedestrian dataset, which can detect people
% at the minimum resolution of 50x21 pixels.
detector = peopleDetectorACF('caltech');

% Configure the detector using the sensor information. The detector only
% tries to find people at image regions above the ground plane. This can
% reduce computation and prevent spurious detections.

% The width of common pedestrian is between 0.5 to 1.5 meters. Only a
% bounding box of width within this range is considered as a detection
% candidate in image.
pedWidth = [0.5, 1.5];

% Configure the detector using the monoCamera sensor and desired width.
detector = configureDetectorMonoCamera(detector, d.sensor, pedWidth);

% Initialize an multi-object tracker including setting the filter,
% the detection-to-track assignment threshold, the coasting and
% confirmation parameters. You can find the |setupTracker| function at the
% end of this example.
[tracker, positionSelector] = setupTracker();

%% Track People in a Video
%
% At each time step, run the detector, compute detection-to-track
% assignment costs, update the tracker with detection results, and display
% the tracking results in a video.

% Setup Video Reader and Player
videoFile = 'pedtracking.mp4';
videoReader = VideoReader(videoFile);
videoPlayer = vision.DeployableVideoPlayer();

currentStep = 0;
snapshot = [];
snapTimeStamp = 107;
cont = hasFrame(videoReader);
while cont
    % Update frame counters.
    currentStep = currentStep + 1;

    % Read the next frame.
    frame = readFrame(videoReader);
    
    % Run the detector and package the returned results into an object
    % required by multiObjectTracker.  You can find the |detectObjects|
    % function at the end of this example.
    detections = detectObjects(detector, frame, currentStep);
    
    % Using the list of objectDetections, return the tracks, updated for
    % 'currentStep' time. After the first frame, a helper function
    % |detectionToTrackCost| is called to compute the customized cost
    % matrix between all tracks and all detections.
    if currentStep == 1
        costMatrix = zeros(0, numel(detections)); 
        [confirmedTracks,~,allTracks] = updateTracks(tracker, detections, currentStep, ...
            costMatrix);
    else
        costMatrix = detectionToTrackCost(allTracks, detections, ...
            positionSelector, tracker.AssignmentThreshold);
        [confirmedTracks,~,allTracks] = updateTracks(tracker, detections, currentStep, ...
            costMatrix);
    end
    
    % Remove the tracks for people that are far away.
    confirmedTracks = removeNoisyTracks(confirmedTracks, positionSelector, d.sensor.Intrinsics.ImageSize);
    
    % Insert tracking annotations.
    frameWithAnnotations = insertTrackBoxes(frame, confirmedTracks, positionSelector, d.sensor);

    % Display the annotated frame.    
    videoPlayer(frameWithAnnotations);  
    
    % Take snapshot for publishing at snapTimeStamp seconds
    if currentStep == snapTimeStamp
        snapshot = frameWithAnnotations;
    end   
    
    % Exit the loop if the video player figure is closed by user.
    cont = hasFrame(videoReader) && isOpen(videoPlayer);
end

%%
% Show the tracked people and display the distance to the ego car. 
if ~isempty(snapshot)
    figure
    imshow(snapshot)
end

%%
% The tracking workflow presented here can be easily integrated into the
% <MonoCameraExample.html Visual Perception Using Monocular Camera>, where
% the people or vehicle detection step can be enhanced with the tracker. To
% learn about additional tracking capabilities in the Automated Driving
% System Toolbox(R), please take a look at the <matlab:doc('monoCamera')
% monoCamera> and <matlab:doc('multiObjectTracker') multiObjectTracker>.

displayEndOfDemoMessage(mfilename)

%% Supporting Functions

%% 
% *setupTracker* function creates a |multiObjectTracker| to track multiple
% objects with Kalman filters. When creating a |multiObjectTracker|
% consider the following:
%
% * |FilterInitializationFcn|: The likely motion and measurement models.
%   In this case, the objects are expected to have a constant velocity
%   motion. See the 'Define a Kalman filter' section.
% * |AssignmentThreshold|: how far detections may fall from tracks. 
%   If there are detections that are not assigned to tracks, but should be,
%   increase this value. If there are detections that get assigned to
%   tracks that are too far, decrease this value. This example uses
%   bounding box overlap ratio as the assignment cost, with the threshold
%   set to 0.999.
% * |NumCoastingUpdates|: How many times a track is coasted before deletion.
%   Coasting is a term used for updating the track without an assigned
%   detection (predicting).
%   The default value for this parameter is 5. 
% * |ConfirmationParameters|: The parameters for confirming a track.
%   A new track is initialized with every unassigned detection. Some of
%   these detections may be false, so all the tracks are initialized as
%   |Tentative|. To confirm a track, it has to be detected at least _M_
%   times in _N_ tracker updates. The choice of _M_ and _N_ depends on the
%   visibility of the objects. This example uses the default of 3
%   detections out of 5 updates.
% * |HasCostMatrixInput|: True if cost matrix is provided as input.
%   Set this property to true to pass a cost matrix as part of the inputs
%   to |updateTracks|.
%
% The outputs of |setupTracker| are:
%
% * |tracker| - The multiObjectTracker that is configured for this case.
% * |positionSelector| - A matrix that specifies which elements of the
%   State vector are the position: |position = positionSelector * State|
function [tracker, positionSelector] = setupTracker()
    % Create the tracker object.
    tracker = multiObjectTracker('FilterInitializationFcn', @initBboxFilter, ...
        'AssignmentThreshold', 0.999, ...
        'NumCoastingUpdates', 5, ... 
        'ConfirmationParameters', [3 5], ...
        'HasCostMatrixInput', true);

    % The State vector is: [x; vx; y; vy; w; vw; h; vh]
    % [x;y;w;h] = positionSelector * State
    positionSelector = [1 0 0 0 0 0 0 0; ...
                        0 0 1 0 0 0 0 0; ...
                        0 0 0 0 1 0 0 0; ...
                        0 0 0 0 0 0 1 0]; 
end

%% 
% *initBboxFilter* function defines a Kalman filter to filter bounding box
% measurement.
function filter = initBboxFilter(Detection)
% Step 1: Define the motion model and state.
%   Use a constant velocity model for a bounding box on the image.
%   The state is [x; vx; y; vy; w; wv; h; hv]
%   The state transition matrix is: 
%       [1 dt 0  0 0  0 0  0;
%        0  1 0  0 0  0 0  0; 
%        0  0 1 dt 0  0 0  0; 
%        0  0 0  1 0  0 0  0; 
%        0  0 0  0 1 dt 0  0; 
%        0  0 0  0 0  1 0  0;
%        0  0 0  0 0  0 1 dt; 
%        0  0 0  0 0  0 0  1]
%   Assume dt = 1. This example does not consider time-variant transition
%   model for linear Kalman filter.
    dt = 1;
    cvel =[1 dt; 0 1];
    A = blkdiag(cvel, cvel, cvel, cvel);
 
% Step 2: Define the process noise. 
%   The process noise represents the parts of the process that the model
%   does not take into account. For example, in a constant velocity model,
%   the acceleration is neglected.
    G1d = [dt^2/2; dt];
    Q1d = G1d*G1d';
    Q = blkdiag(Q1d, Q1d, Q1d, Q1d);
 
% Step 3: Define the measurement model.
%   Only the position ([x;y;w;h]) is measured.
%   The measurement model is
    H = [1 0 0 0 0 0 0 0; ...
         0 0 1 0 0 0 0 0; ...
         0 0 0 0 1 0 0 0; ...
         0 0 0 0 0 0 1 0];
 
% Step 4: Map the sensor measurements to an initial state vector.
%   Because there is no measurement of the velocity, the v components are
%   initialized to 0:
    state = [Detection.Measurement(1); ...
             0; ...
             Detection.Measurement(2); ...
             0; ...
             Detection.Measurement(3); ...
             0; ...
             Detection.Measurement(4); ...
             0];
 
% Step 5: Map the sensor measurement noise to a state covariance.
%   For the parts of the state that the sensor measured directly, use the
%   corresponding measurement noise components. For the parts that the
%   sensor does not measure, assume a large initial state covariance. That way,
%   future detections can be assigned to the track.
    L = 100; % Large value
    stateCov = diag([Detection.MeasurementNoise(1,1), ...
                     L, ...
                     Detection.MeasurementNoise(2,2), ...
                     L, ...
                     Detection.MeasurementNoise(3,3), ...
                     L, ...
                     Detection.MeasurementNoise(4,4), ...
                     L]);
 
% Step 6: Create the correct filter.
%   In this example, all the models are linear, so use trackingKF as the
%   tracking filter.
    filter = trackingKF(...
        'StateTransitionModel', A, ...
        'MeasurementModel', H, ...
        'State', state, ...
        'StateCovariance', stateCov, ... 
        'MeasurementNoise', Detection.MeasurementNoise, ...
        'ProcessNoise', Q);
end

%% 
% *detectObjects* function detects people in an image.
function detections = detectObjects(detector, frame, frameCount)
    % Run the detector and return a list of bounding boxes: [x, y, w, h]
    bboxes = detect(detector, frame);
    
    % Define the measurement noise.
    L = 100;
    measurementNoise = [L 0  0  0; ...
                        0 L  0  0; ...
                        0 0 L/2 0; ...
                        0 0  0 L/2];
                    
    % Formulate the detections as a list of objectDetection reports.
    numDetections = size(bboxes, 1);
    detections = cell(numDetections, 1);                      
    for i = 1:numDetections
        detections{i} = objectDetection(frameCount, bboxes(i, :), ...
            'MeasurementNoise', measurementNoise);
    end
end

%% 
% *removeNoisyTracks* function removes noisy tracks. A track is considered
% to be noisy if its predicted bounding box is too small. Typically, this
% implies the person is far away.
function tracks = removeNoisyTracks(tracks, positionSelector, imageSize)

    if isempty(tracks)
        return
    end
    
    % Extract the positions from all the tracks.
    positions = getTrackPositions(tracks, positionSelector);
    % The track is 'invalid' if the predicted position is about to move out
    % of the image, or the bounding box is too small.
    invalid = ( positions(:, 1) < 1 | ...
                positions(:, 1) + positions(:, 3) > imageSize(2) | ...
                positions(:, 3) <= 5 | ...
                positions(:, 4) <= 10 );
    tracks(invalid) = [];
end

%% 
% *insertTrackBoxes* inserts bounding boxes in an image and displays the
% track's position in front of the car, in world units.
function I = insertTrackBoxes(I, tracks, positionSelector, sensor)

    if isempty(tracks)
        return
    end

    % Allocate memory.
    labels = cell(numel(tracks), 1);
    % Retrieve positions of bounding boxes.
    bboxes = getTrackPositions(tracks, positionSelector);

    for i = 1:numel(tracks)
        box = bboxes(i, :);
        
        % Convert to vehicle coordinates using monoCamera object
        xyVehicle = imageToVehicle(sensor, [box(1)+box(3)/2, box(2)+box(4)]);
        
        labels{i} = sprintf('x=%.1f,y=%.1f',xyVehicle(1),xyVehicle(2));
    end
    
    I = insertObjectAnnotation(I, 'rectangle', bboxes, labels, 'Color', 'yellow', ...
        'FontSize', 10, 'TextBoxOpacity', .8, 'LineWidth', 2);
end

%%
% *detectionToTrackCost* computes the customized cost for
% detections-to-tracks assignment.
%
% Assigning object detections in the current frame to existing tracks is
% done by minimizing cost. The cost is computed using the
% |bboxOverlapRatio| function, and this cost is the overlap ratio between
% the predicted and the detected bounding box. In this example, the person
% is assumed to move gradually in consecutive frames, due to the high frame
% rate of the video and the low motion speed of the person.
%
% First, this example computes the cost of assigning every detection to
% each track by using the |bboxOverlapRatio| measure. As people move toward
% or away from the camera, the centroid point alone cannot accurately
% describe their motion. The cost takes into account the distance on the
% image plane, and the scale of the bounding boxes. This prevents assigning
% detections that are far away from the camera to tracks that are closer to
% the camera, even if the predicted and detected centroids coincide. The
% choice of this cost function eases the computation without resorting to a
% more sophisticated dynamic model. The results are stored in an _M_ by _N_
% matrix, where _M_ is the number of tracks, and _N_ is the number of
% detections.
%
% The value for the cost of not assigning a detection to a track depends on
% the range of values returned by the cost function. This value must be
% tuned experimentally. Setting it too low increases the likelihood of
% creating a new track, and can result in track fragmentation. Setting it
% too high can result in a single track corresponding to a series of
% separate moving objects. Possible assignment entries can be excluded from
% optimization whose value is above the nonassignment cost, and set to
% |Inf|.

function costMatrix = detectionToTrackCost(tracks, detections, ...
                                positionSelector, threshold)

    if isempty(tracks) || isempty(detections)
        costMatrix = zeros(length(tracks), length(detections));
        return
    end
    
    % Compute the overlap ratio between the predicted boxes and the
    % detected boxes, and compute the cost of assigning each detection
    % to each track. The cost is minimum when the predicted bbox is
    % perfectly aligned with the detected bbox (overlap ratio is one).
    
    % Retrieve positions of bounding boxes.
    trackBboxes = getTrackPositions(tracks, positionSelector);
    % Check that the width and height are positive before computing the box
    % overlap ratio.
    trackBboxes(:, 3) = max(eps, trackBboxes(:, 3));
    trackBboxes(:, 4) = max(eps, trackBboxes(:, 4));
    
    % Extract the detected bounding box from all the detections.
    allDetections = [detections{:}];
    bboxes = reshape([allDetections(:).Measurement], 4, length(detections))';
    
    % Compute all pairwise assignment costs.
    costMatrix = 1 - bboxOverlapRatio(trackBboxes, bboxes);
    % Set unrealistic assignment cost to Inf if there is little box
    % overlap.
    costMatrix(costMatrix(:) > threshold) = Inf;
end
