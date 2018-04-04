%% Multiple Object Tracking Tutorial
% This example shows how to perform automatic detection and motion-based
% tracking of moving objects in a video. It simplifies the example
% <matlab:helpview(fullfile(docroot,'toolbox','vision','vision.map'),'MotionBasedMultiObjectTrackingExample')
% Motion-Based Multiple Object Tracking> and uses the |multiObjectTracker|
% available in Automated Driving System Toolbox.
%
%   Copyright 2016 The MathWorks, Inc.

%%
% Detection of moving objects and motion-based tracking are important 
% components of many computer vision applications, including activity
% recognition, traffic monitoring, and automotive safety.  The problem of
% motion-based object tracking can be divided into two parts:
%
% # Detecting moving objects in each frame 
% # Tracking the moving objects from frame to frame 
%
% The detection of moving objects uses a background subtraction algorithm
% based on Gaussian mixture models. Morphological operations are applied to
% the resulting foreground mask to eliminate noise. Finally, blob analysis
% detects groups of connected pixels, which are likely to correspond to
% moving objects. 
%
% The tracking of moving objects from frame to frame is done by the
% |multiObjectTracker| object that is responsible for the following:
%
% # Assigning detections to tracks. 
% # Initializing new tracks based on unassigned detections. All tracks are
%   initialized as |'Tentative'|, accounting for the possibility that they
%   resulted from a false detection.
% # Confirming tracks if they have more than _M_ assigned detections in _N_
%   frames.
% # Updating existing tracks based on assigned detections.
% # Coasting (predicting) existing unassigned tracks.
% # Deleting tracks if they have remained unassigned (coasted) for too long.
%
% The assignment of detections to the same object is based solely on
% motion. The motion of each track is estimated by a Kalman filter. The
% filter predicts the track's location in each frame, and determines the
% likelihood of each detection being assigned to each track. To initialize
% the filter that you design, use the |FilterInitializationFcn| property of
% the |multiObjectTracker|.
%
% For more information, see
% <matlab:helpview(fullfile(docroot,'toolbox','vision','vision.map'),'multipleObjectTracking') Multiple Object Tracking>.
%
% This example is a function, with the main body at the top and helper 
% routines in the form of 
% <matlab:helpview(fullfile(docroot,'toolbox','matlab','matlab_prog','matlab_prog.map'),'nested_functions') nested functions> 
% below.

function MultiObjectTrackerExample()
% Create objects used for reading video and displaying the results.
videoObjects = setupVideoObjects('atrium.mp4');

% Create objects used for detecting objects in the foreground of the video.
minBlobArea = 400; % Minimum blob size, in pixels, to be considered as a detection
detectorObjects = setupDetectorObjects(minBlobArea);

%% Create the Multi-Object Tracker
% When creating a |multiObjectTracker|, consider the following: 
%
% # |FilterInitializationFcn|: The likely motion and measurement models. 
%   In this case, the objects are expected to have a constant speed motion.
%   The |initDemoFilter| function configures a linear Kalman filter to 
%   track the motion. See the 'Define a Kalman filter' section for details.
% # |AssignmentThreshold|: How far detections may fall from tracks. 
%   The default value for this parameter is 30. If there are detections
%   that are not assigned to tracks, but should be, increase this value. If
%   there are detections that get assigned to tracks that are too far,
%   decrease this value.
% # |NumCoastingUpdates|: How long a track is maintained before deletion.
%   In this case, since the video has 30 frames per second, a reasonable
%   value is about 0.75 seconds (22 frames).
% # |ConfirmationParameters|: The parameters controlling track confirmation.
%   A track is initialized with every unassigned detection. Some of these
%   detections might be false, so initially, all tracks are |'Tentative'|. 
%   To confirm a track, it has to be detected at least _M_ out of _N_
%   frames. The choice of _M_ and _N_ depends on the visibility of the
%   objects. This example assumes a visibility of 6 out of 10 frames.
tracker = multiObjectTracker(...
    'FilterInitializationFcn', @initDemoFilter, ...
    'AssignmentThreshold', 30, ...
    'NumCoastingUpdates', 22, ...
    'ConfirmationParameters', [6 10] ...
    );

%% Define a Kalman Filter
% When defining a tracking filter for the motion, complete the following
% steps:
%
% *Step 1: Define the motion model and state*
%
% In this example, use a constant velocity model in a 2-D rectangular
% frame.
%
% # The state is |[x;vx;y;vy]|.
% # The state transition model matrix is |A = [1 dt 0 0; 0 1 0 0; 0 0 1 dt; 0 0 0 1]|.
% # Assume that |dt = 1|.
%
% *Step 2: Define the process noise*
%
% The process noise represents the parts of the process that are not taken
% into account in the model. For example, in a constant velocity model, the
% acceleration is neglected.
%
% *Step 3: Define the measurement model*
%
% In this example, only the position (|[x;y]|) is measured. So, the
% measurement model is |H = [1 0 0 0; 0 0 1 0]|.
%
% Note: To preconfigure these parameters, define the |'MotionModel'|
% property as |'2D Constant Velocity'|.
%
% *Step 4: Initialize the state vector based on the sensor measurement*
%
% In this example, because the measurement is |[x;y]| and the state is
% |[x;vx;y;vy]|, initializing the state vector is straightforward. Because
% there is no measurement of the velocity, initialize the |vx| and |vy|
% components to 0.
%
% *Step 5: Define an initial state covariance*
%
% In this example, the measurements are quite noisy, so define the initial 
% state covariance to be quite large: |stateCov = diag([50, 50, 50, 50])|
%
% *Step 6: Create the correct filter*
% 
% In this example, all the models are linear, so use |trackingKF| as the
% tracking filter.
    function filter = initDemoFilter(detection)
    % Initialize a Kalman filter for this example.
    
    % Define the initial state.
    state = [detection.Measurement(1); 0; detection.Measurement(2); 0];

    % Define the initial state covariance.
    stateCov = diag([50, 50, 50, 50]);

    % Create the tracking filter.
    filter = trackingKF('MotionModel', '2D Constant Velocity', ...    
        'State', state, ...
        'StateCovariance', stateCov, ... 
        'MeasurementNoise', detection.MeasurementNoise(1:2,1:2) ...    
        );
    end

%%% 
% The following loop runs the video clip, detects moving objects in the
% video, and tracks them across video frames. 

% Count frames to create a sense of time.
frameCount = 0;
while hasFrame(videoObjects.reader)
    % Read a video frame and detect objects in it.
    frameCount = frameCount + 1;                                % Promote frame count
    frame = readFrame(videoObjects.reader);                     % Read frame    
    [detections, mask] = detectObjects(detectorObjects, frame); % Detect objects in video frame        
        
    % Run the tracker on the preprocessed detections.
    confirmedTracks = updateTracks(tracker, detections, frameCount);
    
    % Display the tracking results on the video.
    displayTrackingResults(videoObjects, confirmedTracks, frame, mask);
end
%% Create Video Objects
% Create objects used for reading and displaying the video frames.

    function videoObjects = setupVideoObjects(filename)
        % Initialize video I/O
        % Create objects for reading a video from a file, drawing the tracked
        % objects in each frame, and playing the video.
        
        % Create a video file reader.
        videoObjects.reader = VideoReader(filename);
        
        % Create two video players: one to display the video,
        % and one to display the foreground mask.        
        videoObjects.maskPlayer  = vision.VideoPlayer('Position', [20, 400, 700, 400]);
        videoObjects.videoPlayer = vision.VideoPlayer('Position', [740, 400, 700, 400]);
    end

%% Create Detector Objects
% Create objects used for detecting foreground objects.
% Use |minBlobArea| to define the size of the blob, in pixels, that is
% considered to be a detection. 
%
% * Increase |minBlobArea| to avoid detecting small blobs, which are more
%   likely to be false detections, or if several detections are created for 
%   the same object due to partial occlusion.
% * Decrease |minBlobArea| if objects are detected too late or not at all.

    function detectorObjects = setupDetectorObjects(minBlobArea)
        % Create System objects for foreground detection and blob analysis
        
        % The foreground detector segments moving objects from the
        % background. It outputs a binary mask, where the pixel value of 1
        % corresponds to the foreground and the value of 0 corresponds to
        % the background.
        
        detectorObjects.detector = vision.ForegroundDetector('NumGaussians', 3, ...
            'NumTrainingFrames', 40, 'MinimumBackgroundRatio', 0.7);
        
        % Connected groups of foreground pixels are likely to correspond to
        % moving objects.  The blob analysis System object finds such
        % groups (called 'blobs' or 'connected components') and computes
        % their characteristics, such as their areas, centroids, and the
        % bounding boxes.
        
        detectorObjects.blobAnalyzer = vision.BlobAnalysis('BoundingBoxOutputPort', true, ...
            'AreaOutputPort', true, 'CentroidOutputPort', true, ...
            'MinimumBlobArea', minBlobArea);
    end

%% Detect Objects
% The |detectObjects| function returns the centroids and the bounding boxes
% of the detected objects as a list of |objectDetection| objects. You can
% supply this list as an input to the |multiObjectTracker|. The
% |detectObjects| function also returns the binary mask, which has the same
% size as the input frame. Pixels with a value of 1 correspond to the
% foreground. Pixels with a value of 0 correspond to the background.
%
% The function performs motion segmentation using the foreground detector. 
% It then performs morphological operations on the resulting binary mask to
% remove noisy pixels and to fill the holes in the remaining blobs.
%
% When creating the |objectDetection| list, the |frameCount| serves as the
% time input, and the centroids of the detected blobs serve as the
% measurement. The list also has two optional name-value pairs:
%
% * |MeasurementNoise| - Blob detection is noisy, and this example defines 
%   a large measurement noise value.
% * |ObjectAttributes| - The detected bounding boxes that get passed to the
%   track display are added to this argument.

    function [detections, mask] = detectObjects(detectorObjects, frame)
        % Expected uncertainty (noise) for the blob centroid.
        measurementNoise = 100*eye(2);  
        % Detect foreground.
        mask = detectorObjects.detector.step(frame);
        
        % Apply morphological operations to remove noise and fill in holes.
        mask = imopen(mask, strel('rectangle', [6, 6]));
        mask = imclose(mask, strel('rectangle', [50, 50])); 
        mask = imfill(mask, 'holes');
        
        % Perform blob analysis to find connected components.
        [~, centroids, bboxes] = detectorObjects.blobAnalyzer.step(mask);
        
        % Formulate the detections as a list of objectDetection objects.
        numDetections = size(centroids, 1);
        detections = cell(numDetections, 1);
        for i = 1:numDetections
            detections{i} = objectDetection(frameCount, centroids(i,:), ...
                'MeasurementNoise', measurementNoise, ...
                'ObjectAttributes', {bboxes(i,:)});
        end
    end

%% Display Tracking Results
% The |displayTrackingResults| function draws a bounding box and label ID
% for each track on the video frame and foreground mask. It then displays
% the frame and the mask in their respective video players.

    function displayTrackingResults(videoObjects, confirmedTracks, frame, mask)
        % Convert the frame and the mask to uint8 RGB.
        frame = im2uint8(frame);
        mask = uint8(repmat(mask, [1, 1, 3])) .* 255;
        
        if ~isempty(confirmedTracks)            
            % Display the objects. If an object has not been detected
            % in this frame, display its predicted bounding box.
            numRelTr = numel(confirmedTracks);
            boxes = zeros(numRelTr, 4);
            ids = zeros(numRelTr, 1, 'int32');
            predictedTrackInds = zeros(numRelTr, 1);
            for tr = 1:numRelTr
                % Get bounding boxes.
                boxes(tr, :) = confirmedTracks(tr).ObjectAttributes{1}{1};
                
                % Get IDs.
                ids(tr) = confirmedTracks(tr).TrackID;
                
                if confirmedTracks(tr).IsCoasted
                    predictedTrackInds(tr) = tr;
                end
            end
            
            predictedTrackInds = predictedTrackInds(predictedTrackInds > 0);
            
            % Create labels for objects that display the predicted rather 
            % than the actual location.
            labels = cellstr(int2str(ids));
            
            isPredicted = cell(size(labels));
            isPredicted(predictedTrackInds) = {' predicted'};
            labels = strcat(labels, isPredicted);
            
            % Draw the objects on the frame.
            frame = insertObjectAnnotation(frame, 'rectangle', boxes, labels);
            
            % Draw the objects on the mask.
            mask = insertObjectAnnotation(mask, 'rectangle', boxes, labels);
        end
        
        % Display the mask and the frame.
        videoObjects.maskPlayer.step(mask);        
        videoObjects.videoPlayer.step(frame);
    end
displayEndOfDemoMessage(mfilename)
end
%% Summary
% In this example, you created a motion-based system for detecting and
% tracking multiple moving objects. Try using a different video to see if
% you can detect and track objects. Try modifying the parameters of the
% |multiObjectTracker|.
%
% The tracking in this example was based solely on motion, with the
% assumption that all objects move in a straight line with constant speed.
% When the motion of an object significantly deviates from this model, the
% example can produce tracking errors. Notice the mistake in tracking the
% person occluded by the tree.
%
% You can reduce the likelihood of tracking errors by using a more complex
% motion model, such as constant acceleration or constant turn. To do that,
% try defining a different tracking filter, such as |trackingEKF| or
% |trackingUKF|. 
