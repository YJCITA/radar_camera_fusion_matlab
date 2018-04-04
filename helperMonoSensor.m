%helperMonoSensor implements a simulation of a monocular camera sensor.
%  This is an example helper function and is subject to change in future
%  releases.
%
%  monoSensor = helperMonoSensor(sensor) returns a helperMonoSensor object
%  that implements a complete simulation of a monocular camera sensor.
%
%   helperMonoSensor properties:
%      Sensor - Sensor configuration specified as monoCamera object
%
%   helperMonoSensor methods:
%      processFrame         - Analyze frame of video sequence.
%      displaySensorOutputs - Display results returned by monocular sensor
%
%   Example
%   -------
%   focalLength    = [309.4362, 344.2161];
%   principalPoint = [318.9034, 257.5352];
%   imageSize      = [480, 640];
%   height         = 2.1798;    % mounting height in meters from the ground
%   pitch          = 14;        % pitch of the camera in degrees
%
%   camIntrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);
%   sensor        = monoCamera(camIntrinsics, height, 'Pitch', pitch);
%
%   videoReader = VideoReader('caltech_washington1.avi');
%
%   monoSensor   = helperMonoSensor(sensor);
%   isPlayerOpen = true;
%   while hasFrame(videoReader) && isPlayerOpen
%
%       frame = readFrame(videoReader); % get a frame   
%       sensorOut = processFrame(monoSensor, frame);
%
%       closePlayers = ~hasFrame(videoReader);
%       isPlayerOpen = displaySensorOutputs(monoSensor, frame, sensorOut, closePlayers);
%   end
%
%   See also MonoCameraExample

classdef helperMonoSensor < handle
    
    properties                
        % Sensitivity for the lane segmentation routine
        LaneSegmentationSensitivity = 0.25;
        
        % The percentage extent of the ROI a lane needs to cover
        LaneXExtentThreshold  = 0.4;
                
        % The percentage of inlier points required per unit length
        LaneStrengthThreshold = 0.24;

        % Detection threshold for the vehicle detector
        VehicleDetectionThreshold = -1;
    end
    
    properties (SetAccess='private', GetAccess='public')
        Sensor;
        MaxLaneStrength;
        BirdsEyeConfig;        % Bird's eye view associated with the Sensor
    end
    
    properties (SetAccess='private', GetAccess='private', Hidden)
        MonoDetector;       % ACF based vehicle detector        
        OutView;            % [xmin, xmax, ymin, ymax] region of bird's eye view
        
        % Other internal quantities used in visualization routine
        XVehiclePoints;
        BirdsEyeImage;
        VehicleBoxes;
        VehicleScores; % Currently unused
        VehicleCoordSysROI;
        BirdsEyeBW;              
    end
    
    methods
        %------------------------------------------------------------------
        % Constructor
        %------------------------------------------------------------------
        function this = helperMonoSensor(sensor)
            
            % Validate inputs
            this.Sensor = sensor;
            
            % Define what area you want to see in vehicle coordinates
            % [x, y, width, height]
            distAheadOfSensor = 30; % in meters
            spaceToOneSide    = 6;
            bottomOffset      = 3;
            outView           = [bottomOffset, distAheadOfSensor, -spaceToOneSide, spaceToOneSide];
            
            imageSize = [NaN, 250]; % output image width in pixels
            this.OutView  = outView;
            this.BirdsEyeConfig = birdsEyeView(sensor, outView, imageSize);
            
            detector = vehicleDetectorACF();
            
            % The width of common vehicles is between 1.5 to 2.5 meters.
            vehicleWidth = [1.5, 2.5];
            
            % Specialize the detector
            this.MonoDetector = configureDetectorMonoCamera(detector, sensor, vehicleWidth);
                        
            this.VehicleCoordSysROI = this.OutView - [-1, 2, -3, 3]; % look 3 meters to left/right and 4 meters ahead of the sensor

            % Compute the maximum lane strength by assuming all image pixels
            % are lane candidate points
            birdsImageROI           = vehicleToImageROI(this.BirdsEyeConfig, this.VehicleCoordSysROI);
            [laneImageX,laneImageY] = meshgrid(birdsImageROI(1):birdsImageROI(2),birdsImageROI(3):birdsImageROI(4));
            vehiclePoints           = imageToVehicle(this.BirdsEyeConfig,[laneImageX(:),laneImageY(:)]);
            maxPointsInOneLane      = numel(unique(vehiclePoints(:,1)));
            maxLaneLength           = diff(this.VehicleCoordSysROI(1:2));
            this.MaxLaneStrength    = maxPointsInOneLane/maxLaneLength;
        end
    end
    
    methods (Access='public')
        
        %------------------------------------------------------------------
        %processFrame Analyze frame of video sequence.
        %  [sensorOut, intOut] = processFrame(frame) returns results of
        %  analyzing a video frame. The main results are returned in
        %  sensorOut, a struct containing left and right ego-lane
        %  boundaries as well as vehicle detections in vehicle coordinates.
        function sensorOut = processFrame(this, frame)            
            
            birdsEyeConfig  = this.BirdsEyeConfig;
            sensor          = this.Sensor;
            monoDetector    = this.MonoDetector;
            outView         = this.OutView;
            
            % Internal parameters
            approxLaneMarkerWidthVehicle = 0.25; % 25 centimeters
            
            % Compute birdsEyeView image
            birdsEyeViewImage = transformImage(birdsEyeConfig, frame);
            birdsEyeViewImage = rgb2gray(birdsEyeViewImage);
            
            % Detect lane features
            birdsEyeViewBW = segmentLaneMarkerRidge(birdsEyeViewImage, birdsEyeConfig, ...
                approxLaneMarkerWidthVehicle, 'ROI', this.VehicleCoordSysROI, ...
                'Sensitivity', this.LaneSegmentationSensitivity);
            
            % Obtain lane candidate points in vehicle coordinates
            [imageX, imageY] = find(birdsEyeViewBW);
            boundaryPointsxy = imageToVehicle(birdsEyeConfig, [imageY, imageX]);
                        
            maxLanes = 2;
            % Expand boundary width to search for double markers            
            boundaryWidth =  3*approxLaneMarkerWidthVehicle;
            % Find lane boundaries
            [boundaries, boundaryPoints] = findParabolicLaneBoundaries(boundaryPointsxy,boundaryWidth, ...
                'MaxNumBoundaries', maxLanes, 'ValidateBoundaryFcn', @ValidateBoundaryFcn);
            
            % Trim boundaries based on length
            minXLength = diff(this.VehicleCoordSysROI(1:2)) * this.LaneXExtentThreshold;
            isOfMinLength = arrayfun(@(b)diff(b.XExtent) > minXLength, boundaries);
            boundaries = boundaries(isOfMinLength);
            % and strength
            isStrong   = [boundaries.Strength] >= this.LaneStrengthThreshold*this.MaxLaneStrength;
            boundaries = boundaries(isStrong);
            
            % Classify lane marker type as single/double, solid/dashed
            boundaries = classifyLaneTypes(boundaries, boundaryPoints);
            
            xOffset    = 0;   %  0 meters from the sensor
            distanceToBoundaries  = boundaries.computeBoundaryModel(xOffset);
            % Find candidate ego boundaries
            leftEgoBoundaryIndex  = [];
            rightEgoBoundaryIndex = [];            
            minLDistance = min(distanceToBoundaries(distanceToBoundaries>0));
            minRDistance = max(distanceToBoundaries(distanceToBoundaries<=0));
            if ~isempty(minLDistance)
                leftEgoBoundaryIndex  = distanceToBoundaries == minLDistance;
            end
            if ~isempty(minRDistance)
                rightEgoBoundaryIndex = distanceToBoundaries == minRDistance;
            end
            leftEgoBoundary       = boundaries(leftEgoBoundaryIndex);
            rightEgoBoundary      = boundaries(rightEgoBoundaryIndex);

            % Detect vehicles
            [bboxes, scores] = detect(monoDetector, frame);
            % Remove detections with low classification scores
            if ~isempty(scores)
                ind = scores >= this.VehicleDetectionThreshold;
                bboxes = bboxes(ind, :);
                scores = scores(ind);
            end
            % Compute distance in vehicle coordinates
            locations = computeVehicleLocations(bboxes, sensor);
            
            % Visualize sensor outputs and intermediate results
            % Create visualizations
            
            % Pack the core sensor outputs
            sensorOut.leftEgoBoundary  = leftEgoBoundary;
            sensorOut.rightEgoBoundary = rightEgoBoundary;
            sensorOut.vehicleLocations = locations;
            sensorOut.xVehiclePoints   = outView(1):outView(2); % minX to maxX
            sensorOut.vehicleBoxes     = bboxes;
            
            % Assign additional visualization data to internal properties
            this.BirdsEyeImage   = birdsEyeViewImage;
            this.XVehiclePoints  = sensorOut.xVehiclePoints;
            this.VehicleBoxes    = bboxes;
            this.VehicleScores   = scores;
            this.BirdsEyeBW      = birdsEyeViewBW;
        end
        
        %------------------------------------------------------------------
        % displaySensorOutputs method displays core information and
        % intermediate results from the monocular camera sensor simulation.
        function isPlayerOpen = ...
                displaySensorOutputs(this, frame, sensorOut, closePlayers)
            
            sensor = this.Sensor;
            
            % Unpack the main inputs
            leftEgoBoundary  = sensorOut.leftEgoBoundary;
            rightEgoBoundary = sensorOut.rightEgoBoundary;
            locations        = sensorOut.vehicleLocations;
            
            % Unpack additional intermediate data
            xVehiclePoints    = this.XVehiclePoints;
            birdsEyeViewImage = this.BirdsEyeImage;
            birdsEyeConfig    = this.BirdsEyeConfig;
            bboxes            = this.VehicleBoxes;
            birdsEyeViewBW    = this.BirdsEyeBW;
            
            birdsEyeWithOverlays = insertLaneBoundary(birdsEyeViewImage, leftEgoBoundary , birdsEyeConfig, xVehiclePoints, 'Color','Red');
            birdsEyeWithOverlays = insertLaneBoundary(birdsEyeWithOverlays, rightEgoBoundary, birdsEyeConfig, xVehiclePoints, 'Color','Green');
            
            frameWithOverlays = insertLaneBoundary(frame, leftEgoBoundary, sensor, xVehiclePoints, 'Color','Red');
            frameWithOverlays = insertLaneBoundary(frameWithOverlays, rightEgoBoundary, sensor, xVehiclePoints, 'Color','Green');
            
            frameWithOverlays = insertVehicleDetections(frameWithOverlays, locations, bboxes);
            
            imageROI = vehicleToImageROI(birdsEyeConfig, this.VehicleCoordSysROI);
            ROI = [imageROI(1) imageROI(3) imageROI(2)-imageROI(1) imageROI(4)-imageROI(3)];
            
            % Highlight candidate lane points that include outliers
            birdsEyeViewImage = insertShape(birdsEyeViewImage, 'rectangle', ROI); % show detection ROI
            birdsEyeViewImage = imoverlay(birdsEyeViewImage, birdsEyeViewBW, 'blue');
            
            % Display the results
            frames = {frameWithOverlays, birdsEyeViewImage, birdsEyeWithOverlays};
            
            persistent players;
            if isempty(players)
                frameNames = {'Lane marker and vehicle detections', 'Raw segmentation', 'Lane marker detections'};
                players = helperVideoPlayerSet(frames, frameNames);
            end
            update(players, frames);
            
            % terminate the loop when the first player is closed
            isPlayerOpen = isOpen(players, 1);
            
            if (~isPlayerOpen || closePlayers) % close down the other players
                clear players;
            end
        end
        
        
    end
end

%--------------------------------------------------------------------------
function imageROI = vehicleToImageROI(birdsEyeConfig, vehicleCoordSysROI)

vehicleCoordSysROI = double(vehicleCoordSysROI);

loc2 = abs(vehicleToImage(birdsEyeConfig, [vehicleCoordSysROI(2) vehicleCoordSysROI(4)]));
loc1 = abs(vehicleToImage(birdsEyeConfig, [vehicleCoordSysROI(1) vehicleCoordSysROI(4)]));
loc4 =     vehicleToImage(birdsEyeConfig, [vehicleCoordSysROI(1) vehicleCoordSysROI(4)]);
loc3 =     vehicleToImage(birdsEyeConfig, [vehicleCoordSysROI(1) vehicleCoordSysROI(3)]);

[minRoiX, maxRoiX, minRoiY, maxRoiY] = deal(loc4(1), loc3(1), loc2(2), loc1(2));

imageROI = round([minRoiX, maxRoiX, minRoiY, maxRoiY]);

end

%--------------------------------------------------------------------------
% Function that's used to reject some of the found curves
function isGood = ValidateBoundaryFcn(params)

if ~isempty(params)
    a = params(1);
    
    isGood = abs(a < 0.003); % not too bendy
else
    isGood = false;
end
end


%--------------------------------------------------------------------------
% Determine Lane Marker Types Classify lane boundaries as 'solid',
% 'dashed', etc.
%--------------------------------------------------------------------------
function boundaries = classifyLaneTypes(boundaries, boundaryPoints)

for bInd = 1 : numel(boundaries)
    vehiclePoints = boundaryPoints{bInd};
    % Sort by x
    vehiclePoints = sortrows(vehiclePoints, 1);
    
    xVehicle       = vehiclePoints(:,1);
    xVehicleUnique = unique(xVehicle);
    
    % Dashed vs Solid
    xdiff  = diff(xVehicleUnique);
    % Sufficiently large threshold to remove spaces between points of a
    % solid line, but not large enough to remove spaces between dashes
    xdifft = mean(xdiff) + 3*std(xdiff);
    largeGaps = xdiff(xdiff > xdifft);
    
    % Safe default
    boundaries(bInd).BoundaryType= LaneBoundaryType.Solid;
    if largeGaps>2
        % Ideally, these gaps should be consistent - but we cannot rely on
        % that unless we know that the ROI extent includes at least 3
        % dashes.
        boundaries(bInd).BoundaryType = LaneBoundaryType.Dashed;
    end
end

end

%--------------------------------------------------------------------------
function locations = computeVehicleLocations(bboxes, sensor)

locations = zeros(size(bboxes,1),2);
for i = 1:size(bboxes, 1)
    bbox  = bboxes(i, :);
    
    yBottom = bbox(2) + bbox(4) - 1;
    xCenter = bbox(1) + (bbox(3)-1)/2;
    
    locations(i,:) = imageToVehicle(sensor, [xCenter, yBottom]);
end
end

%--------------------------------------------------------------------------
% insertVehicleDetections function inserts bounding boxes and displays
% [x,y] locations corresponding to returned vehicle detections.
function imgOut = insertVehicleDetections(imgIn, locations, bboxes)

imgOut = imgIn;

for i = 1:size(locations, 1)
    location = locations(i, :);
    bbox     = bboxes(i, :);
        
    label = sprintf('X=%0.2f, Y=%0.2f', location(1), location(2));

    imgOut = insertObjectAnnotation(imgOut, ...
        'rectangle', bbox, label, 'Color','g');
end
end
