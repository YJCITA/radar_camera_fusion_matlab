function annotatedFrame = helperAnnotateFCWDemoVideoFrame(frame, laneBoundaries, sensor, confirmedTracks, positionSelector, MIO)
%helperAnnotateFCWDemoVideoFrame  annotates a video frame in the Sensor
%Fusion and Forward Collision Warning Demo
%
%   This is an example helper function and is subject to change in future
%   releases.
%
%  tracks and most important object information for the Sensor Fusion and
%  Forward Collision Warning demo.

%   Copyright 2016 The MathWorks, Inc.

% XY points for lane markers
xRangeVehicle = [1 100];
xPtsInVehicle = linspace(xRangeVehicle(1), xRangeVehicle(2), 100)';

% Display the lane boundary on the video frame
frame = insertLaneBoundary(frame, laneBoundaries, sensor, xPtsInVehicle);

% Display tracks as bounding boxes on video frame
% Typical size of a vehicle on video display from 1 meter
boxSize = 1.25 * size(frame, 2);
annotatedFrame = insertTrackBoxes(frame, sensor, confirmedTracks, xRangeVehicle, ...
    MIO.ThreatColor, boxSize, MIO.ObjectID, positionSelector);
end
%--------------------------------------------------------------------------

function I = insertTrackBoxes(I, sensor, tracks, xVehicle, threatColor, boxSize, isMIO, positionSelector)
% insertTrackBoxes  Inserts bounding boxes in an image based on the
% distance in front of the ego vehicle, as measured by the track's position
% in front of the car
% Note: the function assumes that the first element in the position vector
% is the relative distance in front of the car

    % Define the classification values used in the tracking
    ClassificationValues = {'Unknown', 'UnknS', 'UnknB', ...
        'Pedestrian', 'Bike', 'Car', 'Truck', 'Barrier'};
    
    if isempty(tracks)
        return
    end

    % Extract the state vector from all the tracks    
    positions = getTrackPositions(tracks, positionSelector); % Gets a matrix of all the positions
    xs = positions(:,1);

    % Only use tracks that are confirmed and within the defined xVehicle range
    set1 = (xVehicle(1) <= xs);
    set2 = (xs <= xVehicle(2));    
    set = set1 .* set2;
    tracks = tracks(set == 1);
    xs = xs(set == 1);

    % Make sure the resulting set is not empty
    if isempty(tracks)
        return
    end

    % Sort in descending order by distance forward. This way, the last added
    % annotation will be the nearest.
    [~, sortindx] = sortrows(xs, -1);
    classes = [tracks(:).ObjectClassID]';

    % Memory allocation
    labels = cell(numel(tracks),1);
    bboxes = zeros(numel(tracks), 4);
    colors = cellstr(repmat('white', numel(tracks),1));

    for i = 1:numel(tracks)
        % Convert to image coordinates using monoCamera object
        xyVehicle = (positionSelector * tracks(sortindx(i)).State)';
        xyLocation = sensor.vehicleToImage(xyVehicle);

        % Estimate the bounding box around the vehicle
        size = boxSize/xyVehicle(1);
        bboxes(i,:) =[(xyLocation - [1.5*size, 2*size]), size*3, 2*size];

        % Add label: track ID + class
        labels{i} = [num2str(tracks(sortindx(i)).TrackID), '  ', ClassificationValues{classes(sortindx(i)) + 1}];        

        % The MIO gets the color based on FCW
        if tracks(sortindx(i)).TrackID == isMIO
            colors{i} = threatColor;
        end
    end
    I = insertObjectAnnotation(I, 'rectangle', bboxes, labels, 'Color', colors, ...
        'FontSize', 10, 'TextBoxOpacity', .8, 'LineWidth', 2);
end