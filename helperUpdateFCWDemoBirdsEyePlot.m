function helperUpdateFCWDemoBirdsEyePlot(bepPlotters, laneBoundaries, visObjects, ...
    radObjects, confirmedTracks, positionSelector, velocitySelector, mostImportantTrack)
%helperUpdateFCWDemoBirdsEyePlot  Updates the bird's-eye plot in the Sensor
%Fusion and Forward Collision Warning Demo
%
%   This is an example helper function and is subject to change in future
%   releases.
%
%  Updates the bird's-eye plot with information about lane boundaries,
%  vision and radar detections, confirmed tracks and most important object.

%   Copyright 2016 The MathWorks, Inc.

% Prepare detections:
visObjPos = getDetectionPositions(visObjects);
radObjPos = getDetectionPositions(radObjects);

% Prepare tracks and most important object:
trackIDs = {confirmedTracks.TrackID};
trackLabels = cellfun(@num2str, trackIDs, 'UniformOutput', false);
[trackPositions, trackCovariances] = getTrackPositions(confirmedTracks, positionSelector);
trackVelocities = getTrackVelocities(confirmedTracks, velocitySelector);

% Update all BEP objects
% Display the birdsEyePlot
plotLaneBoundary(bepPlotters.LaneBoundary, laneBoundaries)
plotDetection(bepPlotters.Radar, radObjPos);
plotDetection(bepPlotters.Vision, visObjPos);
plotTrack(bepPlotters.Track, trackPositions, trackVelocities, trackCovariances, trackLabels);

bepPlotters.MIO.MarkerFaceColor = mostImportantTrack.ThreatColor;
plotTrack(bepPlotters.MIO, trackPositions(mostImportantTrack.TrackIndex,:), trackVelocities(mostImportantTrack.TrackIndex,:), trackLabels(mostImportantTrack.TrackIndex));
end
%--------------------------------------------------------------------------

function positions = getDetectionPositions(detections)
%getDetectionPositions  extracts the positions from recorded detections
if detections.numObjects > 0
    positions = [detections.object(1:detections.numObjects).position];
    positions = positions(1:2,:).';
else
    positions = zeros(0,2);
end
end