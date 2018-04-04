function helperUpdateFCWDemoDisplay(frame, videoDisplayHandle, bepPlotters, ...
    laneBoundaries, sensor, confirmedTracks, mostImportantObject, positionSelector, velocitySelector, ...
    visObjects, radObjects)
%helperUpdateFCWDemoDisplay Updates the display for the forward collision warning demo
%
%   This is an example helper function and is subject to change in future
%   releases.
%
% This helper function updates the display for the forward collision
% warning example.
% Please note: a helper function may change without notice

%   Copyright 2016 The MathWorks, Inc.
updateFCWDemoVideoDisplay(videoDisplayHandle, frame, laneBoundaries, sensor, confirmedTracks, positionSelector, mostImportantObject);
helperUpdateFCWDemoBirdsEyePlot(bepPlotters, laneBoundaries, visObjects, radObjects, confirmedTracks, positionSelector, velocitySelector, mostImportantObject);

end
%--------------------------------------------------------------------------

function updateFCWDemoVideoDisplay(videoDisplayHandle, frame, laneBoundaries, sensor, confirmedTracks, positionSelector, MIO)
%updateFCWDemoVideoDisplay   updates the video display with a new annotated frame

% Call the helper function to annotate the frame
annotatedFrame = helperAnnotateFCWDemoVideoFrame(frame, laneBoundaries, sensor, confirmedTracks, positionSelector, MIO);

% Display the annotated frame
if isvalid(videoDisplayHandle)
    set(videoDisplayHandle, 'CData', annotatedFrame);
end
end