% This is a helper function and may be changed or removed without notice.

%   Copyright 2016 The MathWorks, Inc.

% When publishing, function takes one snapshot of the figure fig when
% takeSnapshot is true and then closes the figure so that another snapshot
% will not be generated from it. Does nothing when not publishing.
function helperPublishSnapshot(fig, takeSnapshot)

if ishghandle(fig)
    hasSnapshot = takeSnapshot && ~isempty(snapnow('get'));
    if hasSnapshot
        figure(fig);
        snapnow;
        close(fig);
    end
end
end
