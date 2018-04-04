function htmlfilename = drivingdemohelp
% DRIVINGDEMOHELP Automated Driving System Toolbox demo help function.
%   DRIVINGDEMOHELP Returns the path to the HTML help information file
%   corresponding to the currently selected demo model.

%  This function enables multiple versions of one demo
%  to point to the same doc page.
%
%  REQUIREMENTS:
%     1) The info block calling this function
%        MUST be in the root of the demo model.
%     2) The name of the demo must end with a combination
%        of no more than two items from the following list:
%        _all, _frame, _fixpt, _win or _win32, _color, _intensity.

% Copyright 2016-2017 The MathWorks, Inc.

modelname = get_param(gcb, 'Parent');

% Remove _win32 or _fixpt (or _***) if present (before the ".")
htmlfilename = suffixstrip(modelname);

% Make a second pass to allow for (and remove) two levels of
% suffixes: _fixpt_all or _color_all
htmlfilename = suffixstrip(htmlfilename);

htmlfilename = [matlabroot '/toolbox/driving/drivingdemos/html/' htmlfilename '.html'];

function htmlfilename=suffixstrip(modelname)
htmlfilename = regexprep(modelname, '_frame.*$', '');
htmlfilename = regexprep(htmlfilename, '_fixpt.*$', '');
htmlfilename = regexprep(htmlfilename, '_all.*$', '');
htmlfilename = regexprep(htmlfilename, '_win.*$', '');
htmlfilename = regexprep(htmlfilename, '_color.*$', '');
htmlfilename = regexprep(htmlfilename, '_intensity.*$', '');
htmlfilename = regexprep(htmlfilename, '_mic.*$', '');
htmlfilename = regexprep(htmlfilename, '_audio.*$', '');
htmlfilename = regexprep(htmlfilename, '_vr.*$', '');

