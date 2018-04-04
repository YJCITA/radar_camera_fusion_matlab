%------------------------------------------------
% Set up suitable environment for demo publishing
% -----------------------------------------------

% Location of large data used by the demos
largeDataUrl = [getenv('LARGE_TEST_DATA_ROOT') filesep 'Computer_Vision_System_Toolbox' filesep 'v000'];

% Add paths required to process the demos
addpath([matlabroot,'/test/tools/matlab/harness']);
addpath(largeDataUrl);

% Pre-load Simulink
load_simulink;
