function helperConfigureACCDemoCodeGen(modelName)
% Configure the ACC example model to generate C code with Embedded Coder
%
%   This is a helper function for example purposes and may be removed or
%   modified in the future.

% Copyright 2017 The MathWorks, Inc.

if nargin < 1
    modelName = bdroot;
end

% Check if the model is loaded. If not, load it
wasLoaded = bdIsLoaded(modelName);
if ~wasLoaded
    load_system(modelName)
end

% Set solver options for embedded coder. ert.tlc requires a fixed-step
set_param(modelName, 'SolverType', 'Fixed-step')

% Embedded Coder
set_param(modelName,'SystemTargetFile','ert.tlc')

% Revert things that were inadvertently set by selecting ERT
set_param(modelName,'MATLABDynamicMemAlloc','on')

% Enable common optimizations
set_param(modelName,'SuppressErrorStatus', 'on');
set_param(modelName,'InlineInvariantSignals', 'on');
set_param(modelName,'ZeroExternalMemoryAtStartup', 'on');
set_param(modelName,'CombineSignalStateStructs', 'on');
set_param(modelName,'BuildConfiguration', 'Faster Runs');
set_param(modelName,'ProdLongLongMode', 'on');
set_param(modelName,'OptimizeBlockOrder', 'Speed');

% Enable reporting
set_param(modelName,'GenerateCodeMetricsReport','on')

% Close the model if it wasn't loaded by the user
if ~wasLoaded
    close_system(modelName)
end