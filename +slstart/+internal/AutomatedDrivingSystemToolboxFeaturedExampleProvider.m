classdef AutomatedDrivingSystemToolboxFeaturedExampleProvider < slstart.internal.FeaturedExampleProvider
    % Featured Examples for the Automated Driving System Toolbox

    % Copyright 2017 The MathWorks, Inc.

    properties (GetAccess = public, SetAccess = private)
        % The customer visible product name this example ships with
        Product = 'Automated Driving System Toolbox';

        % The short name for this product as used by the Help Browser.
        ProductShortName = 'driving';

        % Featured examples in this product
        FeaturedExamples = {'SyntheticDataSimulinkExample','ACCTestBenchExample'};
    end
end
