classdef helperCustomUI < handle % retains state.
    
    properties (Access = private)
        hPlotFig % figure to plot data
        hImageFig % figure for BEV points.
        
        hPlotAxes % handle to axes within hPlotFig
        hImageAxes % handle to axes within hImageFig
        
        vReader % videoreader for BEV plot.
        
        plottingData
    end
    
    methods
        function this = helperCustomUI(dataFileName)
            % Constructor: create the two figures, and add frame 0 data to
            % it. Setup the BEV image, and the VideoReader based on the
            % file name stored in the .MAT file.
            
            this.plottingData = load(dataFileName);

            uiName = ['Average Distance Plot'];
            this.hPlotFig = figure('tag', uiName);
            set(this.hPlotFig, 'menubar', 'none');
            set(this.hPlotFig, 'name', uiName);
            set(this.hPlotFig, 'NumberTitle','off');
            this.hPlotAxes = axes('Parent', this.hPlotFig);
            
            uiName = ['Birds Eye View'];
            this.hImageFig = figure('tag', uiName);
            set(this.hImageFig, 'menubar', 'none');
            set(this.hImageFig, 'name', uiName);
            set(this.hImageFig, 'NumberTitle','off');
            this.hImageAxes = axes('Parent', this.hImageFig);
            
            this.vReader = VideoReader(this.plottingData.videoName);
            vidFrame = readFrame(this.vReader);
            birdsEyeImage = transformImage(this.plottingData.monoSensorHelper.BirdsEyeConfig, vidFrame);
            imshow(birdsEyeImage, 'Parent', this.hImageAxes)
            
            this.updateUI(1);
        end
        
        function updateUI(this, val)
            % updateUI updates both the figures when a new time/index is
            % specified. The figures react to the new index and update
            % themselves based on this value. The plot doesn't change
            % except for the vertical 'time marker', but the BEV updates to
            % show the corresponding image, and points on it.
            
            if(val < 1)
               val = 1;
            end
            if(val >= this.lengthOfData())
               val = this.lengthOfData() - 1;
            end
            
            stem(this.hPlotAxes, this.plottingData.gtdata.Time, this.plottingData.averageDistance)
            hold(this.hPlotAxes, 'on')
            plot(this.hPlotAxes, [this.plottingData.gtdata.Time(val) this.plottingData.gtdata.Time(val)], [0 0.20], 'LineWidth', 1)
            hold(this.hPlotAxes, 'off')
            legend(this.hPlotAxes, 'left lane', 'right lane')
            grid(this.hPlotAxes, 'on')
            
            this.vReader.CurrentTime = val*(this.vReader.Duration)/this.lengthOfData();
            vidFrame = readFrame(this.vReader);
            birdsEyeImage = transformImage(this.plottingData.monoSensorHelper.BirdsEyeConfig, vidFrame);
            imshow(birdsEyeImage, 'Parent', this.hImageAxes)
            this.plotBEVPoints(val, this.plottingData.measurements.LanesInVehicleCoord, 'r', 'o', 'Estimate');
            this.plotBEVPoints(val, this.plottingData.gtdata.LanesInVehicleCoord, 'b', 'x', 'Ground Truth');
        end
        
        function len = lengthOfData(this)
            len = size(this.plottingData.averageDistance, 1);
        end

    end
    
    methods (Access = private)
       
        %------------------------------------------------------------------
        function plotBEVPoints(this, index, points, colorName, shape, legendName)
            
            if length(points{index}) == 2
                pointsVehicle = [points{index}{1}; points{index}{2}];
            elseif length(points{index}) == 1
                pointsVehicle = [points{index}{1}];
            else
                 pointsVehicle = [];
            end
            
            if ~isempty(pointsVehicle)
                ptsInBEV = vehicleToImage(this.plottingData.monoSensorHelper.BirdsEyeConfig, pointsVehicle);
                hold(this.hImageAxes, 'on')
                plot(this.hImageAxes, ptsInBEV(:,1), ptsInBEV(:,2), ...
                     shape, 'MarkerSize', 10, 'LineWidth', 2, ...
                     'Color', colorName, 'DisplayName', legendName)
                hold(this.hImageAxes, 'off')
                legend(this.hImageAxes,'show')
            end
            
        end
        
    end

end