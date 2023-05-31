function satelliteOrbitSimulation()
    % Create figure and panel
    fig = figure('Position', [100, 100, 800, 600]);
    panel = uipanel('Parent', fig, 'Position', [0.05, 0.05, 0.9, 0.9]);
    
    % Create text labels and input fields for orbital parameters
    parameters = {'Semimajor Axis (km):', 'Eccentricity:', 'Inclination (degrees):', 'Argument of Perigee (degrees):', 'Right Ascension of Ascending Node (degrees):', 'True Anomaly (degrees):'};
    numParameters = numel(parameters);
    
    parameterLabels = gobjects(numParameters, 1);
    parameterInputs = gobjects(numParameters, 1);
    
    for i = 1:numParameters
        parameterLabels(i) = uicontrol('Parent', panel, 'Style', 'text', 'String', parameters{i}, 'HorizontalAlignment', 'right', 'Position', [10, 550 - 50 * (i - 1), 200, 20]);
        parameterInputs(i) = uicontrol('Parent', panel, 'Style', 'edit', 'Position', [220, 550 - 50 * (i - 1), 100, 20]);
    end
    
    % Create text labels and input fields for velocity variations
    variationLabels = {'Velocity Variation in X:', 'Velocity Variation in Y:', 'Velocity Variation in Z:'};
    numVariations = numel(variationLabels);
    
    variationLabel = gobjects(numVariations, 1);
    variationInput = gobjects(numVariations, 1);
    
    for i = 1:numVariations
        variationLabel(i) = uicontrol('Parent', panel, 'Style', 'text', 'String', variationLabels{i}, 'HorizontalAlignment', 'right', 'Position', [350, 550 - 50 * (i - 1), 150, 20]);
        variationInput(i) = uicontrol('Parent', panel, 'Style', 'edit', 'Position', [520, 550 - 50 * (i - 1), 100, 20]);
    end
    
    % Create stop and interrupt buttons
    stopButton = uicontrol('Parent', panel, 'Style', 'pushbutton', 'String', 'Stop', 'Position', [10, 20, 100, 30], 'Callback', @(src, event) stopMotion());
    interruptButton = uicontrol('Parent', panel, 'Style', 'pushbutton', 'String', 'Interrupt', 'Position', [130, 20, 100, 30], 'Callback', @(src, event) interruptCode());
    
    % Create plot button
    plotButton = uicontrol('Parent', panel, 'Style', 'pushbutton', 'String', 'Plot', 'Position', [250, 20, 100, 30], 'Callback', @(src, event) plotSatelliteOrbit());
    
    % Initialize satellite position and velocity
    satellitePosition = [0; 0; 0];
    satelliteVelocity = [0; 0; 0];
    
    % Initialize motion flag and interrupt flag
    motionFlag = false;
    interruptFlag = false;
    
    % Create plot handle
    plotHandle = plot3(0, 0, 0, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    
    % Callback function to plot satellite orbit
    function plotSatelliteOrbit()
        % Get orbital parameters from input fields
        semimajorAxis = str2double(get(parameterInputs(1), 'String'));
        eccentricity = str2double(get(parameterInputs(2), 'String'));
        inclination = deg2rad(str2double(get(parameterInputs(3), 'String')));
        argumentOfPerigee = deg2rad(str2double(get(parameterInputs(4), 'String')));
        rightAscension = deg2rad(str2double(get(parameterInputs(5), 'String')));
        trueAnomaly = deg2rad(str2double(get(parameterInputs(6), 'String')));
        
        % Get velocity variations from input fields
        velocityVariations = [str2double(get(variationInput(1), 'String')); str2double(get(variationInput(2), 'String')); str2double(get(variationInput(3), 'String'))];
        
        % Run simulation to update satellite position over time
        tStart = 0;
        tEnd = 2 * pi;
        dt = 0.1;
        
        for t = tStart:dt:tEnd
            % Check if motion flag is set to stop the motion
            if ~motionFlag
                break;
            end
            
            % Update true anomaly based on time
            trueAnomaly = trueAnomaly + sqrt(G * semimajorAxis^3 / (semimajorAxis^3)) * dt;
            
            % Calculate satellite position at current time step
            r = (semimajorAxis * (1 - eccentricity^2)) / (1 + eccentricity * cos(trueAnomaly));
            satellitePosition = [r * cos(trueAnomaly); r * sin(trueAnomaly); 0];
            
            % Apply velocity variations
            satelliteVelocity = satelliteVelocity + velocityVariations;
            
            % Update satellite position based on velocity
            satellitePosition = satellitePosition + satelliteVelocity * dt;
            
            % Update plot
            set(plotHandle, 'XData', satellitePosition(1), 'YData', satellitePosition(2), 'ZData', satellitePosition(3));
            
            % Set axis labels and title
            xlabel('X');
            ylabel('Y');
            zlabel('Z');
            title('Satellite Orbit');
            
            % Set axis limits
            axis equal;
            xlim([-semimajorAxis, semimajorAxis]);
            ylim([-semimajorAxis, semimajorAxis]);
            zlim([-semimajorAxis, semimajorAxis]);
            
            % Pause for visualization
            pause(0.01);
            
            % Check if interrupt flag is set to interrupt the code execution
            if interruptFlag
                close all;
                return;
            end
        end
    end
    
    % Callback function to stop the motion
    function stopMotion()
        motionFlag = false;
    end
    
    % Callback function to interrupt the code execution
    function interruptCode()
        interruptFlag = true;
    end
end
