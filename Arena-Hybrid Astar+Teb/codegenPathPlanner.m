function path = codegenPathPlanner(mapData, startPose, goalPose, ...
    resolution,ValidationDistance, ...
    MinTurningRadius, MotionPrimitiveLength, AnalyticExpansionInterval, ...
    InterpolationDistance)
    %#codegen
       
    % Create a binary occupancy map
    map = binaryOccupancyMap(mapData, 'res', resolution);

    % Create a state space object
    stateSpace = stateSpaceSE2;

    % Update state space bounds to be the same as map limits.
    stateSpace.StateBounds = [map.XWorldLimits; map.YWorldLimits;[0 2*pi]];

    % Construct a state validator object using the statespace and map object
    validator = validatorOccupancyMap(stateSpace,Map=map);
    
    % Set the validation distance for the validator
    validator.ValidationDistance = ValidationDistance;
    
    % Assign the state validator object to the plannerHybridAStar object
    planner = plannerHybridAStar( ...
        validator, ...
        'MinTurningRadius', MinTurningRadius, ...
        'MotionPrimitiveLength', MotionPrimitiveLength, ...
        'AnalyticExpansionInterval', AnalyticExpansionInterval, ...
        'ForwardCost', 1, ...
        'ReverseCost', 100, ...
        'InterpolationDistance', InterpolationDistance ...
        );
    
    % Compute a path for the given start and goal poses
    pathObj = plan(planner,startPose,goalPose,SearchMode='exhaustive');
    
    % Extract the path poses from the path object
    path = pathObj.States;

end
