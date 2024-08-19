classdef Policy < handle
    properties
        point_current
        point_aim
        map
        scan_map
        processed_map
        sat_level
        collide
        %pai = pi
        path
        path_no_update
        path_show
        t

        car

        use_hybrid_astar = true;
        path_update_flag
        resolution = 1
        ValidationDistance = 0.1
        MinTurningRadius
        MotionPrimitiveLength
        AnalyticExpansionInterval = 1
        InterpolationDistance = 0.3

        teb

        PathPlannerParamsBasedOnMap = [2, 1*pi]

        u, v
    end

    methods
        
        function self = Policy()
            self.MinTurningRadius = self.PathPlannerParamsBasedOnMap(1);
            self.MotionPrimitiveLength = self.PathPlannerParamsBasedOnMap(2);

            self.path = codegenPathPlanner( ...
                zeros(50, 50), ...
                [2 3 0.835], ...
                [42.5 49.5 pi/2], ...
                self.resolution, ...
                self.ValidationDistance, ...
                self.MinTurningRadius, ...
                self.MotionPrimitiveLength, ...
                self.AnalyticExpansionInterval, ...
                self.InterpolationDistance ...
                );
            % self.path_no_update = self.path;
            disp("请在更换地图时调整参数：MinTurningRadius，MotionPrimitiveLength")

            %self.teb = controllerTEB(self.path, zeros(50, 50));

            self.map.w = 50;
            self.map.h = 50;
            self.car.w = 3;
            self.car.ht = 2;
            self.u = 0;
            self.v = 0;
        end
        
        function action=action(self,observation)
            x_current = observation.agent.x;
            y_current = observation.agent.y;
            heading = mod(observation.agent.h, 2*pi);
            self.point_current = [x_current, y_current];
            self.point_aim = [observation.endPos.x, observation.endPos.y];
            self.map.w = size(observation.scanMap, 1);
            self.map.h = size(observation.scanMap, 2);
            self.sat_level = observation.agent.satLevel;
            self.collide = observation.collide;
            self.t = observation.t;
            self.car.w = observation.agent.w;
            self.car.ht = observation.agent.ht;

            if isempty(self.scan_map)
                self.scan_map = zeros(self.map.w, self.map.h);
            end
            if isempty(self.processed_map)
                self.processed_map = zeros(self.map.w, self.map.h);
            end
            
            self.path_update_flag = ~isequal(self.scan_map, ...
                max(self.scan_map, rot90(observation.scanMap)));
            if self.path_update_flag
                self.scan_map = max(self.scan_map, rot90(observation.scanMap));
            end %Env把地图转了90°

            bomap = binaryOccupancyMap(self.scan_map);
            map_data = occupancyMatrix(bomap);
            
            % map_data = rot90(map_data);
            %对地图做一些处理
            
            % global planner 使用hybrid astar
            startPose = [x_current, y_current, heading];
            goalPose = [self.point_aim(1) + 0.5, ...
                        self.point_aim(2) + 0.5,...
                        pi/2];
            % if self.path_update_flag
            self.path = codegenPathPlanner(...
                    map_data, ...
                    startPose, ...
                    goalPose, ...
                    self.resolution, ...
                    self.ValidationDistance, ...
                    self.MinTurningRadius, ...
                    self.MotionPrimitiveLength, ...
                    self.AnalyticExpansionInterval, ...
                    self.InterpolationDistance ...
            );
            % end
            
            % local planner 使用TEB
            self.teb = controllerTEB(self.path, bomap);
            self.teb.RobotInformation.Dimension ...
                = [self.car.w - 0.5, self.car.ht - 0.5];
            self.teb.RobotInformation.Shape = "Rectangle";
            self.teb.MinTurningRadius = 1;
            self.teb.ObstacleSafetyMargin = 0.5;
            self.teb.NumIteration = 2;
            self.teb.MaxVelocity = self.sat_level;
            self.teb.ReferenceDeltaTime = 0.3;
            self.teb.LookAheadTime = 10;
            self.teb.GoalTolerance = [0.1 0.1 0.1];

            [uv, ~, ~] = step(self.teb, startPose, [self.u, self.v]);
            self.u = uv(1, 1);
            self.v = uv(1, 2);
            % self.v = wrapToPi(uv(1, 2));
            % disp(uv(:,1))
            
            
            self.scan_map(self.scan_map > 1) = 0;
            %绘制路径
            delete(self.path_show);
            self.path_show = scatter(self.path(:, 1), ...
                self.path(:, 2), 3, 'red', 'filled');

            action = [self.u, self.v];
        end
    end
end