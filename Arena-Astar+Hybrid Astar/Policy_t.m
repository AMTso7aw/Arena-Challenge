classdef Policy_t < handle
    properties
        point_current
        point_aim
        map
        scan_map
        processed_map
        sat_level
        collide
        pai = 3.14
        path
        path_line

        path_update_flag
        resolution = 1
        ValidationDistance = 0.1
        MinTurningRadius = 4
        MotionPrimitiveLength = 2*pi
        AnalyticExpansionInterval = 1
        InterpolationDistance = 0.3
    end

    methods
        
        function self = Policy_t()
            self.path = codegenPathPlanner( ...
                zeros(50, 50), ...
                [2 3 0.835], ...
                [42 49 1.57], ...
                self.resolution, ...
                self.ValidationDistance, ...
                self.MinTurningRadius, ...
                self.MotionPrimitiveLength, ...
                self.AnalyticExpansionInterval, ...
                self.InterpolationDistance);
        end
        
        function action = action(self, observation)
            x_current = observation.agent.x;
            y_current = observation.agent.y;
            heading = mod(observation.agent.h, 2*self.pai);
            self.point_current = [floor(x_current + 1), floor(y_current + 1)];
            self.point_aim = [observation.endPos.x, observation.endPos.y];
            self.map.w = size(observation.scanMap, 1);
            self.map.h = size(observation.scanMap, 2);
            self.sat_level = observation.agent.satLevel;
            self.collide = observation.collide;

            if isempty(self.scan_map)
                self.scan_map = zeros(self.map.w, self.map.h);
            end
            if isempty(self.processed_map)
                self.processed_map = zeros(self.map.w, self.map.h);
            end
            
            self.path_update_flag = ~isequal(self.scan_map, observation.scanMap);
            if self.path_update_flag
                self.scan_map = max(self.scan_map, observation.scanMap);
            end
       
            dmap = binaryOccupancyMap(self.scan_map);
            map_data = occupancyMatrix(dmap);
            %为什么Env把地图转了90°，为什么为什么
            map_data = rot90(map_data);

            %对地图做一些处理
            se = strel('square', 5);
            %做闭运算，消除小缝隙
            self.processed_map = imclose(map_data, se);
            filter = ones(3, 3);
            %做卷积，增大障碍体积
            self.processed_map = conv2(self.processed_map, filter, 'same');
            % 确保当前点、终点不受刚才运算的影响
            self.processed_map(self.processed_map >= 1) = 1;
            self.processed_map(self.processed_map < 1) = 0;
            self.processed_map(self.point_aim(1) + 1, ...
                               self.point_aim(2) + 1) = 0;
            self.processed_map(floor(x_current + 1), ...
                               floor(y_current + 1)) = 0;

            startPose = [x_current, y_current, heading];
            goalPose = [self.point_aim(1) + 0.5, ...
                self.point_aim(2) + 0.5,...
                self.pai/2];
            self.path = codegenPathPlanner(...
                self.processed_map, ...
                startPose, ...
                goalPose, ...
                self.resolution, ...
                self.ValidationDistance, ...
                self.MinTurningRadius, ...
                self.MotionPrimitiveLength, ...
                self.AnalyticExpansionInterval, ...
                self.InterpolationDistance);

            delete_flag_1 = abs(self.path(1,1) - self.path(2,1)) < 1e-5 ...
                && abs(self.path(1,2) - self.path(2,2)) < 1e-5 ...
                && abs(self.path(1,3) - self.path(2,3)) < 1e-5;
            delete_flag_2 = abs(startPose(1) - self.path(2,1)) < 1e-3 ...
                && abs(startPose(2) - self.path(2,2)) < 1e-3 ...
                && abs(startPose(3) - self.path(2,3)) < 1e-3;

            if delete_flag_1 || delete_flag_2
                self.path(1, :) = [];
            end


            X = self.path(:, 1); Y = self.path(:, 2);H = self.path(:, 3);
    
            %绘制路径
            delete(self.path_line);
            self.path_line = scatter(X, Y, 3, 'red', 'filled');

            x_next = X(2); y_next = Y(2); heading_next = H(2);
            x_error = x_next - x_current;
            y_error = y_next - y_current;

            angle = wrapToPi(heading_next - heading);
            theta = (heading_next + heading)/2;

            U = [x_error/cos(theta), y_error/sin(theta)];
            % [~, index] = min(abs(U));
            % u = max(-1, min(U(index), 1));
            u = max(-1, min(mean(U)/self.InterpolationDistance, 1));
            v = angle/self.InterpolationDistance;

            action = [u, v];
            % disp(self.path(1, :))
            % disp(self.path(2, :))
            % disp(isequal(self.path(1, :), self.path(2, :)))
            % disp(' x_c:' + string(x_current) ...
            %     + ' y_c:' + string(y_current) ...
            %     + ' h_c:' + string(heading));
            % disp(' x_n:' + string(x_next) ...
            %     + ' y_n:' + string(y_next) ...
            %     + ' h_n:' + string(heading_next));
            % disp(' x_e:' + string(x_error) ...
            %     + ' y_e:' + string(y_error) ...
            %     + ' θ:' + string(theta));
            % disp(' U:' + string(U(1)) + ' ' + string(U(2)) ...
            %     + ' u:' + string(u) ...
            %     + ' v:' + string(v));
        end
    end
end

