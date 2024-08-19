classdef Policy < handle
    properties
        point_current
        point_aim
        map
        scan_map
        processed_map
        sat_level
        collide
        pai = pi
        path
        path_no_update
        path_show
        t

        obstacles
        astar

        use_hybrid_astar = true;
        path_update_flag
        resolution = 1
        ValidationDistance = 0.1
        MinTurningRadius
        MotionPrimitiveLength
        AnalyticExpansionInterval = 1
        InterpolationDistance = 0.3

        PathPlannerParamsBasedOnMap = [2, 1*pi]
        %不同地图能跑通的MinTurningRadius和MotionPrimitiveLength的合适值如下
        %amap：[2, 1*pi]
        %a1map：[4, 2*pi] [2, 1*pi]均可
        %a2map：[4, 2*pi] 若不行请尝试 [6, 3*pi]
        %bmap：[4, 2*pi]
        %b1map：[2, 1*pi]
        %b2map：[2, 1*pi]
        %cmap：[2, 1*pi]
        %c1map：[1, 0.5*pi]
        
        u
        v
                
        %fc %尝试使用了模糊控制来控制uv，效果不是很好
        %w; z; u; v; e1; e2; zite; K; %尝试使用了单神经元来控制v，效果不是很好
        
    end

    methods
        
        function self = Policy()
            % self.w = [0.58 0.58 0.58]';
            % self.z = [0,0,0]';
            % self.v = 0;
            % self.e1 = 0;
            % self.e2 = 0;
            % self.zite = 2;
            % self.K = 4;
            self.MinTurningRadius = self.PathPlannerParamsBasedOnMap(1);
            self.MotionPrimitiveLength = self.PathPlannerParamsBasedOnMap(2);
            if self.use_hybrid_astar
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
            end
            self.map.w = 50;
            self.map.h = 50;
        end
        
        function action = action(self, observation)
            x_current = observation.agent.x;
            y_current = observation.agent.y;
            heading = mod(observation.agent.h, 2*pi);
            % disp(heading == 2*pi), disp(heading == 0)
            self.point_current = [x_current, y_current];
            self.point_aim = [observation.endPos.x, observation.endPos.y];
            self.map.w = size(observation.scanMap, 1);
            self.map.h = size(observation.scanMap, 2);
            self.sat_level = observation.agent.satLevel;
            self.collide = observation.collide;
            self.t = observation.t;

            if isempty(self.scan_map)
                self.scan_map = zeros(self.map.w, self.map.h);
            end
            if isempty(self.processed_map)
                self.processed_map = zeros(self.map.w, self.map.h);
            end

            self.path_update_flag = ~isequal(self.scan_map, ...
                max(self.scan_map, observation.scanMap));
            if self.path_update_flag
                self.scan_map = max(self.scan_map, observation.scanMap);
            end

            if self.use_hybrid_astar
                dmap = binaryOccupancyMap(self.scan_map);
                map_data = occupancyMatrix(dmap);
                %Env把地图转了90°
                map_data = rot90(map_data);
                %对地图做一些处理
            end

            se = strel('square', 5);
            %做闭运算，消除小缝隙
            if self.use_hybrid_astar
                self.processed_map = imclose(map_data, se);
            else
                self.processed_map = imclose(self.scan_map, se);
            end
            filter = ones(3, 3);
            %做一次卷积，增大障碍体积
            self.processed_map = conv2(self.processed_map, filter, 'same');
            self.processed_map(self.processed_map > 1) = 1;
            %确保当前点、终点不受刚才运算的影响
            self.processed_map(self.processed_map >= 1) = 1;
            self.processed_map(self.processed_map < 1) = 0;
            self.processed_map(self.point_aim(1) + 1, ...
                               self.point_aim(2) + 1) = 0;
            self.processed_map(floor(x_current + 1), ...
                               floor(y_current + 1)) = 0;

            if self.use_hybrid_astar
                startPose = [x_current, y_current, heading];
                goalPose = [self.point_aim(1) + 0.5, ...
                    self.point_aim(2) + 0.5,...
                    self.pai/2];
                if self.t < 4
                    self.MinTurningRadius = 2;
                    self.MotionPrimitiveLength = 1*pi;
                else
                    self.MinTurningRadius = self.PathPlannerParamsBasedOnMap(1);
                    self.MotionPrimitiveLength = self.PathPlannerParamsBasedOnMap(2);
                end
                
                % if self.path_update_flag
                self.path = codegenPathPlanner(...
                    self.processed_map, ...
                    startPose, ...
                    goalPose, ...
                    self.resolution, ...
                    self.ValidationDistance, ...
                    self.MinTurningRadius, ...
                    self.MotionPrimitiveLength, ...
                    self.AnalyticExpansionInterval, ...
                    self.InterpolationDistance ...
                );
                    % self.path_no_update = self.path;
                % % else
                %     goalPose = self.path(3, :)
                %     self.path_no_update = codegenPathPlanner(...
                %         self.processed_map, ...
                %         startPose, ...
                %         goalPose, ...
                %         self.resolution, ...
                %         self.ValidationDistance, ...
                %         self.MinTurningRadius, ...
                %         self.MotionPrimitiveLength, ...
                %         self.AnalyticExpansionInterval, ...
                %         self.InterpolationDistance ...
                %     );
                %     % self.path(1, :) = [];
                %     disp(self.path_no_update)
                % end
                
                delete_current_point_in_path(self, startPose);
                % disp(self.path)
                X = self.path(:, 1);
                Y = self.path(:, 2);
                H = self.path(:, 3);

                x_next = X(2); y_next = Y(2); heading_next = H(2);
                heading_next = mod(wrapTo2Pi(heading_next), 2*pi);

                [self.u, self.v] = solve_uv(self, x_current, y_current, ...
                                    heading, x_next, y_next, heading_next);
                
                % self.path(1, :) = [];

                % 环境的时间间隔是0.3s，故两个速度均要除以self.InterpolationDistance
                % disp(self.path(1, :))
                % disp(self.path(2, :))
                disp(isequal(self.path(1, :), self.path(2, :)))
                disp(' x_c:' + string(x_current) ...
                    + ' y_c:' + string(y_current) ...
                    + ' h_c:' + string(heading));
                disp(' x_n:' + string(x_next) ...
                    + ' y_n:' + string(y_next) ...
                    + ' h_n:' + string(heading_next));
                
            else
                filter = ones(3, 3);
                %再一次卷积之后用于计算H
                conv_map = conv2(self.processed_map, filter, 'same');
                
                %获取障碍
                [I,J] = find(self.processed_map >= 1);
                self.obstacles = [I J]';
    
                %Astar算法搜索得到路径
                self.point_current = [floor(x_current + 1), floor(y_current + 1)];
                self.astar = Astar_frl(self.point_current, self.point_aim, ...
                                       self.map, self.obstacles, conv_map);
                self.path = self.astar.path;
                %self.scan_map(self.scan_map ~= 1) = 0; %蓝色小车会移动
    
                %路径点
                X = self.path(:,1); Y = self.path(:,2); 
                
                x_next = X(2); y_next = Y(2);
                dx = x_next - x_current; dy = y_next - y_current;
                theta = atan2(dy, dx);
                theta = mod(theta, 2*pi);
                
                angle = mod(theta - heading, 2*pi);
                if(angle > 0.5*pi && angle < 1.5*pi)
                    self.v = -4*tan(angle);
                else
                    self.v = 4*tan(angle);
                end
                if abs(self.v) < 0.1
                    self.u = 1;
                else
                    self.u = 0.001;
                end
            end
            
            self.scan_map(self.scan_map > 1) = 0;%障碍小车扫描值>=2
            %绘制路径
            delete(self.path_show);
            self.path_show = scatter(self.path(:, 1), ...
                self.path(:, 2), 3, 'red', 'filled');

            action = [self.u, self.v];               
        end
        
        function flag = check_obstacle_around(self, point_current)
            range = 3;
            flag = false;
            x_current = floor(point_current(1) + 1);
            y_current = floor(point_current(2) + 1);
            x_min = max(1, x_current - range);
            x_max = min(x_current + range, self.map.w);
            y_min = max(1, y_current - range);
            y_max = min(y_current + range, self.map.h);
            for i = x_min:x_max
                for j = y_min:y_max
                    if self.processed_map(i,j) >= 1
                        flag = true;
                        disp(flag)
                    end
                end
            end
        end

        function [u, v] = solve_uv(self, x_current, y_current, heading, ...
            x_next, y_next, heading_next)
            x_error = x_next - x_current;
            y_error = y_next - y_current;

            angle = wrapToPi(heading_next - heading);
            theta = mean([heading_next, heading]);

            U_index = [abs(cos(theta)), abs(sin(theta))] >= 1e-3;
            U = [x_error/cos(theta), y_error/sin(theta)];
            U = U(U_index);% 排除分母为0的情况
            u = max(-1, min(mean(U)/self.InterpolationDistance, 1));
            v = angle/self.InterpolationDistance;

            disp(' x_e:' + string(x_error) ...
                + ' y_e:' + string(y_error) ...
                + ' θ:' + string(theta));
            disp(' U:' + string(U(:)) ...
                + ' u:' + string(u) ...
                + ' v:' + string(v));
        end

        function delete_current_point_in_path(self, startPose)
            delete_flag_1 = abs(self.path(1,1) - self.path(2,1)) < 1e-5 ...
                && abs(self.path(1,2) - self.path(2,2)) < 1e-5 ...
                && abs(self.path(1,3) - self.path(2,3)) < 1e-5;
            delete_flag_2 = abs(startPose(1) - self.path(2,1)) < 1e-3 ...
                && abs(startPose(2) - self.path(2,2)) < 1e-3 ...
                && abs(startPose(3) - self.path(2,3)) < 1e-3;
    
            if delete_flag_1 || delete_flag_2
                self.path(1, :) = [];
            end % 不主动删除会发生当前点和下一点重合导致速度为0的bug
        end

        %尝试使用单神经元控制来控制uv，效果不是很好
        % function calculate_v(self, angle)
        %     self.w = self.w + self.zite*angle*self.v*self.z;
        %     self.w = self.w./norm(self.w,2);  
        % 
        %     self.z(1) = angle - self.e1;
        %     self.z(2) = angle;
        %     self.z(3) = angle - 2*self.e1 + self.e2;
        % 
        %     self.v = self.v + self.K*self.w'*self.z;
        %     self.v = max(-self.sat_level(2), min(self.v, self.sat_level(2)));
        %     self.e2 = self.e1;
        %     self.e1 = angle;
        % end
        % 
        % function calculate_u(self)
        %     if abs(self.v) <= 0.05
        %         self.u = 1;
        %     elseif abs(self.v) <= 0.1
        %         self.u = 0.8;
        %     elseif abs(self.v) <= 0.2
        %         self.u = 0.5;
        %     elseif abs(self.v) <= 0.3
        %         self.u = 0.4;
        %     elseif abs(self.v) <= 0.4
        %         self.u = 0.2;
        %     else
        %         self.u = 0.02;
        %     end
        % end
        
    end
end

function wrapped_angle = wrapTo2pi(angle)
    angle = wrapTo2Pi(angle);
    if angle == 2*pi
        angle = 0;
    end
    wrapped_angle = angle;
end
