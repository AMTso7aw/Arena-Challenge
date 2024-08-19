classdef Astar_frl
    properties
        map
        boundary
        obstacles
        path
        conv
    end
    
    methods
        function self = Astar_frl(startPos, endPos, map, obstacles, conv)
            self.map.XMAX = map.w;
            self.map.YMAX = map.h;

            self.map.start = endPos;
            self.map.goal = startPos;
            
            self.boundary = self.get_boundary();
            self.obstacles = obstacles;
            self.obstacles = [self.obstacles, self.boundary];

            self.conv = conv;
            self.path = self.A_star_search(self.map, self.conv);
        end
        
        function path = A_star_search(self, map, conv)
            path = []; %储存路径
            close = [];
            
            open = [map.start(1), map.start(2), ... %节点坐标
                    0 + self.h(map.start, map.goal, conv) , 0, ... %F=G+H,G
                    map.start(1), map.start(2)]; %父节点坐标
            
            next = self.motion_model();
            
            complete_flag = false; % 搜索完成时置为true，退出循环
            while ~complete_flag
                %首先判断是否达到目标点，或无路径
                if isempty(open(:, 1))
                    disp('No path to goal');
                    return;
                end

                %检查open列表中是否有目标点，若有，置complete_flag为true
                [isopen_flag, index] = self.isopen(map.goal, open);
                if isopen_flag
                    %disp('Path already found');
                    close = [open(index,:); close];
                    complete_flag = true;
                    break;
                end
                
                %a.以代价函数F(第三列)为依据对open列表排序，查找F值最小的节点
                [~, index] = sort(open(:, 3)); %获取F从小到大的顺序
                open = open(index, :); %将open列表按顺序重新排列
                
                %b.将F最小的节点转移到close列表的最前面(向后顺移)，作为当前节点
                close = [open(1, :); close];
                current = open(1, :);
                open(1, :) = []; %移动到close列表后从open列表中删除，之后不再关注
                
                %c.对当前节点周围的8个相邻节点进行操作：
                for in = 1:length(next(:, 1))
                    %获得邻近节点的坐标,
                    %先令代价值F等于0,代价值G等于0
                    %父节点的坐标值，暂定为零(暂时无法判断其父节点坐标)
                    vicinity = [current(1, 1) + next(in, 1), current(1, 2) + next(in, 2), ...
                                0, 0, ...
                                0, 0];
                    [find_type, target_index] = self.find_list(vicinity, open, close);
                    %find_type == 1：邻近节点在close列表中,target_index = close中的行号
                    %find_type == 2：邻近节点不在open列表中,target_index = []
                    %find_type == 3：邻近节点在open列表中,target_index = open中的行号

                    %若节点不可达或在close列表中，将其忽略并处理下一个邻近节点
                    if self.is_obstacle(vicinity) || find_type == 1
                        continue;
                    end
                    %对于需要忽略的节点，不需要计算G和F；现在计算没有被忽略的节点的G和F
                    vicinity(4) = current(1, 4) + next(in, 3); %相邻节点G值
                    vicinity(3) = vicinity(4) + self.h(vicinity(1:2), map.goal, conv);% m(3)  相邻节点F值
                    %如果它不在open列表中,加入open列表,并把当前节点设置为它的父节点
                    if find_type == 2 %不在open列表中
                        vicinity(5:6)=[current(1, 1), current(1, 2)]; %将当前节点作为其父节点
                        open = [open; vicinity]; %将此邻近节点加放进open列表中
                    %剩下的情况就是它在open列表中,检查由当前节点到相邻节点是否更好
                    %如果更好则将当前节点设置为其父节点,并更新F、G值,否则不操作
                    else
                        %由当前节点到达相邻节点更好
                        %(target_index是此相邻节点在open中的行号 此行的第3列是代价函数F值)
                        if vicinity(3) < open(target_index,3)
                            %更好，则将此相邻节点的父节点设置为当前节点，否则不作处理
                            vicinity(5:6)=[current(1, 1),current(1, 2)];%将当前节点作为其父节点
                            open(target_index, :) = vicinity;%将此相邻节点在Openlist中的数据更新
                        end
                    end
                end
            end
            path = self.get_path(close, self.map.start);
        end
        
        
        function [find_type, target_index] = find_list(~, vicinity, open, close)
            %find_type == 1：相邻节点在closelist中,target_index = close中的行号
            %find_type == 2：相邻节点不在openlist中,target_index = []
            %find_type == 3：相邻节点在openlist中,target_index = open中的行号
            %节点：在两个列表之外->先加入到open列表中->之后转移到close列表中
            %检测是否在open列表中(close列表中的节点来自open列表)
            find_type = 2;
            target_index = []; 
            if ~isempty(open) %open列表不为空，检测是否在open列表中
                for i_open = 1:length(open(:,1))
                    if isequal(vicinity(1:2), open(i_open, 1:2))
                        find_type = 3;
                        target_index = i_open;
                        return; %在open列表中，return  
                    end
                end   
            end
            %如果能到这一步，一定不在open列表中，判断是否在close列表中
            %检测是否在close列表中(close列表至少包含起点，必不为空)
            for i_close = 1:length(close(:, 1))
                if isequal(vicinity(1:2), close(i_close, 1:2))
                    find_type = 1;
                    target_index = i_close;
                    return; %在close列表中，return
                end
            end
        end
        
        function boundary = get_boundary(self)
            %获得地图的边界的坐标
            b=[];
            for i1 = 0:(self.map.YMAX + 1)
                b = [b, [0; i1]];
            end
            for i2 = 0:(self.map.XMAX + 1)
                b = [b, [i2; 0]];
            end
            for i3 = 0:(self.map.YMAX + 1)
                b = [b, [self.map.XMAX + 1; i3]];
            end
            for i4 = 0:(self.map.XMAX + 1)
                b = [b, [i4; self.map.YMAX + 1]];
            end
            boundary=b;
        end
        
        function path = get_path(~, close, start)
            index = 1;
            path = [];
            while 1
                path = [path; close(index, 1:2)];
                if isequal(close(index, 1:2), start)
                    break; %到达终点，跳出while
                end
                for i_close = 1:length(close(:, 1))
                    if isequal(close(i_close, 1:2), close(index, 5:6))
                        index = i_close; %查找父节点的index
                        break;
                    end
                end
            end
        end
        
        function h_cost = h(~, vicinity, goal, conv)
            dx = abs(vicinity(1) - goal(1));
            dy = abs(vicinity(2) - goal(2));
            D = 10;
            h_cost = D*(dx + dy) + (sqrt(2) - 2)*D*min(dx, dy);
            % h_cost = D*(dx + dy);
            h_cost = h_cost + 40*conv(vicinity(1), vicinity(2));
        end
        
        function flag = is_obstacle(self, vicinity)
            %判断邻近节点vicinity是否为障碍点，是返回true，不是返回false
            %先检查是否在已知障碍中
            for i = 1:length(self.obstacles(1, :))
                if isequal(self.obstacles(:, i)', vicinity(1:2))
                    flag = true;
                    return;
                end
            end
            if self.map.goal(1) ~= vicinity(1) || self.map.goal(2) ~= vicinity(2)
                if vicinity(2) == 50
                    flag = true;
                    return;
                end
                if vicinity(1) == 50
                    flag = true;
                    return;
                end  
            end
            flag = false;
        end
        
        function [isopen_flag, index] = isopen(~, node, open)
            %判断节点是否在open列表中，在open中，isopenFlag = 1,
            %不在open列表中，isopen_flag = 0
            isopen_flag = 0;
            index = 0;
            if ~isempty(open)
                for i = 1:length(open(:, 1))
                    if isequal(node(1:2), open(i, 1:2))  %在open列表中
                        isopen_flag = 1;
                        index = i;
                        return;
                    end
                end
            end
        end
        
        function next = motion_model(~)
            %当前节点  周围的八个相邻节点  与  当前节点的坐标差值（前两列）
            %当前节点  周围的八个相邻节点  与  当前节点的距离值（最后一列）
            next = [-1,  1, 14;
                     0,  1, 10;
                     1,  1, 14;
                    -1,  0, 10;
                     1,  0, 10;
                    -1, -1, 14;
                     0, -1, 10;
                     1, -1, 14
                   ];
            % next = [-0.5,  0.5, 7;
            %          0,    0.5, 5;
            %          0.5,  0.5, 7;
            %         -0.5,  0,   5;
            %          0.5,  0,   5;
            %         -0.5, -0.5, 7;
            %          0,   -0.5, 5;
            %          0.5, -0.5, 7
            %        ];
        end
    end
end
