% This version realized first stage - spreading out but remain connected;
% second stage - switching back to a conglomerated configuration
% The CBF part is still centralized, needs to be decentralized in the
% future. The spanning tree identification was realized by 2 different ways
% so that the performance is different. 

clear all
clc

xrange = [-20,20];
yrange = [-20,20];
SpaceRadius = 4;
SpeedSize = 20;
R_limit = 5;

ParticleCounts = 20;

time_step = 0.01; %time step
time_period = 100;
% tm is the time sequence  
tm = time_step:time_step:time_period;
size_tm = size(tm);
length = size_tm(2); % length of tm

%connectivity = zeros(length,1);
%data_end = zeros (1,length(round_number));

%set initial random walks for every node
Initial_radius = SpaceRadius*sqrt(rand(ParticleCounts,1));
Initial_direction = 2*pi*rand(ParticleCounts,1);

%set initial locations for everyone
Initial_Location_x = Initial_radius.*cos(Initial_direction);
Initial_Location_y = Initial_radius.*sin(Initial_direction);
Initial_Location = [Initial_Location_x,Initial_Location_y];

%set location matrix for use in every step and initialize them
Location_x = Initial_Location_x;
Location_y = Initial_Location_y;
Location_x_unc = Initial_Location_x;
Location_y_unc = Initial_Location_y;
 
%v = VideoWriter("spread.mp4",'MPEG-4');
%open(v)

fig = figure;

%v = VideoWriter("Task_3_0423.mp4",'MPEG-4');
%open(v)


for RdCt = 1:length % every step
    %initialize matrix marking the nodes' status
    Inter_point_distance = zeros(ParticleCounts,ParticleCounts);
    Inter_point_distance_unc = zeros(ParticleCounts,ParticleCounts);

    %Now run every pair in a matrix
    for i = 1:ParticleCounts
    for j = 1:ParticleCounts
        % read the location and find out pairwise distances
        Inter_point_distance(i,j)= sqrt((Location_x(i)-Location_x(j))^2+(Location_y(i)-Location_y(j))^2);
        Inter_point_distance_unc(i,j)= sqrt((Location_x_unc(i)-Location_x_unc(j))^2+(Location_y_unc(i)-Location_y_unc(j))^2);
    end
    end

    %Now set all diagonal as zero
    for k = 1:ParticleCounts
     Inter_point_distance(k,k)=0;
     Inter_point_distance_unc(k,k)=0;
    end

    %Now create the graph
    G = graph(Inter_point_distance);
    G_unc = graph(Inter_point_distance_unc);

    % Now Remove edges that are too long
    for i = 1:ParticleCounts
    for j = 1:ParticleCounts        
        if Inter_point_distance (i,j) > R_limit+SpeedSize*time_step*2
            G = rmedge(G,i,j);
        end
         if Inter_point_distance_unc (i,j) > R_limit+SpeedSize*time_step*2
             G_unc = rmedge(G_unc,i,j);
         end
    end
    end

    % From now on, we let all robots taking turns to delete 1 or 0 edge

    %Critically_connected = ones(1,ParticleCounts); % every robot has a control. 0 means skip this robot, it is critically connected.
   

    for round = 1:ParticleCounts
        %stop_cutting = sum(Critically_connected); % all robots are critical. Stop cutting at 0.
        %if stop_cutting >0.5      % if keep cutting in this round, then take particles one by one
             for z = 1:ParticleCounts
                %if Critically_connected(z)>0.5 % if this robot is not yet critical, keeps going
                    moved_once = 0; % set moved_once as control. skip when it is 1.
                    % remove edges from the closest neighbor that is not in great demand for i (j is in i's cluster)
                    N = neighbors(G,z);
                    neighborsize = size(N);
                    NeighborSize = neighborsize(1);
                    if NeighborSize <1.5
                        moved_once = 1;
                    else
                    end
                   
                        distance_for_z = Inter_point_distance(z,N); %now create a distance list for z to try to kick out some edge
                        [val,ind] = sort(distance_for_z,'descend'); %find who is the farthest neighbor.
                        %[val,ind] = sort(distance_for_z);%find who is the closest neighbor.
                        Drop_List = N(ind); % this is a list of the tail index you are trying to kick
                            for w = 1:NeighborSize
                                if moved_once < 0.5 % if we haven't kicked out any for z in this round
                                    N1 = neighbors(G,z);
                                    N2 = neighbors(G,Drop_List(w));
                                    cluster_nodes = intersect(N2,N1);
                                    cluster = size(cluster_nodes);
                                    if cluster(1)>0.5
                                        G = rmedge(G,z,Drop_List(w));
                                        moved_once =1;
                                    else
                                    end
                                  
                                else
                                end
                            end

                            % if moved_once <0.5 % if you cannot kick out any
                            %     Critically_connected(z)=0;
                            % else
                            % end
                %else % if this robot is critically connected, skip.
                %end
            end

        % 
        % else
        % end
    end


    % Now Remove edges that are too short
    for i = 1:ParticleCounts
    for j = 1:ParticleCounts        
        if Inter_point_distance (i,j) < R_limit - SpeedSize*time_step*2
            G = rmedge(G,i,j);
        end
    end
    end

%[T,pred] = minspantree(G);
T = G;
SkipControl = numedges(G)<0.5;
[T_unc,pred] = minspantree(G_unc);
%T_unc = G_unc;
subplot(1,2,1)
plot(T,'XData',Location_x,'YData',Location_y,'EdgeLabel',T.Edges.Weight)
xlim(xrange)
ylim(yrange)
title('CBF on dist. generated subgraph')
subplot(1,2,2)
plot(T_unc,'XData',Location_x_unc,'YData',Location_y_unc,'EdgeLabel',T_unc.Edges.Weight)
xlim(xrange)
ylim(yrange)
title('No Control')
drawnow
%frame = getframe(fig);
%writeVideo(v,frame);

A_T = adjacency (T);
[Barrier_A_s,Barrier_A_t] = find (A_T); % starting node and tail node
Barrier_A_value = Inter_point_distance(A_T>0.5);



%Now let them move - randomly
angle = 2*pi*rand(ParticleCounts,1);
speed = SpeedSize*rand(ParticleCounts,1);
Velocity_x = speed.*cos(angle);
Velocity_y = speed.*sin(angle);

% %Now set optimization objectives:
% quad_H = eye(2*ParticleCounts);
% quad_f = zeros(2*ParticleCounts,1);
% quad_f(1:2:end-1) = -Velocity_x;
% quad_f(2:2:end) = -Velocity_y;
% 
% %Now set the barrier certificate:
% quad_A = [];
% quad_b = [];
% if SkipControl < 0.5
% for q = 1:numedges(G)
%     index1 = Barrier_A_s(q);
%     index2 = Barrier_A_t(q);
%     x1 = Location_x(index1);
%     x2 = Location_x(index2);
%     y1 = Location_y(index1);
%     y2 = Location_y(index2);
%     distance = Barrier_A_value(q);
%     h_function = R_limit^2 - distance^2;
%     L_h = 2*[x2-x1,x1-x2,y2-y1,y1-y2];
%     quad_A_temp = zeros(1,2*ParticleCounts);
%     quad_A_temp(2*index1-1:2*index1) = -2*[(x2-x1),(y2-y1)];
%     quad_A_temp(2*index2-1:2*index2) = -2*[(x1-x2),(y1-y2)];
%     quad_A = [quad_A;quad_A_temp];
%     quad_b = [quad_b;(x2-x1)*x1+(x1-x2)*x2+(y2-y1)*y1+(y1-y2)*y2+h_function];
% end
% 
% end

% [u_opt] = quadprog(quad_H,quad_f,quad_A,quad_b,[],[],[],[]);
% Velocity_x_actual = u_opt(1:2:end-1);
% Velocity_y_actual = u_opt(2:2:end);
% Location_x = Location_x + Velocity_x_actual*time_step;
% Location_y = Location_y + Velocity_y_actual*time_step;


% Now let's compute the control for each robot.
for p = 1:ParticleCounts
    Np = neighbors(G,p);
    neighborsize_p = size(Np);
    NSp = neighborsize_p(1);

% Set optimization objective for this robot:
quad_H = eye(2);
quad_f = zeros(2,1);
quad_f(1) = -Velocity_x(p);
quad_f(2) = -Velocity_y(p);

%Now set the barrier certificate:
quad_A = [];
quad_b = [];

if NSp < 0.5
 Velocity_x_actual = Velocity_x(p);
 Velocity_y_actual = Velocity_y(p);
else
for q = 1:NSp
    index1 = p;
    index2 = Np(q);
    x1 = Location_x(index1);
    x2 = Location_x(index2);
    y1 = Location_y(index1);
    y2 = Location_y(index2);
    distance = Inter_point_distance(p,q);
    h_function = R_limit^2 - distance^2;
    L_h = 2*[x2-x1,x1-x2,y2-y1,y1-y2];
    quad_A_temp = zeros(1,2);
    quad_A_temp(1:2) = -2*[(x2-x1),(y2-y1)];
    %quad_A_temp(2*index2-1:2*index2) = -2*[(x1-x2),(y1-y2)];
    quad_A = [quad_A;quad_A_temp];
    quad_b = [quad_b;(x2-x1)*x1+(x1-x2)*x2+(y2-y1)*y1+(y1-y2)*y2+h_function];
end

[u_opt] = quadprog(quad_H,quad_f,quad_A,quad_b,[],[],[],[]);
Velocity_x_actual = u_opt(1:2:end-1);
Velocity_y_actual = u_opt(2:2:end);

end

Location_x(p) = Location_x(p) + Velocity_x_actual*time_step;
Location_y(p) = Location_y(p) + Velocity_y_actual*time_step;

end




Location_x_unc = Location_x_unc + Velocity_x*time_step;
Location_y_unc = Location_y_unc + Velocity_y*time_step;


end

%From here we start to do the start shape movement


for RdCt = (length+1):(2*length) % every step
    %initialize matrix marking the nodes' status
    Inter_point_distance = zeros(ParticleCounts,ParticleCounts);
    Inter_point_distance_unc = zeros(ParticleCounts,ParticleCounts);

    %Now run every pair in a matrix
    for i = 1:ParticleCounts
    for j = 1:ParticleCounts
        % read the location and find out pairwise distances
        Inter_point_distance(i,j)= sqrt((Location_x(i)-Location_x(j))^2+(Location_y(i)-Location_y(j))^2);
        Inter_point_distance_unc(i,j)= sqrt((Location_x_unc(i)-Location_x_unc(j))^2+(Location_y_unc(i)-Location_y_unc(j))^2);
    end
    end

    %Now set all diagonal as zero
    for k = 1:ParticleCounts
     Inter_point_distance(k,k)=0;
     Inter_point_distance_unc(k,k)=0;
    end

    %Now create the graph
    G = graph(Inter_point_distance);
    G_unc = graph(Inter_point_distance_unc);

    % Now Remove edges that are too long
    for i = 1:ParticleCounts
    for j = 1:ParticleCounts        
        if Inter_point_distance (i,j) > R_limit+SpeedSize*time_step*2
            G = rmedge(G,i,j);
        end
         if Inter_point_distance_unc (i,j) > R_limit+SpeedSize*time_step*2
             G_unc = rmedge(G_unc,i,j);
         end
    end
    end

    % From now on, we let all robots taking turns to delete 1 or 0 edge

    %Critically_connected = ones(1,ParticleCounts); % every robot has a control. 0 means skip this robot, it is critically connected.
   

   % remove edges that is not in great demand for i (j is in i's cluster)
   for i = 1:ParticleCounts;
      N = neighbors(G,i);
      neighborsize = size(N);
      NeighborSize = neighborsize(1);
      for k = 1:NeighborSize
          for q = 1:NeighborSize
          Nq = neighbors(G,N(q));
          if ismember (N(k), Nq)
              G = rmedge(G,i,k);
          end
          end
      end
   end

    % Now Remove edges that are too short
    for i = 1:ParticleCounts
    for j = 1:ParticleCounts        
        if Inter_point_distance (i,j) < R_limit - SpeedSize*time_step*2
            G = rmedge(G,i,j);
        end
    end
    end

%[T,pred] = minspantree(G);
T = G;
SkipControl = numedges(G)<0.5;
[T_unc,pred] = minspantree(G_unc);
%T_unc = G_unc;
subplot(1,2,1)
plot(T,'XData',Location_x,'YData',Location_y,'EdgeLabel',T.Edges.Weight)
xlim(xrange)
ylim(yrange)
title('CBF on dist. generated subgraph')
subplot(1,2,2)
plot(T_unc,'XData',Location_x_unc,'YData',Location_y_unc,'EdgeLabel',T_unc.Edges.Weight)
xlim(xrange)
ylim(yrange)
title('No Control')
drawnow
%frame = getframe(fig);
%writeVideo(v,frame);

A_T = adjacency (T);
[Barrier_A_s,Barrier_A_t] = find (A_T); % starting node and tail node
Barrier_A_value = Inter_point_distance(A_T>0.5);



%Now let them move - randomly
angle = 2*pi*rand(ParticleCounts,1);
speed = SpeedSize*rand(ParticleCounts,1);
Velocity_x = speed.*cos(angle);
Velocity_y = speed.*sin(angle);

% %Now set optimization objectives:
% quad_H = eye(2*ParticleCounts);
% quad_f = zeros(2*ParticleCounts,1);
% quad_f(1:2:end-1) = -Velocity_x;
% quad_f(2:2:end) = -Velocity_y;
% 
% %Now set the barrier certificate:
% quad_A = [];
% quad_b = [];
% 
% if SkipControl < 0.5
% for q = 1:numedges(G)
%     index1 = Barrier_A_s(q);
%     index2 = Barrier_A_t(q);
%     x1 = Location_x(index1);
%     x2 = Location_x(index2);
%     y1 = Location_y(index1);
%     y2 = Location_y(index2);
%     distance = Barrier_A_value(q);
%     h_function = R_limit^2 - distance^2;
%     L_h = 2*[x2-x1,x1-x2,y2-y1,y1-y2];
%     quad_A_temp = zeros(1,2*ParticleCounts);
%     quad_A_temp(2*index1-1:2*index1) = -2*[(x2-x1),(y2-y1)];
%     quad_A_temp(2*index2-1:2*index2) = -2*[(x1-x2),(y1-y2)];
%     quad_A = [quad_A;quad_A_temp];
%     quad_b = [quad_b;(x2-x1)*x1+(x1-x2)*x2+(y2-y1)*y1+(y1-y2)*y2+h_function];
% end
% end
% 
% [u_opt] = quadprog(quad_H,quad_f,quad_A,quad_b,[],[],[],[]);
% Velocity_x_actual = u_opt(1:2:end-1);
% Velocity_y_actual = u_opt(2:2:end);
% Location_x = Location_x + Velocity_x_actual*time_step;
% Location_y = Location_y + Velocity_y_actual*time_step;

% 
% Now let's compute the control for each robot.
for p = 1:ParticleCounts
    Np = neighbors(G,p);
    neighborsize_p = size(Np);
    NSp = neighborsize_p(1);

% Set optimization objective for this robot:
quad_H = eye(2);
quad_f = zeros(2,1);
quad_f(1) = -Velocity_x(p);
quad_f(2) = -Velocity_y(p);

%Now set the barrier certificate:
quad_A = [];
quad_b = [];

if NSp < 0.5
 Velocity_x_actual = Velocity_x(p);
 Velocity_y_actual = Velocity_y(p);
else

for q = 1:NSp
    index1 = p;
    index2 = Np(q);
    x1 = Location_x(index1);
    x2 = Location_x(index2);
    y1 = Location_y(index1);
    y2 = Location_y(index2);
    distance = Inter_point_distance(p,q);
    h_function = R_limit^2 - distance^2;
    L_h = 2*[x2-x1,x1-x2,y2-y1,y1-y2];
    quad_A_temp = zeros(1,2);
    quad_A_temp(1:2) = -2*[(x2-x1),(y2-y1)];
    %quad_A_temp(2*index2-1:2*index2) = -2*[(x1-x2),(y1-y2)];
    quad_A = [quad_A;quad_A_temp];
    quad_b = [quad_b;(x2-x1)*x1+(x1-x2)*x2+(y2-y1)*y1+(y1-y2)*y2+h_function];
end

[u_opt] = quadprog(quad_H,quad_f,quad_A,quad_b,[],[],[],[]);
Velocity_x_actual = u_opt(1:2:end-1);
Velocity_y_actual = u_opt(2:2:end);

end

Location_x(p) = Location_x(p) + Velocity_x_actual*time_step;
Location_y(p) = Location_y(p) + Velocity_y_actual*time_step;

end


Location_x_unc = Location_x_unc + Velocity_x*time_step;
Location_y_unc = Location_y_unc + Velocity_y*time_step;


end