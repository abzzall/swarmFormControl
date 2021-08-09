%Constants
%places in formation

global ROBOT_RADIUS;
ROBOT_RADIUS=0.25;
global FORMATION_VELOSITY;
FORMATION_VELOSITY=0.1;  %step
global SAFE_DISTANCE;
SAFE_DISTANCE=1;
global NUM_ROBOTS;
NUM_ROBOTS=4;

global SENSOR_RANGE;
SENSOR_RANGE=3;
global SENSOR_FIELD_OF_VIEW;
SENSOR_FIELD_OF_VIEW=2*pi;
global SENSOR_COUNT;
SENSOR_COUNT=12;
global LIDAR_ANGLES;
LIDAR_ANGLES=(pi/12-pi):(pi/12):pi;

global Dx;
%Dx=[-SAFE_DISTANCE-1, 0,  SAFE_DISTANCE+1, -SAFE_DISTANCE-1, 0, SAFE_DISTANCE+1, -SAFE_DISTANCE-1, 0, SAFE_DISTANCE+1];
Dx=[-2*SAFE_DISTANCE-1,  2*SAFE_DISTANCE+1, -2*SAFE_DISTANCE-1,2*SAFE_DISTANCE+1];
global Dy;
%Dy=[-1-SAFE_DISTANCE, -1-SAFE_DISTANCE,-1-SAFE_DISTANCE, 0, 0, 0,  1+SAFE_DISTANCE, 1+SAFE_DISTANCE, 1+SAFE_DISTANCE];
Dy=[-1-2*SAFE_DISTANCE, -1-2*SAFE_DISTANCE, 1+2*SAFE_DISTANCE, 1+2*SAFE_DISTANCE];
%figure(1)
%%Environment SETUP
env = MultiRobotEnv(NUM_ROBOTS);
env.showTrajectory = false;
env.robotRadius = ROBOT_RADIUS;
env.hasWaypoints = true;

MAP_SIZE=100;
p=[ones(1, MAP_SIZE+2); repmat([1, zeros(1, MAP_SIZE), 1], MAP_SIZE, 1); ones(1, MAP_SIZE+2)];
p(40:60, 40:60)=1;
map=binaryOccupancyMap(p);
env.mapName='map';
%Sensors
lidars = cell(1,NUM_ROBOTS);
for rIdx = 1:NUM_ROBOTS
    lidar = MultiRobotLidarSensor;
    lidar.robotIdx = rIdx;
    lidar.scanAngles = LIDAR_ANGLES;
    lidar.maxRange = SENSOR_RANGE;
    %lidar.sensorOffset=[0,0];
    attachLidarSensor(env,lidar);
    lidars{rIdx}=lidar;
end

%Robot detectors
% detectors = cell(1,NUM_ROBOTS);
% for rIdx = 1:NUM_ROBOTS
%     detector = RobotDetector(env,rIdx);
%     detector.maxDetections = NUM_ROBOTS;
%     detector.maxRange = SENSOR_RANGE;
%     detector.fieldOfView = SENSOR_FIELD_OF_VIEW;
%     detectors{rIdx} = detector;
% end


% env.plotSensorLines = false;


%%Simulation initialisation
sampleTime = 1;              % Sample time [s]
tVec = 0:sampleTime:10000;        % Time array                
poses = (rand(3,NUM_ROBOTS).*[20;20;2*pi]+[1;1;-pi]); %[0, 10]
goalPose=[90, 90]; %rand([2]).*10+[90, 90];
vel = zeros(3,NUM_ROBOTS);
ranges = cell(1,NUM_ROBOTS);


objects = [goalPose, 1];
env.objectColors = [1 0 0];
env.objectMarkers = 'so^';
%{
figure(2)
tiledlayout(NUM_ROBOTS, 1);
ax=[];
for i=1:NUM_ROBOTS
    ax=[ax, nexttile];
end

%hold(ax, 'on')
%}
%loop

T=numel(tVec);
P=zeros(NUM_ROBOTS,  T, 3);
PL=zeros(T, 2);

V=zeros(NUM_ROBOTS, T,4, 2);




for t=1:T
    
    
    pLeader=virtLeader(poses);  
    
    PL(t,:)=pLeader;
    
    waipoints=[pLeader; goalPose];
    %fprintf("tvec --- %d\n", tVec);
    %env(1:NUM_ROBOTS, poses, ranges);

%    figure(1)
    env(1:NUM_ROBOTS, poses,waipoints, ranges);
 %{
    d=false;
    if mod(t, 10)==0
    
        figure(2)
        d=true;
    end
    %}
    for rIdx=1:NUM_ROBOTS
        lidar=lidars{rIdx};
        ranges{rIdx}=lidar();
        pose=poses(1:2, rIdx)';
        v1=vAvoidObsMinAngle(ranges{rIdx});
        v2=vFormation(pose,pLeader, [Dx(rIdx), Dy(rIdx)] );
        v3=vGoal(pLeader, goalPose);
        v=v1+v2+v3;
        
        V(rIdx, t, 1, :)=v1;
        V(rIdx, t, 2, :)=v2;
        V(rIdx, t, 3, :)=v3;
        V(rIdx, t, 4, :)=v;
        
        
        %v(3)=atan(v(2)/v(1));
        pose=pose+v;
        pose(3)=vectorOrientation(v);
        poses(:, rIdx) = pose;
     %{
        if d
           hold(ax(rIdx), 'on')

           l1=vectorLength(v1);
           l2=vectorLength(v2);
           l3=vectorLength(v3);
           l=vectorLength(v);

           plot(ax(rIdx), t, l1,'Marker', '.',  'k')
            plot(ax(rIdx), t, l2,'Marker', '.', 'b')
            plot(ax(rIdx), t, l3,'Marker', '.', 'g')
            plot(ax(rIdx), t, l,'Marker', '.',  'r')



            plot(ax(rIdx), t, vectorOrientation(v1), 'Marker', '*', 'k')
            plot(ax(rIdx), t, vectorOrientation(v2), 'Marker', '*', 'b')
            plot(ax(rIdx), t, vectorOrientation(v3), 'Marker', '*', 'g')
            plot(ax(rIdx), t, vectorOrientation(v), 'Marker', '*', 'Color', [1, 1, 1])        

            plot(ax(rIdx), t, v1(1), 'Marker', 'x', 'k')
            plot(ax(rIdx), t, v2(1), 'Marker', 'x', 'b')
            plot(ax(rIdx), t, v3(1), 'Marker', 'x', 'g')
            plot(ax(rIdx), t, v(1), 'Marker', 'x', 'Color', [1, 1, 1])


            plot(ax(rIdx), t, v1(2), 'Marker', 'o', 'k')
            plot(ax(rIdx), t, v2(2), 'Marker', 'o', 'b')
            plot(ax(rIdx), t, v3(2), 'Marker', 'o', 'g')
            plot(ax(rIdx), t, v(2), 'Marker', 'o', 'Color', [1, 1, 1])


           hold(ax(rIdx), 'off')
        end
%}
    end



end
