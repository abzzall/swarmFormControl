%%Constants
%places in formation

%%global constants
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
isDead=zeros(NUM_ROBOTS, 1);
%loop
for t=1:numel(tVec)
    
    pLeader=virtLeader(poses(:, isDead==0));  
    waipoints=[pLeader; goalPose];
    %fprintf("tvec --- %d\n", tVec);
    %env(1:NUM_ROBOTS, poses, ranges);
    env(1:NUM_ROBOTS, poses,waipoints, ranges);    
    for rIdx=1:NUM_ROBOTS
        if ~isDead(rIdx)
            lidar=lidars{rIdx};
            ranges{rIdx}=lidar();
            pose=poses(1:2, rIdx)';
            v1=vAvoidObsMinAngle(ranges{rIdx});
            v2=vFormation(pose,pLeader, [Dx(rIdx), Dy(rIdx)] );
            v3=vGoal(pLeader, goalPose);
            v=v1+v2+v3;
            %v(3)=atan(v(2)/v(1));
            pose=pose+v;
            pose(3)=vectorOrientation(v);
            poses(:, rIdx) = pose;
        end 
        
    end
    isDead=checkDead(map,poses', isDead);

end
