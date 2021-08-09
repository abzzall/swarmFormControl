%%Constants
%places in formation

global Dx;
Dx=[-1, 0, 1, -1, 0, 1, -1, 0, 1];
global Dy;
Dy=[-1, -1, -1, 0, 0, 0, 1, 1, 1];
%%global constants
global ROBOT_RADIUS;
ROBOT_RADIUS=0.25;
global FORMATION_VELOSITY;
FORMATION_VELOSITY=1;  %step
global SAFE_DISTANCE;
SAFE_DISTANCE=3;
global NUM_ROBOTS;
NUM_ROBOTS=9;

global SENSOR_RANGE;
SENSOR_RANGE=10;
global SENSOR_FIELD_OF_VIEW;
SENSOR_FIELD_OF_VIEW=2*pi;
global SENSOR_COUNT;
SENSOR_COUNT=12;
global LIDAR_ANGLES;
LIDAR_ANGLES=(pi/12-pi):(pi/12):pi;


%%Environment SETUP
env = MultiRobotEnv(NUM_ROBOTS);
env.showTrajectory = false;
env.robotRadius = ROBOT_RADIUS;


MAP_SIZE=100;
p=[ones(1, MAP_SIZE+2); repmat([1, zeros(1, MAP_SIZE), 1], MAP_SIZE, 1); ones(1, MAP_SIZE+2)];
map=binaryOccupancyMap(p);
env.mapName='map';
%Sensors
lidars = cell(1,NUM_ROBOTS);
for rIdx = 1:NUM_ROBOTS
    lidar = MultiRobotLidarSensor;
    lidar.robotIdx = rIdx;
    lidar.scanAngles = LIDAR_ANGLES;
    lidar.maxRange = SENSOR_RANGE;
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
sampleTime = 0.1;              % Sample time [s]
tVec = 0:sampleTime:100;        % Time array                
%poses = 20*(rand(3,NUM_ROBOTS).*[1;1;2*pi]+[1;1;0]); %[0, 10]
for i=1:NUM_ROBOTS
   poses(1, i)=3;
   poses(2, i)=3+3*i;
   poses(3, i)=0;
end
goalPose=rand([2]).*90+[10, 10];
vel = zeros(3,NUM_ROBOTS);
ranges = cell(1,NUM_ROBOTS);

%loop
%for t=1:numel(tVec)
    env(1:NUM_ROBOTS, poses, ranges);
    pLeader=virtLeader(poses);        
    fprintf("tvec --- %d", tVec);

    for rIdx=1:NUM_ROBOTS
        lidar=lidars{rIdx};
        ranges{rIdx}=lidar();
        
        pose=poses(rIdx);
        v=vAvoidObs(ranges{rIdx})+vFormation(pose,pLeader, [Dx(rIdx), Dy(rIdx)] )+vGoal(pose, goalPose);
        pose=pose+v;
        pose(3)=normaliseAngle(pose(3));
        poses(:, rIdx) = pose;

        range=ranges{rIdx};
        LIDAR_ANGLES_C=(180/12-180):(180/12):180;
        fprintf("robot --- %d", rIdx);

        for i=1:length(LIDAR_ANGLES_C)
            LIDAR_ANGLES_C(i)
            range(i)
        end
            
    end
%end
