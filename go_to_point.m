% Initializing the agents to random positions with barrier certificates. 
% This script shows how to initialize robots to a particular point.
% Sean Wilson
% 07/2019

N = 3;
% initial_positions = generate_initial_conditions(N, 'Spacing', 0.5);
rb = RobotariumBuilderProj2(); % Do not modify
initial_positions = [0.5 0 0.1; 0.1 0.3 -0.3 ; pi/2 pi-0.2 pi-0.1];
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_positions);

% Create a barrier certificate so that the robots don't collide
si_barrier_certificate = create_si_barrier_certificate('SafetyRadius', 0.05);
si_to_uni_dynamics = create_si_to_uni_dynamics();
        
%Get randomized initial conditions in the robotarium arena
% final_goal_points = generate_initial_conditions(N, ...
%     'Width', r.boundaries(2)-r.boundaries(1)-r.robot_diameter, ...
%     'Height', r.boundaries(4)-r.boundaries(3)-r.robot_diameter, ...
%     'Spacing', 0.5);
final_goal_points = [-0.3 .2 .1;0.2 0 0;0 0 0];
% We'll make the rotation error huge so that the initialization checker
% doesn't care about it
args = {'PositionError', 0.02, 'RotationError', 50};
init_checker = create_is_initialized(args{:});
controller = create_si_position_controller();

% Get initial location data for while loop condition.
x=r.get_poses();
r.step();
flag = 1;

while(~init_checker(x, final_goal_points))
    
    x = r.get_poses();
    dxi = controller(x(1:2, :), final_goal_points(1:2, :));
    dxi = si_barrier_certificate(dxi, x(1:2, :));
    dxu = si_to_uni_dynamics(dxi, x);
    r.set_velocities(1:N, dxu);
    r.step();
    if( abs(x(1,1)-final_goal_points(1,1))<0.05 && abs(x(1,2)-final_goal_points(1,2))<0.05 && flag==1)
        final_goal_points(1:2,1) = [-.3;0];
        flag = 2;
    end
    if( abs(x(1,1)-final_goal_points(1,1))<0.03 && abs(x(1,2)-final_goal_points(1,2))<0.03 && flag==2)
        final_goal_points(1:2,1) = [.3;0]; 
        flag = 0;
    end
end

% We can call this function to debug our experiment!  Fix all the errors
% before submitting to maximize the chance that your experiment runs
% successfully.
r.debug();

