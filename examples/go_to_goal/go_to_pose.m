% Initializing the agents to random positions with barrier certificates. 
% This script shows how to initialize robots to a particular pose.
% Paul Glotfelter edited by Sean Wilson
% 07/2019

N = 3;
% c = generate_initial_conditions(N, 'Spacing', 0.5);
initial_positions = [0.5 0 0.1; 0.1 0.3 -0.3 ; pi/2 pi-0.2 pi-0.1];
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_positions);

% Set some parameters for use with the barrier certificates.  We don't want
% our agents to collide

% Create a barrier certificate for use with the above parameters 
unicycle_barrier_certificate = create_uni_barrier_certificate_with_boundary();
        
%Get randomized initial conditions in the robotarium arena
% final_goal_points = generate_initial_conditions(N, ...
%     'Width', r.boundaries(2)-r.boundaries(1)-r.robot_diameter, ...
%     'Height', r.boundaries(4)-r.boundaries(3)-r.robot_diameter, ...
%     'Spacing', 0.5);
final_goal_points = [0.1 0.2 0.3;0 0 0;0 0 0];

args = {'PositionError', 0.025, 'RotationError', 0.05};
init_checker = create_is_initialized(args{:});
controller = create_waypoint_controller(args{:});

% Get initial location data for while loop condition.
x=r.get_poses();
r.step();

while(~init_checker(x, final_goal_points))
    x = r.get_poses();
    dxu = controller(x, final_goal_points);
    dxu = unicycle_barrier_certificate(dxu, x);      

    r.set_velocities(1:N, dxu);
    r.step();   
end

% We can call this function to debug our experiment!  Fix all the errors
% before submitting to maximize the chance that your experiment runs
% successfully.
r.debug();
