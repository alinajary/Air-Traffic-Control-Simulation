%% Simulator Skeleton File - Project 2 

% This file provides the bare-bones requirements for interacting with the
% Robotarium.  Note that this code won't actually run.  You'll have to
% insert your own algorithm!  If you want to see some working code, check
% out the 'examples' folder.

close all

%% Get Robotarium object used to communicate with the robots/simulator
rb = RobotariumBuilderProj2(); % Do not modify

% Get the number of available agents from the Robotarium.  
N = 3;  % Do not modify (we will only be using 3 robots for this project)

% Set the number of agents and whether we would like to save data.  Then,
% build the Robotarium simulator object!
r = rb.set_number_of_agents(N).set_save_data(true).build(); % Do not modify

% Select the number of iterations for the experiment.
iterations = 1750; % Do not modify

% Other important variables
initpos1 = [.5; .1; pi/2];
initpos2 = [0; .3; pi-0.2];
initpos3 = [.1; -.3; pi-0.1];
target = [.3 .1 .2; 0 0 0];
targetalt = [.3 .1 .2; 0 0 0];
%%
%%%%%%%%%%%%%%%%%%%%%%%% Place Static Variables Here %%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%% Do not modify anything outside this area %%%%%%%%%%%%%%%
% a = 0;
final_goal_points = [-0.3 -.5 0.55;0.15 0.3 -0.25;0 0 0];    
z = 0;
args = {'PositionError', 0.02, 'RotationError', 50};
init_checker = create_is_initialized(args{:});
controller = create_si_position_controller();
% Create a barrier certificate so that the robots don't collide
si_barrier_certificate = create_si_barrier_certificate('SafetyRadius', 0.05);
si_to_uni_dynamics = create_si_to_uni_dynamics();  
flag = 1;  
flag_3 = 1; 
flag_2 = 1;
Final_goal_points = [0.1;0];
global p
% global minLinearVelocity
    
%%%%%%%%%%%%%% Do not modify anything outside this area %%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%
% Iterate for the previously specified number of iterations
for t = 1:iterations
    
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    p = r.get_poses();
    
    % Plot path traces every 20 iterations
    if mod(t,20) == 0
        plot(p(1,1),p(2,1),'k.',p(1,2),p(2,2),'m.',p(1,3),p(2,3),'b.');
    end
    
    % Success check with position tolerance embedded
    if isequal(round(p(1:2,:), 2), target) || isequal(round(p(1:2,:), 2), targetalt)
        fprintf('Success! The final iteration number is %d.\n', t);
        break
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%% Place Algorithm Here %%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%% Do not modify anything outside this area %%%%%%%%%%%%%%%
    % Do not include r.set_velocities(1:N, u) or r.step() in this area
    % It will make the iteration counter, t, inaccurate
    % Set u in this area only

    dxi = controller(p(1:2, :), final_goal_points(1:2, :));
    dxi = si_barrier_certificate(dxi, p(1:2, :));
    
    if( abs(p(1,1)-final_goal_points(1,1))<0.005 && abs(p(2,1)-final_goal_points(2,1))<0.005 && flag==0)
        dxi(:,1) = zeros(2,1);
    end
    if( abs(p(1,3)-final_goal_points(1,3))<0.005 && abs(p(2,3)-final_goal_points(2,3))<0.005 && flag==0)
        dxi(:,3) = zeros(2,1);
    end
    if( abs(p(1,2)-Final_goal_points(1))<0.005 && abs(p(2,2)-Final_goal_points(2))<0.005 && flag==0)
        dxi(:,2) = zeros(2,1);
    end
    
    
    dxu = si_to_uni_dynamics(dxi, p);
    
    
    if( abs(p(1,1)-final_goal_points(1,1))<0.05 && abs(p(2,1)-final_goal_points(2,1))<0.05 && flag==1)
        final_goal_points(1:2,1) = [-.3;0];
        flag = 2;
    end
    if( abs(p(1,1)-final_goal_points(1,1))<0.03 && abs(p(2,1)-final_goal_points(2,1))<0.03 && flag==2)
        final_goal_points(1:2,1) = [.3;0];
        flag = 0;
    end
    
    if( abs(p(1,3)-final_goal_points(1,3))<0.03 && abs(p(2,3)-final_goal_points(2,3))<0.03 && flag_3==1)
        final_goal_points(1:2,3) = [-0.5;-0.3];
        flag_3 = 2;
    end
    if( abs(p(1,3)-final_goal_points(1,3))<0.03 && abs(p(2,3)-final_goal_points(2,3))<0.03 && flag_3==2)
        final_goal_points(1:2,3) = [-0.3;0];
        flag_3 = 3;
    end
    if( abs(p(1,3)-final_goal_points(1,3))<0.03 && abs(p(2,3)-final_goal_points(2,3))<0.03 && flag_3==3)
        final_goal_points(1:2,3) = [0.2;0];
        flag_3 = 4;
    end
    
    if( abs(p(1,2)-final_goal_points(1,2))<0.03 && abs(p(2,2)-final_goal_points(2,2))<0.03 && flag_2==1)
        final_goal_points(1:2,2) = [-0.55;-0.3];
        flag_2 = 2;
    end
    if( abs(p(1,2)-final_goal_points(1,2))<0.03 && abs(p(2,2)-final_goal_points(2,2))<0.03 && flag_2==2)
        final_goal_points(1:2,2) = [-0.55;0.3];
        flag_2 = 3;
    end
     if( abs(p(1,2)-final_goal_points(1,2))<0.03 && abs(p(2,2)-final_goal_points(2,2))<0.03 && flag_2==3)
        final_goal_points(1:2,2) = [-0.3;0.3];
        flag_2 = 4;
    end
    if( abs(p(1,2)-final_goal_points(1,2))<0.03 && abs(p(2,2)-final_goal_points(2,2))<0.03 && flag_2==4)
        final_goal_points(1:2,2) = [-0.3;0];
        flag_2 = 5;
    end
    if( abs(p(1,2)-final_goal_points(1,2))<0.003 && abs(p(2,2)-final_goal_points(2,2))<0.003 && flag_2==5)
        final_goal_points(1:2,2) = [0.1;0];
        flag_2 = 6;
    end
    
    
    %%%%%%%%%%%%%% Do not modify anything outside this area %%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %% Send velocities to agents
    
    % Set velocities of agents 1,...,N
    r.set_velocities(1:N, dxu); % u is the input, [v; w], a 2x3 matrix for 3 robots 
    
    % Send the previously set velocities to the agents.  This function must be called!
    r.step(); 
end

    
% Call r.call_at_scripts_end() after our experiment is over!
r.call_at_scripts_end(); % Do not modify

%%%%%%%%%%%%%%%%%%%%%%%% Place Helper Functions Here %%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%% Do not modify anything outside this area %%%%%%%%%%%%%%%

% function a = foo(b)

  
    
    
%%%%%%%%%%%%%% Do not modify anything outside this area %%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

