function [U] = controller(X)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% function [U] = controller(X)
%
% controller for the single-track model
%
% inputs: x (x position), y (y position), v (velocity), beta
% (side slip angle), psi (yaw angle), omega (yaw rate), x_dot (longitudinal
% velocity), y_dot (lateral velocity), psi_dot (yaw rate (redundant)), 
% varphi_dot (wheel rotary frequency)
%
% external inputs (from 'racetrack.mat'): t_r_x (x coordinate of right 
% racetrack boundary), t_r_y (y coordinate of right racetrack boundary),
% t_l_x (x coordinate of left racetrack boundary), t_l_y (y coordinate of
% left racetrack boundary)
%
% outputs: delta (steering angle ), G (gear 1 ... 5), F_b (braking
% force), zeta (braking force distribution), phi (gas pedal position)
%
% files requested: racetrack.mat
%
% This file is for use within the "Project Competition" of the "Concepts of
% Automatic Control" course at the University of Stuttgart, held by F.
% Allgoewer.
%
% prepared by J. M. Montenbruck, Dec. 2013 
% mailto:jan-maximilian.montenbruck@ist.uni-stuttgart.de
%
% written by *Huipanjun Tian, Jiaqi Qin*, *Mai. 2021*
% mailto:*st170504@stud.uni-stuttgart.de*


%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% INITIALIZATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% state vector

x=X(1); % x position
y=X(2); % y position
v=X(3); % velocity (strictly positive)
beta=X(4); % side slip angle
psi=X(5); % yaw angle
omega=X(6); % yaw rate
x_dot=X(7); % longitudinal velocity
y_dot=X(8); % lateral velocity
psi_dot=X(9); % yaw rate (redundant)
varphi_dot=X(10); % wheel rotary frequency (strictly positive)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% racetrack
% keep the track data unchanged for controller
persistent t_r t_l
if isempty(t_r)
    load('racetrack.mat','t_r'); % load right  boundary from *.mat file
    load('racetrack.mat','t_l'); % load left boundary from *.mat file
end

t_r_x=t_r(:,1); % x coordinate of right racetrack boundary
t_r_y=t_r(:,2); % y coordinate of right racetrack boundary
t_l_x=t_l(:,1); % x coordinate of left racetrack boundary
t_l_y=t_l(:,2); % y coordinate of left racetrack boundary

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% STATE FEEDBACK %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
delta=0; % steering angle
G = getGearStage(v); % get the best gear stage according to the current speed.; %gear 1 ... 5
Fb=0; % braking force
zeta=0.5; %braking force distribution
phi=0.3; % gas pedal position

%% Gear Controller
% gear transmission 3.91, 2.002, 1.33, 1, 0.805
function G = getGearStage(v)
    if v<7
        G=1;
    elseif v>=7 && v<14
        G=2;
    elseif v>=14 && v<21
        G=3;
    elseif v>=21 && v<28
        G=4;
    else
        G=5;
    end
end
%% Trajectory
% (x,y)_soll. for the moment, the middle line between the left and right border
t_x = (t_l_x+t_r_x)/2; 
t_y = (t_l_y+t_r_y)/2;

% Getting Q which stores the distances to all points of the trajectory
Q = [t_x - x* ones(length(t_x),1), t_y - y*ones(length(t_y),1)];
Q = cellfun(@norm,num2cell(Q,2));
[~, idx_t] = min(Q);

% target-position of x and y and target velocity v_s_vec in vector form ('soll')
x_s = t_x(idx_t);
y_s = t_y(idx_t);

% Back to the beginning
if idx_t+2 > length(t_x) 
    idx_t = 1; 
end

% Calculate the instantaneous speed with the next 2 points as the unit
v_s_vec = [t_x(idx_t+2) - x_s; t_y(idx_t+2) - y_s; 0]; 
v_vec = [x_dot; y_dot; 0 ]; % actual velocity

% calculation of angle alpha between actual velocity and target velocity
sigma = sign(dot(cross(v_vec, v_s_vec), [1, 1, 1]));
if norm(v_vec) ~= 0 && norm(v_s_vec) ~= 0
    % alpha = acos(dot(v_s_vec,v_vec))/(norm(v_s_vec)*norm(v_vec));
    tmp1 = dot(v_vec,v_s_vec);
    tmp2 = norm(v_vec)*norm(v_s_vec);
    tmp3 = tmp1/tmp2;
    alpha = acos(tmp3);
else
    alpha = 0;
end

%% velocity controller
%how many steps to look ahead
[steps_ahead,~] = pos2target_velocity(x,y); 
steps_ahead = 2; %initial

%target velocity at looked ahead position
[~,v_s] = pos2target_velocity(t_x(idx_t + steps_ahead), t_y(idx_t + steps_ahead)); 

if v < v_s
    % accelerate
    Fb = 0;
    phi = 0.7;
else
    % break
    Fb = (v-v_s)*150000;
    phi = 0;
end    

%%
% P-controller for yaw angle psi
K_p_psi = 5;% 9
delta = K_p_psi * sigma * alpha;

if Fb > 15000
    Fb = 15000;
end

% Condition for delta
if delta > 0.53
    delta = 0.53;
elseif delta < -0.53
    delta = -0.53;
end

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% OUTPUT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
U=[delta G Fb zeta phi]; % input vector
end

function [steps_ahead,v_s] = pos2target_velocity(x,y) %calculate how many steps to look ahead and target velocity at the inital point
% Case differentiation between different course segments

% straights
if x < 1 && x > -6 && y >= 0 && y < 185 || ...
    x < -29 && y >= 250 && y < 355 || ...
    x > 19 && x < 26 && y <= 400 && y > 345 || ...
    x > 29 && y <= 290 && y > 110
    v_s = 40;
    steps_ahead = 15;
% breaking1
elseif x < 1 && x > -6 && y >= 185 && y < 250
    v_s = 5;
    steps_ahead = 15;
% tight curves
elseif x < 1 && x > -21 && y >= 250 && y < 261 || ...
        x < -14 && x > -36 && y <=250
    v_s = 5;
    steps_ahead = 5;
% breaking2
elseif x < -29 && y >= 355 && y < 400
    v_s = 12;
    steps_ahead = 15;
% big curve on top
elseif y >= 400
    v_s = 12;
    steps_ahead = 10;
% breaking3
elseif x > 19 && x < 26 && y <= 345 && y > 305
    steps_ahead = 5;
    v_s = 5;
% chicanery
elseif x > 19 && x < 35 && y <= 305 && y > 290
    v_s = 5;
    steps_ahead = 5;
%breaking4
elseif x > 29 && y <= 110 && y > 45
    v_s = 7;
    steps_ahead = 5;
% first tight part of last curve    
elseif x > 29 && x < 61 && y <= 45 && y > 20
    v_s = 8;
    steps_ahead = 5;
% wide part of last curve
elseif x > 15 && y <= 20 && y > -19
    v_s = 13;
    steps_ahead = 10;
% second tight part of the last curve
elseif x <= 15 && y < 0 
    v_s = 8;
    steps_ahead = 5;
else
    error('point not in trajectory description')
end
end