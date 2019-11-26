function [ flag, r, q, c, rho, lambda ] = followWppDubins( W, Chi, p, R, newpath)
%followWppDubins implements waypoint following via Dubins paths
%
% Inputs:
%   P = description (units)
%   p = description (units)
%   R = description (units)
%   newpath = description (units)
%
% Outputs
%   flag = flag for straight line path (1) or orbit (2)
%   r = origin of straight-line path in NED (m)
%   q = direction of straight-line path in NED (m)
%   c = center of orbit in NED (m)
%   rho = radius of orbit (m)
%   lambda = direction or orbit, 1 clockwise, -1 counter clockwise
%
% Example Usage
%  [ flag, r, q, c, rho, lambda ] = followWppDubins( P, p, R, newpath)
%

% Author: Randy Christensen
% Date: 15-Mar-2019 23:11:19
% Reference: Beard, Small Unmanned Aircraft, Chapter 11, Algorithm 8
% Copyright 2018 Utah State University



persistent i state;
if isempty(i)
    i = 0;
    state = 0;
end

if newpath
    i = 2;
    state = 1;
    [m,N] = size(W);
    assert(N >= 3,'Non enough vehicle configurations.');
    assert(m == 3);
else
    [m,N] = size(W);
    assert(N >= 3,'Non enough vehicle configurations.');
    assert(m == 3);
end
% Determine the Dubins path parameters


[dp] = findDubinsParameters(W(:,i-1), Chi(i-1), W(:,i), Chi(i), R);

L = dp.L; % path length (m)
c_s = dp.c_s; % start circle origin (m)
lambda_s = dp.lambda_s; % start circle direction (unitless)
c_e = dp.c_e; % end circle origin (m)
lambda_e = dp.lambda_e; % end circle direction (unitless)
z1 = dp.z_1; % Half-plane H_1 location (m)
q1 = dp.q_12; % Half-planes H_1 and H_2 unit normals (unitless)
z2 = dp.z_2; % Half-plane H_2 location (m)
z3 = dp.z_3; % Half-plane H_3 location  (m)
q3 = dp.q_3; % Half-plane H_3 unit normal (m)
%dp.case = case (unitless)

    %flag, r, q, c, rho, lambda
if state == 1
    %Follow start orbit until on the correct side of H1
%     q = [1, 0, 0]'
    q = nan(3,1); %
    r = nan(3,1); %
    
    
    flag = 2;
    c = c_s; 
    rho = R; 
    lambda = lambda_s;
    
    if in_half_plane(p,z1,-q1)
        state = 2;
    end
    
    
elseif state == 2
    %Continue following the start orbit until in H1
%     q = [1, 0, 0]'
    q = nan(3,1); %
    r = nan(3,1); %
    
    
    flag = 2; %
    c = c_s; %
    rho = R; %
    lambda = lambda_s; %
    
    
    if in_half_plane(p,z1,q1)
        state = 3;
        
        
    end
    
elseif state == 3
    %Transition to straight-line path until in H2
    %<insert code here>
    c = nan(3,1); %
    rho = nan; %
    lambda = nan; %
    
    flag = 1;
    r = z1; 
    q = q1;
    
    if in_half_plane(p,z2,q1)
        state = 4;
    end
    
    
elseif state == 4
    %Follow the end orbit until on the correct side of H3
    %<insert code here>
    
    q = nan(3,1); %
    r = nan(3,1); %
    
    flag = 2; 
    c = c_e; 
    rho = R; 
    lambda = lambda_e;
    
    if in_half_plane(p,z3,-q3)
        state = 5;
    end
    
   
else %state == 5
    %Continue following the end orbit until in H3

    q = nan(3,1); % 
    r = nan(3,1);  %  
    
    flag = 2; % 
    c = c_e; %
    rho = R; %
    lambda = lambda_e; %
    
    if in_half_plane(p,z3,q3)
        
        state = 1;
        %flag = 2;
        
        if i < N
         i = (i+1);
        end
        
%         [dp] = findDubinsParameters(W(:,i-1), Chi(i-1), W(:,i), Chi(i), R);
%         L = dp.L; % path length (m)
%         c_s = dp.c_s; % start circle origin (m)
%         lambda_s = dp.lambda_s; % start circle direction (unitless)
%         c_e = dp.c_e; % end circle origin (m)
%         lambda_e = dp.lambda_e; % end circle direction (unitless)
%         z1 = dp.z_1; % Half-plane H_1 location (m)
%         q1 = dp.q_12; % Half-planes H_1 and H_2 unit normals (unitless)
%         z2 = dp.z_2; % Half-plane H_2 location (m)
%         z3 = dp.z_3; % Half-plane H_3 location  (m)
%         q3 = dp.q_3; % Half-plane H_3 unit normal (m)   
%         state  = dp.case; % case (unitless)
       
    end


end


end

