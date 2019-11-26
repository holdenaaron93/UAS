function [flag, r, q, c, rho, lambda] = followWppFillet( w, p, R, newpath )
%followWppFillet implements waypoint following via straightline paths
%connected by fillets
%
% Inputs:
%   W = 3xn matrix of waypoints in NED (m)
%   p = position of MAV in NED (m)
%   R = fillet radius (m)
%   newpath = flag to initialize the algorithm or define new waypoints
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
% [flag, r, q, c, rho, lambda] = followWppFillet( w, p, R, newpath )
%

% Author: Randy Christensen
% Date: 14-Mar-2019 23:51:41
% Reference: Beard, Small Unmanned Aircraft, Chapter 11, Algorithm 6
% Copyright 2018 Utah State University

persistent i state
if isempty(i)
    i = 0;
    state = 0;
end
if newpath
    % Initialize index
    i = 2;
    state = 1;
    % Check sizes
    [m,N] = size(w);
    assert(N >=3);
    assert(m == 3);
else
    [m,N] = size(w);
    assert(N >=3);
    assert(m == 3);
end
% Calculate all the q vector and fillet angle
% persistent q
% persistent q_i

q    = (w(:,i)-w(:,i-1))/norm(w(:,i)-w(:,i-1));
q_i  = (w(:,i+1)-w(:,i))/norm(w(:,i+1)-w(:,i));
rho  = acos(-q.'*q_i);
% flag, r, q, c, rho, lambda
% Determine if the MAV is on a straight or orbit path
if state == 1
    flag = 1;
    r = w(:,i-1);
    q = q;
    c = nan(3,1);
    lambda = nan;
    z = w(:,i)-(R/(tan( rho/2)))*q;
    if in_half_plane(p,z,q)
        state = 2;
    end
elseif state == 2
    flag = 2;
    
    c = w(:,i)- (R/sin(rho/2))*(q - q_i)/norm(q - q_i);
    q = [1, 0, 0]';
    
    r = p;
    
    rho = R;
    lambda = sign(q(1)*q_i(2)-q(2)*q_i(1)); %check equation algo 6 line 19
    z = w(:,i) + (R/tan(rho/2))*q_i;
    if in_half_plane(p,z,q_i)
        if i < N-1
            i = i + 1; 
            
        end
        state = 1;
    end
        
    
else
    %Fly north as a default
    flag = -1;
    r = p;
    q = [1, 0, 0]';
    c = nan(3,1);
    rho = nan;
    lambda = nan;
end
end
