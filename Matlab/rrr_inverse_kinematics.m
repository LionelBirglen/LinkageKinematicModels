function [theta1, theta2, theta3, success] = rrr_inverse_kinematics(L1, L2, L3, Px, Py, phi, elbow_config)
% RRR_INVERSE_KINEMATICS - Compute inverse kinematics of a planar RRR manipulator
%
% OBJECTIVE:
%   Calculate the joint angles required for a planar 3R serial linkage (RRR manipulator)
%   to reach a desired end-effector position and orientation.
%
% INPUTS:
%   L1, L2, L3   - lengths of link 1, link 2, and link 3 (units, e.g., mm)
%   Px, Py       - desired end-effector coordinates (mm)
%   phi          - desired end-effector orientation (rad)
%   elbow_config - +1 for elbow-up configuration, -1 for elbow-down
%
% OUTPUTS:
%   theta1 - angle of joint 1 relative to base frame (rad)
%   theta2 - relative angle between link 1 and link 2 (rad)
%   theta3 - relative angle between link 2 and link 3 (rad)
%   success - true if solution exists, false if target is unreachable
%
% USAGE EXAMPLE:
%   [t1, t2, t3, ok] = rrr_inverse_kinematics(57,46,51,35,125,deg2rad(116),1)
%         t1 =
%             0.6571
%         t2 =
%             0.6447
%         t3 =
%             0.7228
%         ok =
%           logical
%            1
%
% BY: 
% Prof. Lionel Birglen
% Polytechnique Montreal, 2025
% Last Update: 2025/05/15
% Contact: lionel.birglen@polymtl.ca
% 
% Code provided under GNU Affero General Public License v3.0


% Compute wrist position by subtracting the offset of link 3
Wx = Px - L3 * cos(phi);
Wy = Py - L3 * sin(phi);

% Distance squared from base to wrist center
R2 = Wx^2 + Wy^2;
R = sqrt(R2);

% Check reachability: law of cosines for theta2
cos_theta2 = (R2 - L1^2 - L2^2) / (2 * L1 * L2);
if abs(cos_theta2) > 1
    % Target unreachable
    theta1 = NaN; theta2 = NaN; theta3 = NaN; success = false;
    return;
end

% Compute theta2 with correct elbow configuration
theta2 = atan2(elbow_config * sqrt(1 - cos_theta2^2), cos_theta2);

% Compute theta1 from wrist position and link 2 orientation
k1 = L1 + L2 * cos(theta2);
k2 = L2 * sin(theta2);
theta1 = atan2(Wy, Wx) - atan2(k2, k1);

% Compute theta3 to match end-effector orientation
theta3 = phi - (theta1 + theta2);

success = true;
end
