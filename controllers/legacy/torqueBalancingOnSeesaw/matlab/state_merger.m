function [ x ] = state_merger( ss_pos, ss_vel, robot_pos, robot_vel, robotDofs)
%STATE_MERGER Summary of this function goes here
%   Detailed explanation goes here

assert(size(robot_pos, 1) == robotDofs + 7);
assert(size(robot_vel, 1) == robotDofs + 6);

seesaw_pos_len = 4;
seesaw_vel_len = 3;
robot_pos_len = 7 + robotDofs;
robot_vel_len = 6 + robotDofs;

seesaw_pos_initial_index = 1;
robot_pos_initial_index = seesaw_pos_initial_index + seesaw_pos_len;
seesaw_vel_initial_index = robot_pos_initial_index + robot_pos_len;
robot_vel_initial_index = seesaw_vel_initial_index + seesaw_vel_len;

x = zeros(sum([seesaw_pos_len,seesaw_vel_len,robot_pos_len,robot_vel_len]), 1);

x(seesaw_pos_initial_index:seesaw_pos_initial_index + seesaw_pos_len - 1) = ss_pos;
x(robot_pos_initial_index : robot_pos_initial_index + robot_pos_len - 1) = robot_pos;
x(seesaw_vel_initial_index : seesaw_vel_initial_index + seesaw_vel_len - 1) = ss_vel;
x(robot_vel_initial_index : robot_vel_initial_index + robot_vel_len - 1) = robot_vel;


end

