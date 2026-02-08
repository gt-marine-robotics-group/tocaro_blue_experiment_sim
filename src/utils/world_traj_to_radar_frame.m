function radar_traj = world_traj_to_radar_frame(world_traj, radar_pose_world)
%WORLD_TRAJ_TO_RADAR_FRAME Convert world trajectory into radar frame.

num_steps = numel(world_traj);
radar_traj(num_steps, 1) = Pose2();

cos_radar = cos(radar_pose_world.theta);
sin_radar = sin(radar_pose_world.theta);

for step_index = 1:num_steps
    delta_x_world = world_traj(step_index).x - radar_pose_world.x;
    delta_y_world = world_traj(step_index).y - radar_pose_world.y;

    x_radar =  cos_radar * delta_x_world + sin_radar * delta_y_world;
    y_radar = -sin_radar * delta_x_world + cos_radar * delta_y_world;

    theta_radar = wrap_to_pi(world_traj(step_index).theta - radar_pose_world.theta);

    radar_traj(step_index) = Pose2(x_radar, y_radar, theta_radar);
    radar_traj(step_index).timestamp  = world_traj(step_index).timestamp;
    radar_traj(step_index).covariance = world_traj(step_index).covariance;
end
end
