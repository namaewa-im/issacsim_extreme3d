# issacsim_extreme3d
Teleop devices for issacsim


### 2025-03-11 issue  
[Today's notion](./https://www.notion.so/joystick-ROS2-bridge-enable-1b2bcc8dc7cf8007b0a3ec791f04fb28?pvs=4)

[./isaaclab.sh -p scripts/environments/teleoperation/teleop_se3_agent.py --task Isaac-Lift-Cube-Franka-IK-Rel-v0 --num_envs 1 --teleop_device extreme3d](https://github.com/user-attachments/assets/a1a0ea0f-bfaf-4add-81ca-0ed0a18b8b44)


### 2025-03-12 issue  

[./isaaclab.sh -p scripts/tools/record_demos.py --task Isaac-Stack-Cube-Franka-IK-Rel-v0 --teleop_device extreme3d --dataset_file ./datasets/dataset.hdf5 --num_demos 10](https://github.com/user-attachments/assets/be65ed46-fcec-4e4e-86ff-a5d8da123137)

move_x가 task space에서 직선으로 움직이지 않는 문제

            # get keyboard command
            delta_pose, gripper_command = teleop_interface.advance()
            # convert to torch
            delta_pose = torch.tensor(delta_pose, dtype=torch.float, device=env.device).repeat(env.num_envs, 1)
            # compute actions based on environment
            actions = pre_process_actions(delta_pose, gripper_command)
--> 

            # get keyboard command
            delta_pose, gripper_command = teleop_interface.advance()
            delta_pose = delta_pose.astype("float32")
            # convert to torch
            delta_pose = torch.tensor(delta_pose, device=env.device).repeat(env.num_envs, 1)
            actions = pre_process_actions(delta_pose, gripper_command)
