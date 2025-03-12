# issacsim_extreme3d
Teleop devices for issacsim

--
2025-03-11 issue  
[Today's notion](./https://www.notion.so/joystick-ROS2-bridge-enable-1b2bcc8dc7cf8007b0a3ec791f04fb28?pvs=4)

[./isaaclab.sh -p scripts/environments/teleoperation/teleop_se3_agent.py --task Isaac-Lift-Cube-Franka-IK-Rel-v0 --num_envs 1 --teleop_device extreme3d](https://github.com/user-attachments/assets/a1a0ea0f-bfaf-4add-81ca-0ed0a18b8b44)


--
2025-03-12  
[./isaaclab.sh -p scripts/tools/record_demos.py --task Isaac-Stack-Cube-Franka-IK-Rel-v0 --teleop_device extreme3d --dataset_file ./datasets/dataset.hdf5 --num_demos 10](https://github.com/user-attachments/assets/e853dc10-e1a5-4532-8b36-a914bcaab072)

move_x가 task space에서 직선으로 움직이지 않는 문제
