# fanuc_state_ros2
This package is for fetching robot state and restart the program using the IO and UI of Fanuc robot


To start robot program:

```bash 
ros2 service call /fanuc_state_ros2/start_program sereact_custom_messaging/srv/BoolSrv
```

To stop robot program

```bash 
ros2 service call /fanuc_state_ros2/stop_program sereact_custom_messaging/srv/BoolSrv
```

To reset fault

```bash 
ros2 service call /fanuc_state_ros2/reset_fault sereact_custom_messaging/srv/BoolSrv
```