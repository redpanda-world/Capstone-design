# Problem 1
I neither connect camera to laptop nor figure out the reason.
The reason would be caused by either communication or hardware.
If I can check which one is reason, I could fix in the right way.
Then how can I check it?

## solution
To check if the reason is communication, I run a test called hamerger test.
After running ros2 launch command, run the command below in turtlebot3 terminal

>ros2 run image_tools cam2image --ros-args -p burger_mode:=true

and so in laptop.

>ros2 run rqt_image_view rqt_image_view


Then we can get floating hamberger video like below

<img width="555" height="489" alt="image" src="https://github.com/user-attachments/assets/95130d9f-8242-4c31-9b1a-25fca0f4bc1c" />

### How does it work?
ros2 run image_tools cam2image --ros-args -p burger_mode:=true
This command can generate hamburger video itself, and send it to laptop. If it works, we can find there's no problem about communication.

