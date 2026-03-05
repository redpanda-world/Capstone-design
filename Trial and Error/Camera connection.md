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

## How does it work?
ros2 run image_tools cam2image --ros-args -p burger_mode:=true
This command can generate hamburger video itself, and send it to laptop. If it works, we can find there's no problem about communication.


# Problem 2
Now that I understood that my network doesn't have any problem. But the it seems like camera doesn't work at all. I couldn't see the camera is seeing.

## Reason
Usually, camera is assigned as video0 and command cam2image is set to operate video0. However rasberrypi assigns my webcamera as video2.

## solution
>v4l2-ctl --list-devices

This command can show us the camera list that is connected to rasberrypi.

<img width="262" height="259" alt="image" src="https://github.com/user-attachments/assets/3bb5adcb-8451-4797-a92f-86dc358eac98" />

>ros2 run image_tools cam2image --ros-args -p width:=640 -p height:=480 -p device_id:=2

And operate this command in rasberrypi with changing device_id, then I could see the vision of camera.
I should have checked what camera is connected and its name.

# Problem 3
Now, I can see the sight that the robot is seeing. But it was way too slow. It was choppy.

# Reason
The reason was intuitive. The video that the rasberry pi was sending had too big size not to be choppy.

# Solution
>ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:="/dev/video2" -p image_size:="[320,240]"

Solution is obvious as well. We can diminish the size of image to decrease its bytes, and we can use run v4l2 command. This command is for webcamera and it can make it faster.


# Conclusion
In rasberrypi
>ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:="/dev/video2" -p image_size:="[320,240]"

In laptop
> ros2 run rqt_image_view rqt_image_view

