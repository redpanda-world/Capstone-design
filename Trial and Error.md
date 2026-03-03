# Problem.
When using turtlebot, I have to connect my labtop and robot. So I need ip address.
Then I have to always check the ip address after connecting monitor cable to robot,
and type command ifconfig to check it. Then finally I can connect it by command ssh ubuntu@ipaddress.
I have to one day perform this robot without monitor. THis has to be solved. I have to connect two without monitor.

# Solution.
I figured out the way not to check ip always.
just use ssh ubuntu@name_of_robot.local
Then all I need to know is its name, which is never changed.
