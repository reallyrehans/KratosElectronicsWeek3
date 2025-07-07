# Kratos Electronics Week 3
~ Rehan Siddiqui  

---
### Question 1 
* [q1_publisher.py](https://github.com/reallyrehans/week3_submission_rehansiddiqui/blob/main/q1_publisher.py)
    * Publishes `Hello World!` to the topic */new*
    * Rate of 15Hz (`time_period=1.0/15.0`)
* [q1_subscriber.py](https://github.com/reallyrehans/week3_submission_rehansiddiqui/blob/main/q1_subscriber.py)
    * Subscribed to the topic */new*
    * Print messages on terminal
---
### Question 2
* [q2_signal1.py](https://github.com/reallyrehans/week3_submission_rehansiddiqui/blob/main/q2_signal1.py)
    * Publishes `"red"` and `"green"` alternately on topic */s1* each for 10 seconds.
* [q2_signal2.py](https://github.com/reallyrehans/week3_submission_rehansiddiqui/blob/main/q2_signal2.py)
    * Subscribed to topic */s1*
    * Publishes opposite color to topic */s2*
---
### Question 3
* [q3_roverinfo.py](https://github.com/reallyrehans/week3_submission_rehansiddiqui/blob/main/q3_roverinfo.py)
    * Assumes standard values for rover information
    * Uses `Twist` and `Pole` from ROS2
    * Example output : 
    ~~~
    ROVER DATA at 2025-07-07 19:36:26        
    Velocity = Linear[0.0, 0.0, 0.0]        
            Angular[0.0, 0.0, 0.0]        
    Distance Travelled = 0.0 metres        
    Coordinates = Position[0.0, 0.0, 0.0]        
                Orientation[0.0, 0.0, 0.0, 1.0]        
    Battery Level = 100.0%        
    Time of Travel = 0.0 seconds        
    x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x
    ~~~
---
### Question 4
* [q4_seconds_node.py](https://github.com/reallyrehans/week3_submission_rehansiddiqui/blob/main/q4_seconds_node.py)
    * Publishing to topic */second*
    * Counter resets at 60
* [q4_minutes_node.py](https://github.com/reallyrehans/week3_submission_rehansiddiqui/blob/main/q4_minutes_node.py)
    * Subscribed to topic */second*
    * Counter increments at 60th second
    * Publishes to topic */minute*
    * Counter resets at 60
* [q4_hours_node.py](https://github.com/reallyrehans/week3_submission_rehansiddiqui/blob/main/q4_hours_node.py)  
    * Subscribed to topic */minute*
    * Counter increments at 60th minute
    * Publishes to topic */hour*
* [q4_format_node.py](https://github.com/reallyrehans/week3_submission_rehansiddiqui/blob/main/q4_format_node.py)
    * Subscribed to topic */second*
    * Subscribed to topic */minute*
    * Subscribed to topic */hour*
    * Publishes to topic */clock*
    * Uses f-string to publish time in `HH:MM:SS` format
    * Publishes on calllback from topic */second*
    * RQT Graph : 

![q4_clock_rosgraph](https://github.com/reallyrehans/week3_submission_rehansiddiqui/blob/main/q4_clock_rosgraph.png?raw=true)

---

### Bonus Question
To Be added


---
Back to [Google Classroom Week 3](https://classroom.google.com/c/Nzg2Njk5MzMyNTM4/a/NzgyODI2ODUwNjIz/details)
