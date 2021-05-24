# ultrasonicSensor -- Individual Progress – Yu Li

# Update (5/23)
  proj_v2 is fully functional and it will be able to calculate distance from an obstacle using HC-SR04 and display the distance serially with the serial communication feature of the launchpad, which is known as universal asynchronous receiver/transmitter (UART).


# Tasks
  •	One of the main aspects of this project involves integrating a MCU into our design. To achieve and integrate the launchpad, background research is needed.
  
  •	Motion sensors are required, so we need to determine which kind of sensor we will be using. 
# Progresses
  •	To start coding, we need to find out which software is suitable for this project. tested using Energia and Code Composer Studio. Energia gave me errors all the time and it is not the best option when dealing with the TI MSP432 launchpad, so I determined to use Code Composer Studio to do the implementation.
  
  •	I am having ultrasonic sensors and IR sensors on hand, but I founded that the ultrasonic sensor is easier for us to measuring the distance between the object and sensor. 
# Plan
  •	Continue research and connect ultrasonic sensor into the board.
  
  •	Write functional code so the sensor can properly detect whether there is an object and the distance between the object and the sensor itself. 

