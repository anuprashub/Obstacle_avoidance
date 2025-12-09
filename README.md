# Autonomous Obstacle-Avoiding Robot Using ESP32-S3 CAM & Yolov8

## 1. Project Overview

    This project demonstrates a fully autonomous robotic system that integrates embedded systems, sensors, computer vision, and control.

    An ESP32-S3 CAM captures live video and streams it to a computer.
    A Python script uses YOLOv8n to detect obstacles and send steering commands back to the robot.
    Additionally, two HC-SR04 ultrasonic sensors allow the robot to decide safe turning directions.

    The car responds :

        Obstacle left → turn right
        Obstacle right → turn left
        Obstacle center → choose direction using ultrasonic sensors
        No obstacle → move forward

## 2. Motivation
    I was always interested in autonomous vehicles and I wanted to build a car that can avoid obstacle while being one road.

## 3. Hardware Used
    Component	                        Quantity	    Purpose                                 Cost
    -------------------------------------------------------------------------------------------------
    ESP32-S3 CAM                    	    1	        Camera streaming + motor control        $17
    L298N Motor Driver	                    1	        Drives left/right motors                $4
    DC Gear Motors (RC car kit)	            4	        Locomotion                              $25
    HC-SR04 Ultrasonic Sensors	            2	        Detect left/right free space            $6
    18650 / AA Battery Pack	                4	        Motor power                             $3
    Power bank	                            1	        To power ESP32                          $15
    Jumper Wires	                        –	        Wiring                                  $4
    -------------------------------------------------------------------------------------------------
    Total                                                                                     ~ $73

## 4. System Architecture

    ESP32-S3 → streams video → PC
    PC runs YOLOv8 → detects obstacle direction
    PC sends movement commands → ESP32 via Wi-Fi
    ESP32 → controls L298N → moves car
    Ultrasonic sensors → decide turning direction when obstacle is centered

    Inputs

        Camera frames (video)
        Ultrasonic sensor distances

    Outputs

    Motor direction commands:

        FORWARD
        BACKWARD
        TURN_LEFT
        TURN_RIGHT
        STOP

## 5. Circuit Diagrams

    Motor Driver Wiring (L298N)
    L298N Pin	ESP32-S3
    IN1	            GPIO 33
    IN2	            GPIO 34
    IN3	            GPIO 35
    IN4	            GPIO 36
    ENA	            GPIO 46
    ENB	            GPIO 45

    +12V	Battery
    GND	Battery + ESP32 GND

[Circuit diagram](circuit_diagram.png)


## 6. Code Overview

    src/main.c                     →   ESP32 firmware (camera + TCP + motors + ultrasonic)
   
    src/motors.c & motors.h        →   motor driver code
   
    training.py                →   YOLO training script
   
    pc_esp_yolo.py         →   YOLO detection + TCP sender

    To install python dependencies run command bellow in your environment:

       pip install ultralytics opencv-python numpy
    
## 7. Limitations

    - Processing happens on PC, not on ESP32
    - Latency depends on Wi-Fi quality
    - Ultrasonic sensors only give left/right free-space, not depth map
    - No lane detection.

## 8. Future work

    - Use custom lightweight models for faster processing.
    - Replace esp32 with Raspberry pi or Jetson nano for on board processing

## 9. Reference

    - Espressif Systems – ESP-IDF Official Documentation
      https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/

    - Espressif ESP32 Camera Driver
      https://github.com/espressif/esp32-camera

    - FreeRTOS
      https://www.freertos.org/Documentation/00-Overview

    - Ultralytics YOLOv8 Framework
      https://docs.ultralytics.com
