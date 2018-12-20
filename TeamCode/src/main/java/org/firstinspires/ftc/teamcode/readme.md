# Team Code

Code can be found in the "Gators" folder

## ChrisHardware
This is the class that establishes the hardware for the robot.
For every motor on the robot, there is a class instance of either a DcMotor or a Servo.

```java
public DcMotor frontPortDrive;      // front left wheel
public DcMotor frontStarboardDrive; // front right wheel
public DcMotor backPortDrive;       // back left wheel
public DcMotor backStarboardDrive;  // back right wheeel

public DcMotor armRaisingMotor;     // motor to raise and lower the arm
public DcMotor armBaseMotor;        // motor to rotate the arm
public DcMotor armExtensionMotor;   // motor to extend the arm

public Servo   leftClawServo;       // servo controlling left side of the claw
public Servo   rightClawServo;      // servo controlling right side of the claw
```

Note: A guide to programming control classes can be found [here](https://github.com/ftctechnh/ftc_app/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/readme.md)
