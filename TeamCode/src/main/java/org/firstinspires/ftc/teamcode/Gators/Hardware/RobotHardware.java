/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Gators.Hardware;

import android.graphics.Color;
import android.text.format.Time;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// sensor reading libraries
import com.qualcomm.hardware.bosch.BNO055IMU;         // IMU
import com.qualcomm.hardware.rev.Rev2mDistanceSensor; // Distance sensor
import com.qualcomm.robotcore.hardware.ColorSensor;   // Color Sensor

// navigation
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */

public class RobotHardware
{
    /* Public OpMode members. */
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    public DcMotor armRaisingMotor;
    public DcMotor armBaseMotor;
    public DcMotor armExtensionMotor;

    public Servo   leftClawServo;
    public Servo   rightClawServo;

    public Servo distanceSensorArmServo; // used for rotating the distance sensor

    /* sensors */
    public Rev2mDistanceSensor distanceSensor;
    public BNO055IMU imu;
    public ColorSensor colorSensor;

    public Orientation lastAngles = new Orientation();
    public double globalAngle, power = .30, correction;

    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    // Constants

    // all measurements in centimeters

    public static final double DISTANCE_BETWEEN_PARTICLES = 5;
    public static final double DISTANCE_SENSOR_OFFSET = 1; // distance from sensor to the left side of the robot
    public static final double WIDTH_OF_ROBOT = 18 - DISTANCE_SENSOR_OFFSET; // so that the robot knows when a ball is in front of it
    public static final double COLOR_SENSOR_RANGE = 2; // range that the color sensor can detect colors

    // Motor Locations

    public static final double EXTENDED_SENSOR_ARM_POSITION = 75 / 180;
    public static final double RETRACTED_SENSOR_ARM_POSITION = 0;


    /* local OpMode members. */
    private HardwareMap hwMap  =  null;
    private ElapsedTime period = new ElapsedTime();

    public RobotHardware() {

    }

    /* Initialize standard Hardware interfaces */

    /**
     * Function that initializes the robot from a hardware map
     * @param ahwMap
     */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Comment out if the robot does not have that hardware:

        // Define and Initialize Motors
        frontLeft  = hwMap.get(DcMotor.class, "front_left");
        frontRight = hwMap.get(DcMotor.class, "front_right");
        backLeft   = hwMap.get(DcMotor.class, "back_left");
        backRight  = hwMap.get(DcMotor.class, "back_right");

        //armRaisingMotor   = hwMap.get(DcMotor.class, "arm_raising");
        //armExtensionMotor = hwMap.get(DcMotor.class, "arm_extension");

        //leftClawServo  = hwMap.get(Servo.class, "left_servo");
        //rightClawServo = hwMap.get(Servo.class, "right_servo");

        // SENSORS
        //imu = hwMap.get(BNO055IMU.class, "imu");
        //distanceSensor = hwMap.get(Rev2mDistanceSensor.class, "distance_sensor");
        //colorSensor = hwMap.get(ColorSensor.class, "color_sensor");

        // These might need to be changed
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to run with encoders.
        // May want to use RUN_USING_ENCODER if encoders are installed.
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // set certain motors to run with encoders
        //armRaisingMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //armExtensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set up sensors

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.

        //BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        //parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        //parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        //parameters.loggingEnabled      = true;
        //parameters.loggingTag          = "IMU";
        //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        //imu.initialize(parameters);

    }

    // Movement functions - to be used primarily for AutoOp

    public void move(double linearSpeed, double rotarySpeed, double angle) {
        final double pi = Math.PI;

        double wheelCoefficients[] = new double[] {0, 0, 0, 0};

        wheelCoefficients[0] = linearSpeed * Math.sin(Math.toRadians(angle) + (pi / 4)) + rotarySpeed;
        wheelCoefficients[1] = linearSpeed * Math.cos(Math.toRadians(angle) + (pi / 4)) - rotarySpeed;
        wheelCoefficients[2] = linearSpeed * Math.cos(Math.toRadians(angle) + (pi / 4)) + rotarySpeed;
        wheelCoefficients[3] = linearSpeed * Math.sin(Math.toRadians(angle) + (pi / 4)) - rotarySpeed;

        frontLeft.setPower(wheelCoefficients[0]);
        frontRight.setPower(wheelCoefficients[1]);
        backLeft.setPower(wheelCoefficients[2]);
        backRight.setPower(wheelCoefficients[3]);
    }

    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }


    // Sensor functions

    public double getOrientation() {
        return AngleUnit.DEGREES.fromUnit(imu.getAngularOrientation().angleUnit, imu.getAngularOrientation().firstAngle);
    }

    public double getDistance() {
        return distanceSensor.getDistance(DistanceUnit.CM) + DISTANCE_SENSOR_OFFSET;
    }


    // Gadget functions

    // new gadget functions

    public void lowerDistanceSensor() {
        distanceSensorArmServo.setPosition(EXTENDED_SENSOR_ARM_POSITION);
    }
    public void raiseDistanceSensor() {
        distanceSensorArmServo.setPosition(RETRACTED_SENSOR_ARM_POSITION);
    }



    // Old functions
    public void raiseArmBase(double torque) {
        armRaisingMotor.setPower(torque);
    }

    public void extendArm(double power) {
        armExtensionMotor.setPower(power);
    }

    void setClaw(double position) {
        double left = 180 - position;
        double right = position;
        // must account for different position of the servos

        leftClawServo.setPosition(left);
        rightClawServo.setPosition(right);
    }

    public void openClaw()  {
        setClaw(180.0);
    }

    public void closeClaw() {
        setClaw(0.0);
    }

    // lets us have 2 values for direction
    enum Direction {
        left, right, forward, backward
    }

    enum MotorGroup {
        port, starboard, front, back, all
    }

}
