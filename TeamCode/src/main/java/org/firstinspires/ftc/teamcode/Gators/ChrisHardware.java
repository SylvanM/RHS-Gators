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

package org.firstinspires.ftc.teamcode.Gators;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

public class ChrisHardware
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

    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public ChrisHardware() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontLeft       = hwMap.get(DcMotor.class, "front_left");
        frontRight  = hwMap.get(DcMotor.class, "front_right");
        backLeft        = hwMap.get(DcMotor.class, "back_left");
        backRight   = hwMap.get(DcMotor.class, "back_right");

        armRaisingMotor      = hwMap.get(DcMotor.class, "arm_raising");
        // armBaseMotor         = hwMap.get(DcMotor.class, "arm_base");
        armExtensionMotor    = hwMap.get(DcMotor.class, "arm_extension");

        leftClawServo        = hwMap.get(Servo.class, "left_servo");
        rightClawServo       = hwMap.get(Servo.class, "right_servo");

        // These might need to be changed
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        /*
        // Set all motors to zero power/
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
*/
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed. Encoders aren't installed yet
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // set certain motors to run with encoders
        armRaisingMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armExtensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // open claw
        openClaw();
    }

    //void rotateArm(double torque) {
    //    armBaseMotor.setPower(torque);
    //}

    void raiseArmBase(double torque) {
        armRaisingMotor.setPower(torque);
    }

    void extendArm(double power) {
        armExtensionMotor.setPower(power);
    }

    void setClaw(double position) {
        double left = 180 - position;
        double right = position;
        // must account for different position of the servos

        leftClawServo.setPosition(left);
        rightClawServo.setPosition(right);
    }

    void openClaw()  {
        setClaw(180.0);
    }

    void closeClaw() {
        setClaw(0.0);
    }

    void headSideways(Direction direction, double speed) {
        switch (direction) {
            case left:
                frontLeft.setPower(-speed);
                frontRight.setPower(-speed);

                backLeft.setPower(speed);
                backRight.setPower(speed);
            case right:
                frontLeft.setPower(speed);
                frontRight.setPower(speed);

                backLeft.setPower(-speed);
                backRight.setPower(-speed);
        }
    }

    void setMotorGroup(MotorGroup group, double power) {
        switch (group) {
            case port:
                setPorts(power);
            case starboard:
                setStarboards(power);
            case front:
                frontLeft.setPower(power);
                frontRight.setPower(power);
            case back:
                backLeft.setPower(power);
                backRight.setPower(power);
            case all:
                frontRight.setPower(power);
                frontLeft.setPower(power);
                backRight.setPower(power);
                backLeft.setPower(power);
                /*
                setStarboards(power);
                setPorts(power);
                */
        }
    }

    void setStarboards(double speed) {
        frontRight.setPower(speed);
        backRight.setPower(speed);
    }

    void setPorts(double speed) {
        frontLeft.setPower(speed);
        backLeft.setPower(speed);
    }

    // lets us have 2 values for direction
    enum Direction {
        left, right, forward, backward
    }

    enum MotorGroup {
        port, starboard, front, back, all
    }

}

