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

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotHardware
{

    /* Constants and properties */

    // position of the lift
    public int liftPosition = MAX_LIFT_DOWN;

    // power for lift
    private double liftPower = 0.3;

    /* Public OpMode members. */
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    /* lift motors */
    public DcMotor leftLift;
    public DcMotor rightLift;

    /* servos */
    public CRServo claw;

    /* Constants */

    public static final int MAX_LIFT_UP    = -1810 ; // Maximum upward position for arm
    public static final int MAX_LIFT_DOWN  = 0 ; // Minimum position of arm

    private static final double LIFT_SPEED = 2.50;

    /* local OpMode members. */
    private ElapsedTime period = new ElapsedTime();

    // MARK: Constructors

    // construct with just map and instructions
    public RobotHardware(HardwareMap ahwMap, InitInstructions i) {
        setRobotProperties(ahwMap, i);
    }

    // construct with just map and instructions and value for lift power
    public RobotHardware(HardwareMap ahwMap, InitInstructions i, int power) {
        setRobotProperties(ahwMap, i);
        liftPower = power;
    }

    /* Initialize standard Hardware interfaces */

    /**
     * Function that initializes the robot from a hardware map
     * @param hwMap Hardware map to use to initialize the hardware class
     */
    public void init(HardwareMap hwMap) {

        setRobotProperties(hwMap, new InitInstructions(true));

        // These might need to be changed
        frontLeft.setDirection  (DcMotor.Direction.REVERSE);
        frontRight.setDirection (DcMotor.Direction.FORWARD);
        backLeft.setDirection   (DcMotor.Direction.REVERSE);
        backRight.setDirection  (DcMotor.Direction.FORWARD);

        leftLift.setMode        (DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setMode       (DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setDirection  (DcMotorSimple.Direction.REVERSE);

    }

    // MARK: Methods

    // This simply sets the robot's stuff given certain instructions
    // sorry I know that's the least helpful comment ever written
    private void setRobotProperties(HardwareMap map, InitInstructions i) {
        frontLeft  = i.frontLeft  ? map.get(DcMotor.class, "front_left")  : null;
        frontRight = i.frontRight ? map.get(DcMotor.class, "front_right") : null;
        backLeft   = i.backLeft   ? map.get(DcMotor.class, "back_left")   : null;
        backRight  = i.backRight  ? map.get(DcMotor.class, "back_right")  : null;

        leftLift  = i.leftLift  ? map.get(DcMotor.class, "left_lift")  : null;
        rightLift = i.rightLift ? map.get(DcMotor.class, "right_lift") : null;

        claw = i.claw ? map.get(CRServo.class, "claw") : null;
    }

    // MARK: Gadget functions

    // lift

    public void liftPower(double power) {
        double dpos = (5 * power) * LIFT_SPEED;

        if (liftPosition + dpos > MAX_LIFT_UP) {
            liftPosition = MAX_LIFT_UP;
        } else if (liftPosition + dpos < MAX_LIFT_DOWN) {
            liftPosition = MAX_LIFT_DOWN;
        } else {
            liftPosition += dpos;
        }
    }

    public void updateLift() {
        leftLift.setTargetPosition((int) liftPosition);
        leftLift.setPower(liftPower);
        rightLift.setTargetPosition((int) liftPosition);
        rightLift.setPower(liftPower);
    }

    // Used for initialization instructions
    // we can initialize it with this so it knows what parts to initialize
    // so we don't run into a "lol didn't find it" error
    public static class InitInstructions {

        /* Public OpMode members. */
        public boolean frontLeft  = false;
        public boolean frontRight = false;
        public boolean backLeft   = false;
        public boolean backRight  = false;

        /* lift motors */
        public boolean leftLift  = false;
        public boolean rightLift = false;

        /* servos */
        public boolean claw = false;

        // default
        public InitInstructions() {}

        public InitInstructions(boolean all) {
            /* Public OpMode members. */
            frontLeft  = all;
            frontRight = all;
            backLeft   = all;
            backRight  = all;

            /* lift motors */
            leftLift  = all;
            rightLift = all;

            /* servos */
            claw = all;
        }

    }
//
//    /**
//     * Function that will move the robot
//     * @param angle to calculate angle and magnitude
//     * @param rotarySpeed to set the rotary speed for the wheels
//     */
//    public void moveBot(double angle, double speed, double rotarySpeed) {
//
//        double pi = Math.PI;
//
//        double [] wheelCoefficients = new double[4];
//
//        wheelCoefficients[0] = speed * Math.sin(Math.toRadians(angle + 135)) + rotarySpeed;
//        wheelCoefficients[1] = speed * Math.cos(Math.toRadians(angle + 135)) - rotarySpeed;
//        wheelCoefficients[2] = speed * Math.cos(Math.toRadians(angle + 135)) + rotarySpeed;
//        wheelCoefficients[3] = speed * Math.sin(Math.toRadians(angle + 135)) - rotarySpeed;
//
//        frontLeft.setPower(wheelCoefficients[0]);
//        frontRight.setPower(wheelCoefficients[1]);
//        backLeft.setPower(wheelCoefficients[2]);
//        backRight.setPower(wheelCoefficients[3]);
//
//    }
}

