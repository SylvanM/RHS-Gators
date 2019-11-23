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

package org.firstinspires.ftc.teamcode.Gators.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Gators.Hardware.*;

import java.util.concurrent.TimeUnit;

// GAMEPAD 1 controls MOTION of robot
// GAMEPAD 2 controls ACTIONS of robot (moving the arm, stuff like that)

@TeleOp(name="Human control", group="Omnidirectional Control")
//@Disabled
public class HumanControl extends OpMode {

    /* Declare OpMode members. */
    private RobotHardware robot         = new RobotHardware(); // use the class created to define a Pushbot's hardware
                                                               // could also use HardwarePushbotMatrix class.

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // hey
        // Code runs on init-button pressed

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */

        robot.init(hardwareMap);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        // Run on start
        telemetry.addLine("Driver in control, inputs now will have effect on robot.");
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // might have to remove this
        telemetry.clear();

        int i;

        /*
         * This part of the loop function controls the actual motion of the robot
         *
         * TO CONTROL:
         * Both sides of the robot are controlled by their respective thumb sticks
         * eg. to go forward, push both thumb sticks forward. To rotate right, push the left stick forward and pull the right one backward.
         *
         * To move side to side: Press the respective bumper buttons on the controller in relation to where you want to go
         */

        /*
         * WHEEL MOVEMENT
         */

        // math for calculating standard control

        double pi = Math.PI;

        double wheelCoefficients[] = new double[4];

        double x = (double) -gamepad1.left_stick_x;
        double y = (double) gamepad1.left_stick_y;

        double angle = getAngle(x, y);
        double linearSpeed = getMagnitude(x, y);
        double rotarySpeed = (double) gamepad1.right_stick_x;

        wheelCoefficients[0] = linearSpeed * Math.sin(Math.toRadians(angle + 135)) + rotarySpeed;
        wheelCoefficients[1] = linearSpeed * Math.cos(Math.toRadians(angle + 135)) - rotarySpeed;
        wheelCoefficients[2] = linearSpeed * Math.cos(Math.toRadians(angle + 135)) + rotarySpeed;
        wheelCoefficients[3] = linearSpeed * Math.sin(Math.toRadians(angle + 135)) - rotarySpeed;


        // finer control (for more advanced drivers)
        // now set each wheel to what the controller tells it

        if (gamepad1.left_bumper) {
            wheelCoefficients[2] = 1;
            wheelCoefficients[3] = -1;
        } else if (gamepad1.right_bumper) {
            wheelCoefficients[2] = 1;
            wheelCoefficients[3] = -1;
        }

        else if (gamepad1.left_trigger > 0) {
            wheelCoefficients[0] = 1;
            wheelCoefficients[1] = -1;
        } else if (gamepad1.right_trigger > 0) {
            wheelCoefficients[0] = -1;
            wheelCoefficients[1] = 1;
        }


        // set all motors to corresponding coefficients
//        robot.frontLeft.setPower  (wheelCoefficients[0]);
//        robot.frontRight.setPower (wheelCoefficients[1]);
//        robot.backLeft.setPower   (wheelCoefficients[2]);
//        robot.backRight.setPower  (wheelCoefficients[3]);

        /*
         * MOVEMENT TELEMETRY
         */

        // show power to wheels
        for ( i = 0; i < 4; ++i ) {
            // shows power of wheel i
            telemetry.addData(Integer.toString(i + 1), wheelCoefficients[i]);
        }

//        telemetry.addData("Position: ", Double.toString(robot.position().x) + ", " + Double.toString(robot.position().y));


        /*
         * GADGET CONTROL
         */

        // suckers

        //if (gamepad2.right_bumper) robot.suck();
        //if (gamepad2.left_bumper)  robot.spit();

        //if ( !(gamepad2.left_bumper || gamepad2.right_bumper) ) robot.stopSucking();

        // lifters

        double liftPower = gamepad2.right_stick_y;

        robot.liftPower(liftPower);

        if (gamepad2.y) robot.liftPosition = RobotHardware.MAX_LIFT_UP;
        if (gamepad2.x) robot.liftPosition = RobotHardware.MAX_LIFT_DOWN;

        robot.updateLift();

        /*
         * OTHER TELEMETRY
         */

//        telemetry.addData("a: ", robot.imu.getAcceleration());
//        telemetry.addData("v:" , robot.imu.getVelocity());

        telemetry.addData("left lift pos:", robot.leftLift.getCurrentPosition());
        telemetry.addData("right lift pos:", robot.leftLift.getCurrentPosition());
        telemetry.addData("target pos:", robot.leftLift.getTargetPosition());
        telemetry.addData("input Targ pos:", liftPower);





    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

        // Code runs on stop

        robot.imu.stopAccelerationIntegration();

    }

    // methods

    private double getAngle(double x, double y) {
        double angle = Math.toDegrees(Math.atan2(y, x));
        telemetry.addData("x:", x);
        telemetry.addData("y:", y);

        if (angle < 0) {
            angle += 360;
        } else if (angle > 360) {
            angle -= 360;
        }

        return angle;
    }

    private double getMagnitude(double a, double b) {
        float exponent = 2;
        float aSquared = (float) Math.pow(a, exponent);
        float bSquared = (float) Math.pow(b, exponent);
        double sum = (double)aSquared + (double)bSquared;

        return Math.sqrt(sum);
    }


}