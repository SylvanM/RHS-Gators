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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// GAMEPAD 1 controls MOTION of robot
// GAMEPAD 2 controls ACTIONS of robot (moving the arm, stuff like that)

@TeleOp(name="Pushbot: Teleop Tank", group="Pushbot")
//@Disabled
public class ChrisDrive extends OpMode {

    /* Declare OpMode members. */
    private ChrisHardware robot         = new ChrisHardware(); // use the class created to define a Pushbot's hardware
                                                               // could also use HardwarePushbotMatrix class.
    private double          clawOffset  = 0.0 ;                // Servo mid position
    private final double    CLAW_SPEED  = 0.02 ;               // sets rate to move servo

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // hey
        // Code runs on init-button prassed

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        // Code runs in a loop after init button pressed
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        // Run on start
        telemetry.addLine("Driver in control!");
        telemetry.addData("Left input degrees:", "");
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Actual control

        // Code for robot control goes here

        // turning takes precedence over movement
        // i.e. if you are moving the bot around but then suddenly start moving the right thumb-stick to turn, the bot will stop moving to turn
        robot.headTargetDirection(getLeftStickDegrees(), gamepad1.right_trigger);
        // Send telemetry message to signify robot running
        telemetry.addData("Left stick input degrees:", getLeftStickDegrees());
        telemetry.addData("Right trigger activation:", gamepad1.right_trigger);

        // Claw control

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

        // Code runs on stop

    }

    // methods

    double getLeftStickDegrees() {
        //must invert inputs
        double x = -gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;

        double referenceAngle = Math.abs(Math.atan(y / x) * (180 / 3.14159265358979));
        double angle = 0;

        if (x < 0 && y > 0) {
            angle = 180 - referenceAngle;
        } else if (x > 0 && y < 0) {
            angle = 360 - referenceAngle;
        } else if (x < 0 && y < 0) {
            angle = 180 + referenceAngle;
        }

        telemetry.addData("Reference Angle:", referenceAngle);

        if (x == 0 && y == 0) {
            return -1;
        }

        telemetry.addData("Left stick degrees:", angle);

        return angle;
    }
}