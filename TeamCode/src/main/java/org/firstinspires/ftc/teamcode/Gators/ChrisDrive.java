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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

// GAMEPAD 1 controls MOTION of robot
// GAMEPAD 2 controls ACTIONS of robot (moving the arm, stuff like that)

@TeleOp(name="Chris Drive", group="Tank Control")
//@Disabled
public class ChrisDrive extends OpMode {

    /* Declare OpMode members. */
    private ChrisHardware robot         = new ChrisHardware(); // use the class created to define a Pushbot's hardware
                                                               // could also use HardwarePushbotMatrix class.
    private double          clawOffset  = 0.0;                // Servo mid position
    private final double    CLAW_SPEED  = 0.02;

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
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        /*
         * This part of the loop function controls the actual motion of the robot
         *
         * TO CONTROL:
         * Both sides of the robot are controlled by their respective thumb sticks
         * eg. to go forward, push both thumb sticks forward. To rotate right, push the left stick forward and pull the right one backward.
         *
         * To move side to side: Press the respective bumper buttons on the controller in relation to where you want to go
         */

        // move left and right
        if (gamepad1.right_bumper) { robot.headSideways(ChrisHardware.Direction.right, 1); }
        if (gamepad1.left_bumper)  { robot.headSideways(ChrisHardware.Direction.left, 1); }

        // control separate sides (tank drive)
        robot.setMotorGroup(ChrisHardware.MotorGroup.port, -gamepad1.left_stick_y);
        robot.setMotorGroup(ChrisHardware.MotorGroup.starboard, gamepad1.right_stick_y);

        /*
         * ARM CONTROL
         */

        double armRotationTorque    =    gamepad2.right_stick_x;
        double armBaseRaisingTorque =    gamepad2.right_stick_y;
        double armExtensionPower    =    gamepad2.left_stick_y;

        // robot.rotateArm(armRotationTorque);
        robot.raiseArmBase(armBaseRaisingTorque);
        robot.extendArm(armExtensionPower);

        /*
         * Claw Control
         */

        if (gamepad2.right_bumper) {  robot.openClaw();  }
        if (gamepad2.left_bumper)  {  robot.closeClaw(); }

        /*
         * TELEMETRY
         */

        // show motor group power
        telemetry.addData("port motors power:", gamepad1.left_stick_y);
        telemetry.addData("starboard motors power:", gamepad1.right_stick_y);
        telemetry.addData("left claw:", robot.leftClawServo.getPosition());

        // show sideways movement info
        if (gamepad1.right_bumper || gamepad1.left_bumper) {
            String sidewaysDirection = gamepad1.right_bumper ? "right" : "left";
            telemetry.addData("sideways direction", sidewaysDirection);
        } else {
            telemetry.addData("sideways direction", "null");
        }

        // show arm data
        double armRotationPower = robot.armRaisingMotor.getPower();
        double armRaisingPower  = 0.0; // robot.armBaseMotor.getPower();

        telemetry.addData("arm rotation:", armRotationPower);
        telemetry.addData("arm raising force:", armRaisingPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

        // Code runs on stop

    }

    // methods


}