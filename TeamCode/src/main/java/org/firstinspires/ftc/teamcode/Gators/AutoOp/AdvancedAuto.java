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

package org.firstinspires.ftc.teamcode.Gators.AutoOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

// hardware class
import org.firstinspires.ftc.teamcode.Gators.Hardware.*;

import java.util.concurrent.TimeUnit;


/*
 * NOT A DRIVER MODE
 * This is the Robot's automonous driver mode
 */

@Autonomous(name="Advanced Autonomous", group="AutoOp")
//@Disabled
public class AdvancedAuto extends OpMode {

    /* Declare OpMode members. */
    private RobotHardware robot         = new RobotHardware(); // use the class created to define a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    private double          clawOffset  = 0.0 ;                  // Servo mid position
    private final double    CLAW_SPEED  = 0.02 ;                 // sets rate to move servo
    private final double    DESIRED_COLOR[] = new double[] {0, 0, 0}; // RGB Values of gold mineral
    private int mineralsTested = 0;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Code runs on init-button pressed

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "AutoOp initiating");
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
        runtime.startTime();
        robot.lowerDistanceSensor();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        // TODO: detach from lander

        // begin testing minerals

        while (robot.getDistance() > robot.WIDTH_OF_ROBOT) { // while there is no object directly in front of the robot
            robot.move(.1, 0, 0); // move forward
        }
        robot.stop();

        while (mineralsTested < 3) {
            while (robot.getDistance() > robot.COLOR_SENSOR_RANGE) {
                robot.move(.1,0,90);
            }
            robot.stop();

            double detectedColors[] = new double[] {robot.colorSensor.red(), robot.colorSensor.blue(), robot.colorSensor.green()};
            mineralsTested++;
            if (arrayIsClose(detectedColors, DESIRED_COLOR, 10)) {
                // knock off that ball
                moveBot(.5,0,90,1);
            } else {
                robot.raiseDistanceSensor();
                moveBot(1, 0, 90, 1);
                robot.lowerDistanceSensor();
            }
        }

        // mineral thing is done! Woot woot

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

        // Code runs on stop


        robot.raiseDistanceSensor();

    }

    private void moveBot(double linearSpeed, double rotarySpeed, double angle, double forSeconds) {
        sleep(forSeconds);
        robot.stop();
    }

    private void sleep(double seconds) {
        long millis = (long)seconds * 1000;
        try {
            wait(millis);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    // for color sensing
    boolean doubleIsClose(double testedDouble, double target, double tolerance) {
        if (Math.abs(testedDouble - target) < Math.abs(tolerance)) {
            return true;
        } else {
            return false;
        }
    }

    private boolean arrayIsClose(double testedArray[], double targetArray[], double tolerance) {
        if (testedArray.length != targetArray.length) {
            return false;
        }

        for (int index = 0; index < targetArray.length; index++) {
            if (!doubleIsClose(testedArray[index], targetArray[index], tolerance)) {
                return false;
            }
        }

        return true;
    }

}