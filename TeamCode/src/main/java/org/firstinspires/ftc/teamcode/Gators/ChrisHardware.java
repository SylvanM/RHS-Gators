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
import com.qualcomm.robotcore.hardware.HardwareMap;
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
    public DcMotor frontPortDrive   = null;
    public DcMotor frontStarboardDrive  = null;
    public DcMotor backPortDrive  = null;
    public DcMotor backStarboardDrive = null;

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
        frontPortDrive       = hwMap.get(DcMotor.class, "front_port");
        frontStarboardDrive  = hwMap.get(DcMotor.class, "front_starboard");
        backPortDrive        = hwMap.get(DcMotor.class, "back_port");
        backStarboardDrive   = hwMap.get(DcMotor.class, "back_starboard");

        // These might need to be changed
        frontPortDrive.setDirection(DcMotor.Direction.FORWARD);
        frontStarboardDrive.setDirection(DcMotor.Direction.FORWARD);
        backPortDrive.setDirection(DcMotor.Direction.FORWARD);
        backStarboardDrive.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        frontPortDrive.setPower(0);
        frontStarboardDrive.setPower(0);
        backPortDrive.setPower(0);
        backStarboardDrive.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        frontPortDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontStarboardDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backPortDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backStarboardDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    void closeClaw() {

    }

    void openClaw() {

    }

    void turnWithSpeed(double speed) {
        frontPortDrive.setPower(speed);
        backPortDrive.setPower(speed);

        frontStarboardDrive.setPower(-speed);
        backStarboardDrive.setPower(-speed);
    }

    void headTargetDirection(double degree, double speed) {
        double port = (Math.abs(-degree / 45.0)-2) * speed;
        double starboard = -(Math.abs(-degree / 45.0)-2) * speed;

        // front wheels
        frontPortDrive.setPower(port);
        frontStarboardDrive.setPower(starboard);

        // back wheels
        backPortDrive.setPower(port);
        backStarboardDrive.setPower(starboard);
    }

}

