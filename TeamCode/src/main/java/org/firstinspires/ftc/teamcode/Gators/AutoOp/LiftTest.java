package org.firstinspires.ftc.teamcode.Gators.AutoOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

// hardware class
import org.firstinspires.ftc.teamcode.Gators.Hardware.*;

@Autonomous(name="Lift Test", group="AutoOp")
//@Disabled
public abstract class LiftTest extends LinearOpMode {

    private RobotHardware robot;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize class stuff

        RobotHardware.InitInstructions instructions = new RobotHardware.InitInstructions();
        instructions.leftLift = true;

        robot = new RobotHardware(hardwareMap, instructions);

        waitForStart(); // waits for play button to be hit

        robot.leftLift.setPower(0.1);

        telemetry.addData("Lift position:", robot.leftLift.getCurrentPosition());

    }

}