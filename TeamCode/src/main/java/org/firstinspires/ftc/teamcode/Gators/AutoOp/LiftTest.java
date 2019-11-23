package org.firstinspires.ftc.teamcode.Gators.AutoOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

// hardware class
import org.firstinspires.ftc.teamcode.Gators.Hardware.*;

@Autonomous(name="Lift Test", group="AutoOp")
//@Disabled
public class LiftTest extends OpMode {

    private RobotHardware robot;
    private ElapsedTime runtime = new ElapsedTime();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // make instructions for initialization

        RobotHardware.InitInstructions instructions = new RobotHardware.InitInstructions();

        instructions.leftLift = true;

        robot = new RobotHardware(hardwareMap, instructions);
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

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        // okay! here is where the autonomous is coded!

        /*
         * TELEMETRY
         */

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

        // Code runs on stop

    }

}