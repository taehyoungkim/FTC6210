package org.firstinspires.ftc.teamcode.autonomous;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "TestAutonomous", group="Linear OpMode")
public class TestAutonomous extends StrykeAuto {

    public TestAutonomous() {
        super(4);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        getRobot().initHardware();
        log("init!");

        waitForStart();
        log("run!");

        // Drive forward 12 inches, turn, drive forward, then turn again
        driveDistance(12, 0.5, 1000 * 5);
        simpleWaitS(1);
        encoderTurn(180, 0.5, 1000 * 5);
        simpleWaitS(1);
        driveDistance(12, 0.5, 1000 * 1);
        simpleWaitS(1);
        encoderTurn(180,0.5);

    }

}
