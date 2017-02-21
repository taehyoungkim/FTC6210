package org.firstinspires.ftc.teamcode.stryke.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

@Autonomous(name = "Gyro Test")
public class GyroTest extends LinearOpMode {
    PhoneGyro phoneGyro;
    @Override
    public void runOpMode(){
        phoneGyro = new PhoneGyro(FtcRobotControllerActivity.context);
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("Heading", phoneGyro.getHeading());
            telemetry.update();
        }
    }
}
