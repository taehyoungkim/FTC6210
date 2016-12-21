package org.firstinspires.ftc.teamcode.stryke.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Busch_902818 on 12/12/2016.
 */
@TeleOp(name = "shooter test")
public class ShooterTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor s = hardwareMap.dcMotor.get("s");

        waitForStart();
        while(opModeIsActive()) {
            double val = gamepad1.left_stick_y;
            if (Math.abs(val) < 0.1) val = 0;
            s.setPower(val);
            telemetry.addData("Powa", val* 100 + "%");
            telemetry.update();

            idle();
        }
    }
}
