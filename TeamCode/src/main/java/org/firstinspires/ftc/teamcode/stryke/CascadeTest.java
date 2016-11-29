package org.firstinspires.ftc.teamcode.stryke;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp(name = "Lift Test", group = "Linear Opmode")
public class CascadeTest extends LinearOpMode {

    public DcMotor one, two;

    @Override
    public void runOpMode() throws InterruptedException{

        one = hardwareMap.dcMotor.get("one");
        two = hardwareMap.dcMotor.get("two");

        waitForStart();

        while(opModeIsActive()) {

            if (gamepad1.a){
                one.setPower(1);
                two.setPower(1);
            } else if (gamepad1.b) {
                one.setPower(-1);
                two.setPower(-1);
            } else {
                one.setPower(0);
                two.setPower(0);
            }

        }

    }

}
