package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="CrabDrive", group="OpMode")
public class CrabDrive extends StrykeTeleOp {

    boolean xDrive = true;

    @Override
    public void postInit() {
        listener1.setOnPressed(GamepadListener.Button.A, new Runnable() {
            @Override
            public void run() {
                xDrive = !xDrive;
            }
        });
    }

    @Override
    public void onLoop() {
        double y = scaleGamepadInput(gamepad1.left_stick_y, -1.0);
        double x = scaleGamepadInput(gamepad1.left_stick_x, 1.0);
        double rotation = (-scaleGamepadInput(gamepad1.left_trigger, 1) + scaleGamepadInput(gamepad1.right_trigger, 1)) / 2;

        if (xDrive) {
            getRobot().getLeftDriveFront().setPower(Range.clip(y + x + rotation, -1,1));
            getRobot().getRightDriveFront().setPower(Range.clip(-y + x + rotation, -1,1));
            getRobot().getRightDriveBack().setPower(Range.clip(y + -x + rotation, -1,1));
            getRobot().getLeftDriveBack().setPower(Range.clip(-y + -x + rotation, -1,1));
        } else {
            getRobot().getLeftDriveFront().setPower(Range.clip(y + rotation, -1,1));
            getRobot().getRightDriveFront().setPower(Range.clip(x + rotation, -1,1));
            getRobot().getRightDriveBack().setPower(Range.clip(-y + rotation, -1,1));
            getRobot().getLeftDriveBack().setPower(Range.clip(-x + rotation, -1,1));
        }
    }
}