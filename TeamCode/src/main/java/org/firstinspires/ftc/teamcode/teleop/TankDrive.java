package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TankDrive", group="OpMode")
public class TankDrive extends StrykeTeleOp {

    @Override
    public void postInit() {
        log("init Tank Drive!");

        listener1.setOnPressed(GamepadListener.Button.A, new Runnable() {
            @Override
            public void run() {
                log("Pressed A!!");
            }
        });
    }

    @Override
    public void onLoop() {
        getRobot().setDriveSpeed(scaleGamepadInput(gamepad1.left_stick_y, 1.0), -scaleGamepadInput(gamepad1.left_stick_y, 1.0));
    }

}
