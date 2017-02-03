package org.firstinspires.ftc.teamcode.stryke.teleop;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.stryke.GamepadListener;

@TeleOp(name = "Simple OpMode")
public class SimpleOpMode extends StrykeOpMode {

    @Override
    public void runOpMode(){
        initHardware();

        GamepadListener gp2 = new GamepadListener(gamepad1);
        gp2.setOnPressed(GamepadListener.Button.Y, new Runnable() {
            @Override
            public void run() {
                if(!shooterReady) return;
                new Thread(new Runnable() {
                    @Override
                    public void run() {
                        try {
                            shooterReady = false;
                            shootBall();
                            shooterReady = true;
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                    }
                }).start();
            }
        });

        waitForStart();
        telemetry.addData("Status", "Running, Good Luck!");


        while (opModeIsActive()) {
            gp2.update(gamepad1);

            if(gamepad1.left_trigger > 0.1)
                manipulator.setPower(-gamepad1.left_trigger);
            else if (gamepad1.right_trigger > 0.1)
                manipulator.setPower(gamepad1.right_trigger);

            if(gamepad1.dpad_left)
                gate.setPosition(GATE_DOWN);
            else gate.setPosition(GATE_UP);

            if(gamepad1.a) releaseBallHugger();
            if(gamepad1.x) holdBallHugger();

            idle();
        }


    }
}
