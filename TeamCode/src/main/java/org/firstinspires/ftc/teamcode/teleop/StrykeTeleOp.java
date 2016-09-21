package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public abstract class StrykeTeleOp extends OpMode {

    protected RobotHardware robot;
    public GamepadListener listener1, listener2;


    @Override
    public final void init() {
        robot = new RobotHardware(hardwareMap);
        robot.initHardware();
        listener1 = new GamepadListener(gamepad1);
        listener2 = new GamepadListener(gamepad2);

        postInit();
    }

    @Override
    public final void loop(){
        listener1.update(gamepad1);
        listener2.update(gamepad2);
        onLoop();
    }
    public abstract void postInit();
    public abstract void onLoop();

    public RobotHardware getRobot() {
        return robot;
    }

    public void log(String message) {
        log(this.getClass().getSimpleName(), message);
    }

    public void log(String prefix, String message) {
        DbgLog.msg(prefix, message);
    }

    private static double[] values =
            { 0.00, 0.05, 0.09, 0.10, 0.12
                    , 0.15, 0.18, 0.24, 0.30, 0.36
                    , 0.43, 0.50, 0.60, 0.72, 0.85
                    , 1.00, 1.00
            };

    public static double scaleGamepadInput(double power, double scale){
        // Scale gamepad joystick movement in a nonlinear fashion
        if(Math.abs(power) <= 0.05)// Make sure joystick is actually being moved
            return 0;
        // clamp value between -1 and 1, the min and max values for joystick movement
        power = Range.clip(power, -1, 1);
        int index = (int) Range.clip((int) (Math.abs(power) * values.length-1), 0, values.length-1); // Clamp index between 0 and 16
        return power < 0 ? -values[index] * scale : values[index] * scale; // Return negative value if power is < 0
    }
}
