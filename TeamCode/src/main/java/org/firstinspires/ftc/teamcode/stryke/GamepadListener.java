package org.firstinspires.ftc.teamcode.stryke;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadListener {

    private boolean[] buttons;
    private Runnable[] pressed;
    private Runnable[] released;
    private Runnable[] hold;

    public GamepadListener(Gamepad pad) {
        buttons = getButtons(pad);
        pressed = new Runnable[buttons.length];
        released = new Runnable[buttons.length];
        hold = new Runnable[buttons.length];
    }

    public void update(Gamepad gamepad){
        boolean[] updated = getButtons(gamepad);
        for (int i = 0; i < buttons.length; i ++){
            if(buttons[i] != updated[i]){ // Changed Value
                if(updated[i]){ // Currently Pressed
                    if(pressed[i] != null) pressed[i].run();
                } else {
                    if(released[i] != null) released[i].run();
                }
                buttons[i] = updated[i]; // Update Stored Values
            } else if (buttons[i] && hold[i] != null) hold[i].run();
        }
    }

    public void setOnPressed(Button button, Runnable onPressed) {
        pressed[button.value] = onPressed;
    }

    public void setOnReleased(Button button, Runnable onReleased) {
        released[button.value] = onReleased;
    }

    public void setOnHold(Button button, Runnable onHold) {
        hold[button.value] = onHold;
    }


    private boolean[] getButtons(Gamepad pad) {
        boolean[] out = new boolean[Button.values().length];
        out[Button.A.value] = pad.a;
        out[Button.B.value] = pad.b;
        out[Button.X.value] = pad.x;
        out[Button.Y.value] = pad.y;
        out[Button.LEFT_BUMPER.value] = pad.left_bumper;
        out[Button.RIGHT_BUMPER.value] = pad.right_bumper;
        out[Button.DPAD_UP.value] = pad.dpad_up;
        out[Button.DPAD_DOWN.value] = pad.dpad_down;
        out[Button.DPAD_LEFT.value] = pad.dpad_left;
        out[Button.DPAD_RIGHT.value] = pad.dpad_right;
        return out;
    }


    public enum Button {

        A(0), B(1), X(2), Y(3), LEFT_BUMPER(4), RIGHT_BUMPER(5), DPAD_UP(6), DPAD_DOWN(7), DPAD_LEFT(8), DPAD_RIGHT(9);

        private int value;
        Button(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }

    }

}
