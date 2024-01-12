package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class BetterBoolGamepad {
    Gamepad gamepad;

    FirstBoolean du, dd, dl, dr, a, b, x, y, lb, rb;
    public BetterBoolGamepad(Gamepad gamepad) {
        this.gamepad = gamepad;
        du = new FirstBoolean();
        dd = new FirstBoolean();
        dl = new FirstBoolean();
        dr = new FirstBoolean();
        a = new FirstBoolean();
        b = new FirstBoolean();
        x = new FirstBoolean();
        y = new FirstBoolean();
        lb = new FirstBoolean();
        rb = new FirstBoolean();
    }
    public boolean dpad_up() {return du.betterboolean(gamepad.dpad_up);}
    public boolean dpad_down() {return dd.betterboolean(gamepad.dpad_down);}
    public boolean dpad_left() {return dl.betterboolean(gamepad.dpad_left);}
    public boolean dpad_right() {return dr.betterboolean(gamepad.dpad_right);}
    public boolean a() {return a.betterboolean(gamepad.a);}
    public boolean b() {return b.betterboolean(gamepad.b);}
    public boolean x() {return x.betterboolean(gamepad.x);}
    public boolean y() {return y.betterboolean(gamepad.y);}
    public boolean left_bumper(){return lb.betterboolean(gamepad.left_bumper);}
    public boolean right_bumper() {return rb.betterboolean(gamepad.right_bumper);}

}
