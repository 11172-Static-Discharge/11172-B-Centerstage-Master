package org.firstinspires.ftc.teamcode;

public class FirstBoolean {
    boolean y = false;
    boolean z = false;
    public boolean betterboolean(boolean x) {
        if (!y)  z = x;
        else z =  false;
        y = x;
        return z;
    }
    public void reset() {y = false; z = false;}
}
