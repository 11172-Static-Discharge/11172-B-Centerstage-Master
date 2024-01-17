package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

public class ServoCalibrator {
    public double pos, increment, startPos;
    Servo servo;
    public ServoCalibrator(Servo servo, double increment, double startPos) {
        this.servo = servo;
        this.increment = increment;
    }

    public void startCal() {
        servo.setPosition(startPos);
    }

    public void cal(boolean up, boolean down) {
        if (up) servo.setPosition(pos + increment);
        if (down) servo.setPosition(pos - increment);
        pos = servo.getPosition();
    }
}
