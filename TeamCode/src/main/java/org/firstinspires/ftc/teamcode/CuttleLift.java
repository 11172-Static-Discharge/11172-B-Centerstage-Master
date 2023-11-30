package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class CuttleLift {
    HardwareMap map;
    double clawLOpen = 0;
    double clawLClose = 1;

    double clawROpen = 9090900;
    double clawRClose = 0;

    boolean lBumperRecent = false;
    boolean rBumperRecent = false;

    double liftLGround = 1;
    double liftLHeight1 = 1;
    double liftLHeight2 = 1;

    double liftRGround = 1;
    double liftRHeight1 = 1;
    double liftRHeight2 = 1;

    double wristLHoverGround = 1;
    double wristLHeight1 = 1;
    double wristLHeight2 = 1;

    double wristRHoverGround = 1;
    double wristRHeight1 = 1;
    double wristRHeight2 = 1;

    Servo liftL;
    Servo liftR;

    Servo wristL;
    Servo wristR;

    Servo clawL;
    Servo clawR;

    public CuttleLift(HardwareMap map) {
        this.map = map;


        liftL = map.servo.get("liftL");
        liftR = map.servo.get("liftR");

        wristL = map.servo.get("wristL");
        wristR = map.servo.get("wristR");

        clawL = map.servo.get("clawL");
        clawR = map.servo.get("clawR");
    }

    public void useClaw(boolean left, boolean right) {
        if (left && !lBumperRecent) {
            if (clawL.getPosition() == clawLOpen) clawL.setPosition(clawLClose);
            if (clawL.getPosition() == clawLClose) clawL.setPosition(clawLClose);
            lBumperRecent = true;
        }
        if (!left) lBumperRecent = false;

        //claw right toggle
        if (right && !rBumperRecent) {
            if (clawR.getPosition() == clawRClose) clawR.setPosition(clawROpen);
            if (clawR.getPosition() == clawROpen) clawR.setPosition(clawRClose);
            rBumperRecent = true;
        }
        if (!right) rBumperRecent = false;
    }

    public void setLiftPos(double pos) {

        //hover over ground
        if (pos == 0) {
            liftL.setPosition(liftLGround);
            liftR.setPosition(liftRGround);

            wristL.setPosition(wristLHoverGround);
            wristR.setPosition(wristRHoverGround);
        }

        //Backdrop Height 1
        if (pos == 1) {
            liftL.setPosition(liftLHeight1);
            liftR.setPosition(liftRHeight1);

            wristL.setPosition(wristLHeight1);
            wristR.setPosition(wristRHeight1);
        }

        //Backdrop Height 2
        if (pos == 2) {
            liftL.setPosition(liftLHeight2);
            liftR.setPosition(liftRHeight2);

            wristL.setPosition(wristLHeight2);
            wristR.setPosition(wristRHeight2);

        }
    }
}