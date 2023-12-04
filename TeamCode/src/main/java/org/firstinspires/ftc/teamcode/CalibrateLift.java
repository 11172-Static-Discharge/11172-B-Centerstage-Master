package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

public class CalibrateLift {
    HardwareMap map;
    public double clawLOpen = 1;
    public double clawLClose = 0;

    public double clawROpen = 0;
    public double clawRClose = 1;

    public boolean lBumperRecent, rBumperRecent, recent = false;

    public double liftLGround = 0.92;
    public double liftLHeight1 = 1;
    public double liftLHeight2 = 1;

    public double liftRGround = 0.07944;
    double liftRHeight1 = 1;
    double liftRHeight2 = 1;

    double increment = 0.02;

    double wristHoverGround = 0.3;
    double wristHeight1 = 1;
    double wristHeight2 = 1;

    double wristRHoverGround = 1;
    double wristRHeight1 = 1;
    double wristRHeight2 = 1;


    public Servo liftL, liftR, wrist, launcher, clawL, clawR;

    public ColorRangeSensor cL;
    public ColorRangeSensor cR;

    public double wristPos = 0.5;


    List<Servo> servos = new ArrayList<Servo>();
    public DcMotor slide;


    public String closeMode;

    public double clawLPos, clawRPos, liftLPos, liftRPos;

    Telemetry tele;
    public CalibrateLift(HardwareMap map, Telemetry tele) {
        this.map = map;
        this.tele = tele;

        liftL = map.servo.get("liftL");
        liftR = map.servo.get("liftR");

        wrist = map.servo.get("wrist");

        clawL = map.servo.get("clawL");
        clawR = map.servo.get("clawR");

        servos.add(liftL);
        servos.add(liftR);
        servos.add(wrist);
        servos.add(clawL);
        servos.add(clawR);

        launcher = map.servo.get("launcher");
        slide = map.dcMotor.get("slide");

        //cR = map.get(ColorRangeSensor.class, "cR");
        //cL = map.get(ColorRangeSensor.class, "cL");
        closeMode = "MANUAL";
    }

    public void useClaw(boolean leftClose, boolean leftOpen, boolean rightClose, boolean rightOpen) {
        if (leftClose) clawL.setPosition(clawLClose);
        if (leftOpen) clawL.setPosition(clawLOpen);
        if (rightClose) clawR.setPosition(clawRClose);
        if (rightOpen) clawR.setPosition(clawROpen);
    }



    /*
    public void autoClose(String autoClose) {
        if (autoClose.equals("AUTO")) {
            if (cL.getDistance(DistanceUnit.CM) < 1) clawL.setPosition(clawLClose);
            if (cR.getDistance(DistanceUnit.CM) < 1) clawR.setPosition(clawRClose);
        }
    }*/

    /*
    public void toggleAutoClose(boolean toggle) {
        if (toggle) {
            if(closeMode.equals("AUTO")) {
                closeMode = "MANUAL";
            }
            if(closeMode.equals("MANUAL")) {
                closeMode = "AUTO";
            }
        }
        autoClose(closeMode);
    }*/

    public void launch() {
        launcher.setPosition(0);
    }
    public void launch(boolean launch) {
        if (launch) {
            launcher.setPosition(0);
        }
        else {
            launcher.setPosition(1);
        }
    }



    public void setWristGround(boolean x) {
        if (x) wrist.setPosition(wristHoverGround);
    }

    public void calLift(boolean up, boolean down) {
        if(up) {
            liftL.setPosition(liftLPos + increment);
            liftR.setPosition(liftRPos - increment);
        }
        if (down) {
            liftL.setPosition(liftLPos - increment);
            liftR.setPosition(liftRPos + increment);
        }
        liftLPos = liftL.getPosition();
        liftRPos = liftR.getPosition();

    }

    public void resetLift(boolean dfs) {
        if (dfs) {
            liftL.setPosition(0.5);
            liftR.setPosition(0.5);
        }
    }

    public void calWrist(boolean up, boolean down) {

        if (up) {
            wrist.setPosition(wristPos + increment);
        }
        if (down) {
            wrist.setPosition(wristPos - increment);
        }
        wristPos = wrist.getPosition();

    }

    public void calClaw(boolean lClawL, boolean lClawR, boolean rClawL, boolean rClawR) {
        if (lClawL) clawL.setPosition(clawLPos - increment/2);
        if (lClawR) clawL.setPosition(clawLPos + increment/2);
        if (rClawL) clawR.setPosition(clawRPos + increment/2);
        if (rClawR) clawR.setPosition(clawRPos - increment/2);

        clawLPos = clawL.getPosition();
        clawRPos = clawR.getPosition();
    }
    public void setLiftPos(double pos) {

        //hover over ground
        if (pos == 0) {
            wrist.setPosition(wristHoverGround);

            liftL.setPosition(liftLGround);
            liftR.setPosition(liftRGround);


        }

        //Backdrop Height 1
        if (pos == 1) {
            liftL.setPosition(liftLHeight1);
            liftR.setPosition(liftRHeight1);

            wrist.setPosition(wristHeight1);
        }

        //Backdrop Height 2
        if (pos == 2) {
            liftL.setPosition(liftLHeight2);
            liftR.setPosition(liftRHeight2);

            wrist.setPosition(wristHeight2);

        }
    }

    public void telemetryLift() {
        tele.addData("wrist", wrist.getPosition());

        tele.addData("liftL", liftL.getPosition());
        tele.addData("liftR", liftR.getPosition());

        tele.addData("clawL", clawL.getPosition());
        tele.addData("clawR", clawR.getPosition());
        tele.update();
    }
}