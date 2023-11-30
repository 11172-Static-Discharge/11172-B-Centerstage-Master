package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.List;

public class CuttleLift {
    HardwareMap map;
    double clawLOpen = 0;
    double clawLClose = 1;

    double clawROpen = 1;
    double clawRClose = 0;

    boolean lBumperRecent = false;
    boolean rBumperRecent = false;

    double liftLGround = 0.92;
    double liftLHeight1 = 1;
    double liftLHeight2 = 1;

    double liftRGround = 0.07944;
    double liftRHeight1 = 1;
    double liftRHeight2 = 1;

    double increment = 0.02;

    double wristHoverGround = 0.3;
    double wristHeight1 = 1;
    double wristHeight2 = 1;

    double wristRHoverGround = 1;
    double wristRHeight1 = 1;
    double wristRHeight2 = 1;
    boolean recent = false;

    public Servo liftL;
    public Servo liftR;

    public Servo wrist;
    public ColorRangeSensor cL;
    public ColorRangeSensor cR;

    public Servo launcher;
    public double wristPos = 0.5;

    public Servo clawL;
    List<Servo> servos = new ArrayList<Servo>();
    public Servo clawR;
    public int servoIndex;
    public DcMotor slide;
    public double liftLPos = 0.5;
    public double liftRPos = 0.5;

    public String closeMode;

    Telemetry tele;
    public CuttleLift(HardwareMap map, Telemetry tele) {
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
        servoIndex = 0;

        launcher = map.servo.get("launcher");
        slide = map.dcMotor.get("slide");

        //cR = map.get(ColorRangeSensor.class, "cR");
        //cL = map.get(ColorRangeSensor.class, "cL");
        closeMode = "MANUAL";
    }

    public void useClaw(boolean leftOpen, boolean leftClose, boolean rightOpen, boolean rightClose) {
        if (leftOpen) clawL.setPosition(clawLOpen);
        if (leftClose) clawL.setPosition(clawLClose);
        if (rightOpen) clawR.setPosition(clawROpen);
        if (rightClose) clawR.setPosition(clawRClose);
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
            liftLPos = liftL.getPosition();
            liftRPos = liftR.getPosition();
        }
        if (down) {
            liftL.setPosition(liftLPos - increment);
            liftR.setPosition(liftRPos + increment);
            liftLPos = liftL.getPosition();
            liftRPos = liftR.getPosition();
        }

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
            wristPos = wrist.getPosition();
        }
        if (down) {
            wrist.setPosition(wristPos - increment);
            wristPos = wrist.getPosition();
        }

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

    public void calservo(boolean up, boolean down, boolean left, boolean right) {
        Servo servo = servos.get(servoIndex);
        if (left && servos.indexOf(servo)>=1) {
            servo = servos.get(servos.indexOf(servo)-1);
            servoIndex--;
        }
        if (right && servos.indexOf(servo)+1<servos.size()) {
            servo = servos.get(servos.indexOf(servo)+1);
            servoIndex++;
        }

        if (up) servo.setPosition(servo.getPosition()+0.05);
        if (down) servo.setPosition(servo.getPosition()-0.05);

        tele.addData("Current Servo: ", servo.getDeviceName());
        tele.addData("Current Servo Position: ", servo.getPosition());
        tele.update();
    }

}