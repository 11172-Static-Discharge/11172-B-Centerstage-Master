package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

    public DcMotorEx liftLM, liftRM;


    double increment = 0.02;



    public Servo wrist, clawL, clawR;


    public double wristPos = 0.5;
    public int motorIncrement = 3;

    public double clawLPos, clawRPos;

    Telemetry tele;
    public CalibrateLift(HardwareMap map, Telemetry tele) {
        this.map = map;
        this.tele = tele;

        //liftL = map.servo.get("liftL");
        //liftR = map.servo.get("liftR");

        wrist = map.servo.get("wrist");

        clawL = map.servo.get("clawL");
        clawR = map.servo.get("clawR");


        liftLM = map.get(DcMotorEx.class, "liftL");
        liftRM = map.get(DcMotorEx.class, "liftR");

        liftLM.setTargetPositionTolerance(1);
        liftRM.setTargetPositionTolerance(1);

    }

    public void useClaw(boolean leftClose, boolean leftOpen, boolean rightClose, boolean rightOpen) {
        if (leftClose) clawL.setPosition(clawLClose);
        if (leftOpen) clawL.setPosition(clawLOpen);
        if (rightClose) clawR.setPosition(clawRClose);
        if (rightOpen) clawR.setPosition(clawROpen);
    }

    public void calMotorLift(boolean up, boolean down) {

        if(up) {
            liftLM.setTargetPosition(liftLM.getCurrentPosition() + motorIncrement);
            liftRM.setTargetPosition(liftRM.getCurrentPosition() - motorIncrement);

            liftLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if (down) {
            liftLM.setTargetPosition(liftLM.getCurrentPosition() - motorIncrement);
            liftRM.setTargetPosition(liftRM.getCurrentPosition() + motorIncrement);

            liftLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        //liftLMPos = liftLM.getCurrentPosition();
        //liftRMPos = liftRM.getCurrentPosition();
    }

    /*public void calLift(boolean up, boolean down) {
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

    }*/


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
}