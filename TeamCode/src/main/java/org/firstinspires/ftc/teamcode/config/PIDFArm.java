package org.firstinspires.ftc.teamcode.config;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.time.chrono.MinguoChronology;

public class PIDFArm {

    public PIDController controller;
    public static double p,i,d, f;
    public DcMotorEx l,r;

    public final double ticksInDegree = 4005.558676 / 360.0;
    public Telemetry tele;

    public static int targetL, targetR;
    public int increment;
    public int curLPos;
    public PIDFArm(DcMotorEx l, DcMotorEx r, Telemetry tele, int increment) {
        this.l=l;
        this.r=r;
        this.p=0.005;
        this.i=0;
        this.d=0.001;
        this.f=-0.06;
        this.increment = increment;
        this.tele = tele;
        targetL=0;
        targetR=0;
        controller = new PIDController(p, i, d);
    }

    public void moveTo(int targetPos) {
        controller.setPID(p, i, d);
        int lPos = l.getCurrentPosition();
        double lpid = controller.calculate(lPos, targetPos);
        double lff = Math.cos(Math.toRadians(targetPos / ticksInDegree)) * f;

        double power = lpid + lff;

        l.setPower(power);
        r.setPower(-power);
        tele.addData("lPos: ", lPos);
        tele.addData("lTarget: ", targetPos);
        tele.update();
    }

    public void calibratePos(boolean up, boolean down) {
        curLPos = l.getCurrentPosition();
        if(up) moveTo(curLPos+increment);
        if(down) moveTo(curLPos-increment);
    }


}
