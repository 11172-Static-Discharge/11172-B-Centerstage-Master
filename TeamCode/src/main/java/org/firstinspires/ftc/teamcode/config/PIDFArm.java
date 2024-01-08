package org.firstinspires.ftc.teamcode.config;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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

    public void reset()
    {
        l.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        l.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        r.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveTo(int targetPos) {
        controller.setPID(p, i, d);
        int lPos = l.getCurrentPosition();

        int tolerance = 10;

        // Check if the current position is within the tolerance of the target position
        if (Math.abs(lPos - targetPos) <= tolerance) {
            // Within tolerance, stop the motors and exit the method
            l.setPower(0);
            r.setPower(0);
            tele.addData("Reached target position", lPos);
            tele.update();
            return;
        }

        double lpid = controller.calculate(lPos, targetPos);
        double lff = Math.cos(Math.toRadians(targetPos / ticksInDegree)) * f;

        double power = lpid + lff;

        l.setPower(power * 0.25);
        r.setPower(-power * 0.25);
        tele.addData("lPos: ", lPos);
        tele.addData("lTarget: ", targetPos);
        tele.update();
    }

    public void moveToPower(int targetPos, double pow) {
        controller.setPID(p, i, d);
        int lPos = l.getCurrentPosition();

        int tolerance = 10;

        // Check if the current position is within the tolerance of the target position
        if (Math.abs(lPos - targetPos) <= tolerance) {
            // Within tolerance, stop the motors and exit the method
            l.setPower(0);
            r.setPower(0);
            tele.addData("Reached target position", lPos);
            tele.update();
            return;
        }

        double lpid = controller.calculate(lPos, targetPos);
        double lff = Math.cos(Math.toRadians(targetPos / ticksInDegree)) * f;

        double power = lpid + lff;

        l.setPower(power * 0.25 * pow);
        r.setPower(-power * 0.25 * pow);
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
