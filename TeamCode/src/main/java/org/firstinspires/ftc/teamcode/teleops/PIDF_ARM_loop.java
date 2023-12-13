package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name = "PIDF_ARM_LOOP", group = "drive")
public class PIDF_ARM_loop extends OpMode {

    public PIDController lcontroller, rcontroller;
    public static double lp=0,li=0,ld=0,lf=0, rp=0, ri=0, rd=0, rf=0;
    public DcMotorEx l,r;

    public final double ticksInDegree = 4005.558676 /360.0;

    public static int targetL = 0, targetR = 0;

    public void init() {
        lcontroller = new PIDController(lp, li, ld);
        rcontroller = new PIDController(rp, ri, rd);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        l = hardwareMap.get(DcMotorEx.class, "armLeft");
        r = hardwareMap.get(DcMotorEx.class, "armRight");

    }
    public void loop() {
        lcontroller.setPID(lp,li,ld);
        int lPos = l.getCurrentPosition();
        double lpid = lcontroller.calculate(lPos, targetL);
        double lff = Math.cos(Math.toRadians(targetL / ticksInDegree)) * lf;

        double power = lpid + lff;

        l.setPower(power);
        r.setPower(-power);
        telemetry.addData("lPos: ", lPos);
        telemetry.addData("lTarget: ", targetL);
        telemetry.update();

        /*rcontroller.setPID(rp, ri, rd);
        double rpid = rcontroller.calculate(lPos, targetR);
        double rff = Math.cos(Math.toRadians(targetR / ticksInDegree)) * rf;

        telemetry.addData("rPos: ", )

        telemetry.update();*/
    }
}
