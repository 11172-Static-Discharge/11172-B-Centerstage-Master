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
    public static double p=0,i=0,d=0,f=0;
    public DcMotorEx l,r;

    public final double ticksInDegree = 4005.558676 /360.0;

    public static int targetL = 0;

    public void init() {
        lcontroller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        l = hardwareMap.get(DcMotorEx.class, "liftL");
        r = hardwareMap.get(DcMotorEx.class, "liftR");

    }
    public void loop() {
        lcontroller.setPID(p,i,d);
        int lPos = l.getCurrentPosition();
        double pid = lcontroller.calculate(lPos, targetL);
        double ff = Math.cos(Math.toRadians(targetL / ticksInDegree)) * f;

        double power = pid + ff;

        l.setPower(power);
        r.setPower(-power);
        telemetry.addData("lPos: ", lPos);
        telemetry.addData("lTarget: ", targetL);
        telemetry.update();
    }
}
