package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.config.Lift;

@Config
@TeleOp(name = "PoopTeleOp", group = "drive")
public class PoopTeleOp extends OpMode {

    public PIDController lcontroller, rcontroller;
    public static double p=0.1,i=0,d=0.0001,f=0.22;
    public DcMotorEx l,r;

    public final double ticksInDegree = 288 / 360.0;

    public static int targetL = 0;
    Lift lift;

    public void init() {
        lcontroller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        l = hardwareMap.get(DcMotorEx.class, "liftL");
        r = hardwareMap.get(DcMotorEx.class, "liftR");

        lift = new Lift(telemetry, hardwareMap);

        /*l.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        l.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        r.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/
    }
    public void loop() {







        //LIFT STUFF HERE
        if(Math.abs(gamepad1.left_stick_y) >= 0.2)
        {
            l.setPower(gamepad1.left_stick_y);
            r.setPower(gamepad1.left_stick_y);
            targetL = l.getCurrentPosition();
            return;
        }


        if(gamepad1.dpad_up)
        {
            targetL = -120;
            lift.setWristPosFixed(0.22);
        }

        if(gamepad1.dpad_right)
        {
            targetL = -20;
            lift.setWristPosFixed(0.25);
        }

        if(gamepad1.dpad_down)
        {
            targetL = -13;
            lift.setWristPosFixed(0.85);
        }

        lcontroller.setPID(p,i,d);
        int lPos = l.getCurrentPosition();
        if(lPos > 0) return;

        double pid = lcontroller.calculate(lPos, targetL);

        double ff = Math.cos(Math.toRadians(targetL / ticksInDegree)) * f;


        double power = pid + ff;

        l.setPower(power);
        r.setPower(power);
        telemetry.addData("lPos: ", lPos);
        telemetry.addData("lTarget: ", targetL);
        telemetry.update();
    }
}
