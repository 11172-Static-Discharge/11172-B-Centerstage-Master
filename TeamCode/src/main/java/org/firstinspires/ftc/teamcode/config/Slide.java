package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Slide
{
    private DcMotor slide;

    public Slide(Telemetry tele, HardwareMap map)
    {
        slide = map.dcMotor.get("slide");
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void set(double power)
    {
        slide.setPower(power);
    }
    public void pullDown() {slide.setPower(1);}
}