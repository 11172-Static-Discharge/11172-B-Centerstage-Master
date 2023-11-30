package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift
{
    private static int ERROR_MARGIN = 20;
    private DcMotor intake;
    private DcMotor rightArm, leftArm;
    private Servo clawR, clawL, liftR, liftL, wrist, launcher; //leftWrist;

    double clawLOpen = 1;
    double clawLClose = 0;

    double clawROpen = 0;
    double clawRClose = 1;

    public Lift(Telemetry tele, HardwareMap map)
    {
        clawR = map.servo.get("clawR");
        clawL = map.servo.get("clawL");
        liftR = map.servo.get("liftR");
        liftL = map.servo.get("liftL");
        wrist = map.servo.get("wrist");
        launcher = map.servo.get("launcher");

    }

    public void setLiftPos(double[] positions)
    {
        liftR.setPosition(positions[0]);
        liftL.setPosition(positions[1]);
        wrist.setPosition(positions[2]);
    }

    public void setRightClaw(boolean closed) {clawR.setPosition(closed ? clawRClose : clawROpen);}
    public void setLeftClaw(boolean closed) {clawL.setPosition(closed ? clawLClose : clawLOpen);}

    public void setLauncher(double position){launcher.setPosition(position);}


}
