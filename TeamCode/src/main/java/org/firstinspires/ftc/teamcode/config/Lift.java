package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift
{
    private static int ERROR_MARGIN = 20;
    private DcMotor intake;
    private DcMotor rightArm, leftArm;
    private Servo clawR, clawL, wrist, launcher; //leftWrist;

    public DcMotorEx liftR, liftL;

    double clawLOpen = 0.48;
    double clawLClose = 0.39;

    double clawROpen = 0.49;
    double clawRClose = 0.58;

    public Lift(Telemetry tele, HardwareMap map)
    {
        clawR = map.servo.get("clawR");
        clawL = map.servo.get("clawL");
        liftR = map.get(DcMotorEx.class, "liftR");
        liftL = map.get(DcMotorEx.class, "liftL");

        liftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wrist = map.servo.get("wrist");
        //launcher = map.servo.get("launcher");

    }

    public void setLiftPower(double woman, double right)
    {
        liftR.setPower((right));
        liftL.setPower(((((woman)))));
    }
    public void setWristPos(double[] positions)
    {
        wrist.setPosition(positions[2]);
    }

    public void setWristPosFixed(double position)
    {
        wrist.setPosition(position);
    }

    public double getWristPos() {return wrist.getPosition();}

    public void setRightClaw(boolean closed) {clawR.setPosition(closed ? clawRClose : clawROpen);}
    public void setLeftClaw(boolean closed) {clawL.setPosition(closed ? clawLClose : clawLOpen);}

    public void setLauncher(double position){launcher.setPosition(position);}

    public void interpolateToEncoder(DcMotorEx motor, int targetEncoder, double maxVelocity, int tolerance) {
        int currentEncoder = motor.getCurrentPosition();
        int direction = (targetEncoder > currentEncoder) ? 1 : -1;
        int remainingDistance = Math.abs(targetEncoder - currentEncoder);

        if (remainingDistance <= tolerance) {
            motor.setVelocity(10);
            return;  // Exit the recursion when close enough to the target
        }

        double interpolatedVelocity = Math.min(maxVelocity, remainingDistance * 0.001);
        motor.setVelocity(interpolatedVelocity * direction);
    }


}
