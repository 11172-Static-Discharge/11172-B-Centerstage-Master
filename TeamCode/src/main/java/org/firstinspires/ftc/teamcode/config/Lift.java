package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift
{
    private static int ERROR_MARGIN = 20;
    private DcMotor intake;
    private DcMotor rightArm, leftArm;
    private Servo claw, rightWrist; //leftWrist;

    public Lift(Telemetry tele, HardwareMap map)
    {
        rightArm = map.get(DcMotor.class, "rightArm");
        leftArm = map.get(DcMotor.class, "leftArm");
        claw = map.servo.get("claw");
        //leftWrist = map.servo.get("leftWrist");
        rightWrist = map.servo.get("rightWrist");
    }

    public void setArmPower(double power, int direction)
    {
        DcMotorSimple.Direction dir = direction == 1 ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE;

        rightArm.setDirection(dir);
        leftArm.setDirection(dir);

        rightArm.setPower(power);
        leftArm.setPower(power);
    }

    public void setWristPosition(double position)
    {
        //leftWrist.setPosition(position);
        rightWrist.setPosition(position);
    }

    public void setMotorTargetPos(int rightPosition, int leftPosition)
    {


        rightArm.setTargetPosition(rightPosition);
        leftArm.setTargetPosition(leftPosition);

        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /*public boolean isAtTarget(int reachHeight, DcMotor motor)
    {return (Math.abs(motor.getCurrentPosition() - reachHeight) < ERROR_MARGIN);}*/

    public void openClaw() {claw.setPosition(0.75);}
    public void middleClaw() {claw.setPosition(0.225);}
    public void closeClaw() {claw.setPosition(1);}
}
