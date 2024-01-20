package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Lift
{
    private static int ERROR_MARGIN = 20;
    private DcMotor intake;
    private DcMotor rightArm, leftArm;
    public Servo clawR, clawL, wrist, launcher, autoClaw, dispenser; //leftWrist;

    public DcMotorEx liftR, liftL;

    public ColorRangeSensor sensorL, sensorR;

    public PIDFArm arm;

    double clawLOpen = 0.48;
    double clawLClose = 0.38;

    double autoClawClose = 0;

    double autoClawOpen = 1;

    double clawLPos = 0, clawRPos = 0;

    public boolean autoClosedL = false, autoClosedR = false;
    Telemetry tele;

    double clawROpen = 0.49;
    double clawRClose = 0.607;
    double wristPos, increment = 0.01, closeTol = 1, dispenserPos = 0;

    int sequenceCounter = 0;

    public int dropPos = 0;
    public Lift(Telemetry tele, HardwareMap map)
    {
        clawR = map.servo.get("clawR");
        clawL = map.servo.get("clawL");
       // autoClaw = map.servo.get("autoClaw");
        liftR = map.get(DcMotorEx.class, "liftR");
        liftL = map.get(DcMotorEx.class, "liftL");
        dispenser = map.servo.get("dispenser");



        liftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sensorL = map.get(ColorRangeSensor.class, "sensorL");
        sensorR = map.get(ColorRangeSensor.class, "sensorR");


        wrist = map.servo.get("wrist");
        this.tele = tele;
        launcher = map.servo.get("launcher");

        arm = new PIDFArm(liftL, liftR, tele, 10);

    }

    public void moveTo(int target)
    {
        arm.moveTo(target);
    }

    public void moveToPower(int target, double power)
    {
        arm.moveToPower(target, power);
    }
    public void setLiftPower(double woman, double right)
    {
        liftR.setPower((right));
        liftL.setPower(((((woman)))));
    }
    public void autoClose(boolean autoClose) {
        if(autoClose) {
            if (sensorL.getDistance(DistanceUnit.INCH)<closeTol) {
                clawL.setPosition(clawLClose);
                autoClosedL = true;
            }
            else if (sensorR.getDistance(DistanceUnit.INCH)<closeTol) {
                clawR.setPosition(clawRClose);
                autoClosedR = true;
            }
            else {
                autoClosedL = false;
                autoClosedR = false;
            }
        }
    }
    public void calClaw(boolean lClawL, boolean lClawR, boolean rClawL, boolean rClawR) {
        if (lClawL) clawL.setPosition(clawLPos - increment/2);
        if (lClawR) clawL.setPosition(clawLPos + increment/2);
        if (rClawL) clawR.setPosition(clawRPos + increment/2);
        if (rClawR) clawR.setPosition(clawRPos - increment/2);

        clawLPos = clawL.getPosition();
        clawRPos = clawR.getPosition();

        //tele.addData("clawLPos", clawLPos);
        //tele.addData("clawRPos", clawRPos);
        //tele.update();
    }

    public void setLiftPowerBetter(double power) {
        liftR.setPower(power);
        liftL.setPower(-power);
    }

    public void setDispenser(double pos) {
        dispenser.setPosition(pos);
    }

    public void calDispenser(boolean up, boolean down) {
        if(up) dispenser.setPosition(dispenserPos + increment);
        if(down) dispenser.setPosition(dispenserPos + increment);
        dispenserPos = dispenser.getPosition();
    }

    public void calibrateLift(boolean up, boolean down) {
        arm.calibratePos(up, down);
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
    public void calWrist(boolean up, boolean down) {

        if (up) {
            wrist.setPosition(wristPos + increment);
        }
        if (down) {
            wrist.setPosition(wristPos - increment);
        }
        wristPos = wrist.getPosition();

        //tele.addData("wristPos", wristPos);
        //tele.update();

    }



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
        interpolatedVelocity = (interpolatedVelocity/maxVelocity);
        motor.setPower(interpolatedVelocity * direction);
    }

    public void setAutoClaw(boolean closed)
    {
        autoClaw.setPosition(closed ? autoClawClose : autoClawOpen);
    }



    public void doSequence(boolean input) {
        if(input) {
            switch (sequenceCounter % 6) {
                case 0:
                    setRightClaw(true);
                    setLeftClaw(true);
                    break;
                case 1:
                    setWristPosFixed(0.25);
                    break;
                case 2:
                    switch(dropPos) {
                        case 0:

                    }
                    break;
                case 3:
                    break;
                case 4:
                    break;
                case 5:
                    break;
            }
            sequenceCounter++;
        }
    }


}
