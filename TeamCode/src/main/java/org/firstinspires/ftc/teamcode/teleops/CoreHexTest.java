package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "CoreHexTest", group = "teleops")
public class CoreHexTest extends LinearOpMode
{
    private DcMotor rightLift, leftLift;

    @Override
    public void runOpMode() throws InterruptedException {

        rightLift = hardwareMap.get(DcMotor.class, "rightArm");
        leftLift = hardwareMap.get(DcMotor.class, "leftArm");

        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        while(opModeIsActive())
        {
            telemetry.addData("Right Position", rightLift.getCurrentPosition());
            telemetry.addData("Left Position", leftLift.getCurrentPosition());

            telemetry.update();

            if(gamepad1.dpad_up)
            { rightLift.setDirection(DcMotorSimple.Direction.FORWARD);
                leftLift.setDirection(DcMotorSimple.Direction.FORWARD);
                rightLift.setPower(0.1);
                leftLift.setPower(0.1);
            }

            else if(gamepad1.dpad_down)
            {
                rightLift.setDirection(DcMotorSimple.Direction.REVERSE);
                leftLift.setDirection(DcMotorSimple.Direction.REVERSE);
                rightLift.setPower(0.1);
                leftLift.setPower(0.1);
            }

            else
            {
                rightLift.setPower(0);
                leftLift.setPower(0);
            }
        }
    }
}
