package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.VisionCamera;

@Autonomous (name = "BaseAuto", group = "autos")
public class AutoFormat extends LinearOpMode {
    public VisionCamera camera;
    public double middlePos = 0.75;
    public double boardPos = 0.675;

    public MechDrive drivetrain;
    @Override
    public void runOpMode() throws InterruptedException {
        String path = "middle";
        camera = new VisionCamera(hardwareMap, telemetry, "REd");


        //all your init stuff here
        //under init() for teleop

        //at this stage you should have a wrapper for your drivetrain

        while (!isStarted()) {
            camera.telemetryTfod();
            path = camera.getSide();
        }

        waitForStart();
        sleep(500);

        switch(path) {
            case "left":
                //add code for your bot to drive to the left tape here

                break;
            case "right":
                //code for right tape


                break;
            case "middle":
                //code for middle tape


                break;
        }
    }
}