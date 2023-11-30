package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

public class VisionCamera {
    public TfodProcessor tfod;
    public VisionPortal visionPortal;

    public Telemetry telemetry;
    private static final String[] labels = {"BLUE ELEMENT NAME", "RED ELEMENT NAME"};
    private static final String TFOD_MODEL_ASSET = "/sdcard/FIRST/tflitemodels/YOUR MODEL NAME HERE.tflite";

    public VisionCamera(HardwareMap map, Telemetry tele) {
        telemetry = tele;
        tfod = new TfodProcessor.Builder()
                .setModelFileName(TFOD_MODEL_ASSET)
                .setModelLabels(labels)
                .setIsModelTensorFlow2(true)
                .setIsModelQuantized(true)
                .setModelInputSize(300)
                .build();
        tfod.setMinResultConfidence(0.8F);

        visionPortal = new VisionPortal.Builder()
                .setCamera(map.get(WebcamName.class, "Webcam 1"))
                .addProcessor(tfod)
                .build();
        //WILL ONLY SHOW DETECTIONS IF IT IS AT LEAST 80% CONFIDENT
        //YOU CAN ADJUST THIS
        tfod.setMinResultConfidence(0.8F);
    }

    public void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;

            telemetry.addData("", " ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }

    public String getSide() {
        List<Recognition> recognition = tfod.getRecognitions();
        if (recognition.isEmpty()) return "left";

        //what this does is it checks if the object detected is greater than x value 300 it is on the right
        //if its less than that its on the middle

        //OUR BOT IS SET UP SO THAT THE CAMERA CANNOT SEE THE LEFT TAPE
        //therefore if it doesnt detect anything its left

        for (int i = 0; i < recognition.size(); i++) {
            if (recognition.get(i).getWidth() > 250 || recognition.get(i).getHeight() > 250);
            else if (recognition.get(i).getLeft() > 300) return "right";
            else if (recognition.get(i).getLeft() <= 300) return "middle";
        }
        return "left";
    }
}