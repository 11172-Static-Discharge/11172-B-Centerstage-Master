package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;



public class ColorSensorRGB {
    public ColorSensor csensor;
    public int red;
    public int green;
    public int blue;


    public ColorSensorRGB(ColorSensor sensor) {
        csensor = sensor;
    }
    public String getColor() {
        red = csensor.red();
        blue = csensor.blue();
        green = csensor.green();
        if (red > 145 && blue<200 && green <200) {
            return "red";
        }
        if (blue > 190 && green<200 && red <150) {
            return "blue";
        }
        if (green<=200 && red<=150 && blue <=200)  {
            return "gray";
        }
        return "none";
        }
    }




