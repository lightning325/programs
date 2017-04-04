package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;

/**
 * Created by NYAN on 4/4/2017.
 */
public class PotentTest extends LinearOpMode
{
    public void runOpMode() throws InterruptedException
    {
        AnalogInput potentiometer;

        potentiometer = hardwareMap.analogInput.get("potent");

        double potentDelay;
        potentDelay = (float) potentiometer.getVoltage();

        telemetry.addData("Raw Voltage: ", Double.toString(potentDelay));
        telemetry.update();

        waitForStart();

        while(opModeIsActive())
        {
            potentDelay = (float) potentiometer.getVoltage();

            telemetry.addData("Raw Voltage: ", Double.toString(potentDelay));
            telemetry.update();
        }

    }

}