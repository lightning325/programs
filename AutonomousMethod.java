package org.firstinspires.ftc.robotcontroller.external.samples;

import android.app.Activity;
import android.graphics.Color;
import android.hardware.Sensor;
import android.view.View;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.ftcrobotcontroller.R;
//import com.qualcomm.hardware.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * Write a description of class Autonomous here.
 *
 * @author (your name)
 * @version (a version number or a date)
 */
public class AutonomousMethod extends LinearOpMode {

    public void runOpMode() throws InterruptedException {


    }

    GyroSensor sensorGyro;
    ColorSensor color;

    DcMotor lefttread;
    DcMotor leftbacktread;
    DcMotor righttread;
    DcMotor rightbacktread;
    DcMotor motorgear;
    DcMotor motorwheel;

    Servo bumper1;
    Servo bumper2;
    Servo servolift;
    Servo leftDiver;
    Servo rightDiver;

    TouchSensor touchSensor;
    TouchSensor toggleSwitch;
    ColorSensor sensorRGB1;

    int xVal, yVal, zVal = 0;
    int heading = 0;

    float hsvValues[] = {0F, 0F, 0F};

    int y = 0;
    int x = 0;
    int count = 0;
    int diff = 0;

    public void driveStraight(boolean forward, double power) throws InterruptedException {

        if (forward)
        {
            rightbacktread.setPower(power);
            righttread.setPower(power);
            lefttread.setPower(power);
            leftbacktread.setPower(power);
        }
        else
        {
            rightbacktread.setPower(-power);
            righttread.setPower(-power);
            lefttread.setPower(-power);
            leftbacktread.setPower(-power);
        }
    }

    public void stopDrive() throws InterruptedException {
        righttread.setPower(0.0);
        rightbacktread.setPower(0.0);
        lefttread.setPower(0.0);
        leftbacktread.setPower(0.0);
    }

    public void gyroInit() throws InterruptedException {

        hardwareMap.logDevices();

        sensorGyro.calibrate();

        waitForStart();

        while (sensorGyro.isCalibrating())  {
            Thread.sleep(50);
        }
    }

    public void gyroSenseHeading() throws InterruptedException {
        xVal = sensorGyro.rawX();
        yVal = sensorGyro.rawY();
        zVal = sensorGyro.rawZ();

        heading = sensorGyro.getHeading();

        telemetry.addData("Heading ", String.format("%03d", heading));

        Thread.sleep(100);
    }

    public void setPowers(boolean turn, boolean right, double power) {
        if(turn == true) {
            if (right == true)
            {
                leftbacktread.setPower(-power);
                lefttread.setPower(-power);
                rightbacktread.setPower(power);
                righttread.setPower(power);
            }
            else
            {
                leftbacktread.setPower(power);
                lefttread.setPower(power);
                rightbacktread.setPower(-power);
                righttread.setPower(-power);
            }
        }
        else
        {
            leftbacktread.setPower(power);
            lefttread.setPower(power);
            rightbacktread.setPower(power);
            righttread.setPower(power);
        }
    }

    public String readColor() throws InterruptedException {

        DbgLog.error("Got Here1");

        double RGBaverage;
        telemetry.addData("got here","");

        RGBaverage = ((color.red() + color.blue() + color.green()) / 3);
        String debug = "RGBAverage:" + RGBaverage;
        DbgLog.error(debug);

        //Color.RGBToHSV((color.red() * 255) / 800, (color.green() * 255) / 800, (color.blue() * 255) / 800, hsvValues);
/*
            telemetry.addData("Clear", color.alpha());
            telemetry.addData("Red ", color.red());
            telemetry.addData("Green", color.green());
            telemetry.addData("Blue ", color.blue());
            telemetry.addData("Average", RGBaverage);
*/
            if (RGBaverage >= 3.5) {
                return "White";
            } else

            //waitOneFullHardwareCycle();

                return"";
    }
}






