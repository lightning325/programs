package org.firstinspires.ftc.robotcontroller.external.samples;

import android.app.Activity;
import android.graphics.Color;
import android.text.method.Touch;
import android.view.View;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Created by NYAN on 3/15/2017.
 */
public class OneAuto40 extends LinearOpMode {
    boolean Support; // If true{runFull()}; If false{runSupport()};
    boolean BeaconFirst; // If true{runBeacons()}; If false{runBalls()};
    boolean ColorRed; // If true{runRed()}; If false{runBlue()};
    boolean ParkCenter; // If true{runCenter()}; If false{runCorner()};

    DeviceInterfaceModule cdim;
    DcMotor DriveMotor1;
    DcMotor DriveMotor2;
    DcMotor DriveMotor3;
    DcMotor DriveMotor4;
    DcMotor Ballsack;

    Servo ButtonPresser;
    Servo BallLoad;

    TouchSensor touch;
    TouchSensor BeaconSwitch;
    TouchSensor ColorSwitch;
    TouchSensor ParkSwitch;
    TouchSensor SupportSwitch;

    ModernRoboticsI2cGyro gyro;
    ColorSensor color;

    int heading = 0;
    int angleZ = 0;

    public void runOpMode() throws InterruptedException {
        hardwareMap.logDevices();

        DriveMotor1 = hardwareMap.dcMotor.get("left1");
        DriveMotor2 = hardwareMap.dcMotor.get("left2");
        DriveMotor3 = hardwareMap.dcMotor.get("right1");
        DriveMotor4 = hardwareMap.dcMotor.get("right2");
        Ballsack = hardwareMap.dcMotor.get("ballsack");

        ButtonPresser = hardwareMap.servo.get("servobp");
        BallLoad = hardwareMap.servo.get("loader");

        touch = hardwareMap.touchSensor.get("touch");
        BeaconSwitch = hardwareMap.touchSensor.get("BeaconSwitch");
        ColorSwitch = hardwareMap.touchSensor.get("ColorSwitch");
        ParkSwitch = hardwareMap.touchSensor.get("ParkSwitch");
        SupportSwitch = hardwareMap.touchSensor.get("SupportSwitch");

        DriveMotor3.setDirection(DcMotor.Direction.REVERSE);
        DriveMotor4.setDirection(DcMotor.Direction.REVERSE);

        cdim = hardwareMap.deviceInterfaceModule.get("dim");

        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        color = hardwareMap.colorSensor.get("color");

        // start calibrating the gyro.
        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        gyro.calibrate();

        // make sure the gyro is calibrated.
        while (!isStopRequested() && gyro.isCalibrating()) {
            sleep(50);
            idle();
        }

        // Initialize color sensor values
        float hsvValues[] = {0F, 0F, 0F};

        final float values[] = hsvValues;

        // Setup coloring for screen so screen color changes accordingly
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(R.id.RelativeLayout);

        boolean bPrevState = false;
        boolean bCurrState = false;

        DriveMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveMotor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveMotor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ButtonPresser.setPosition(1.0);

        BallLoad.setPosition(0.8);

        color.enableLed(false);

        int DesiredAngle;

        DriveMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        DriveMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        DriveMotor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DriveMotor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        if(SupportSwitch.isPressed())
        {
            Support = false;
        }
        else
        {
            Support = true;
        }

        if(BeaconSwitch.isPressed())
        {
            BeaconFirst = true;
        }
        else
        {
            BeaconFirst = false;
        }

        if(ColorSwitch.isPressed())
        {
            ColorRed = true;
        }
        else
        {
            ColorRed = false;
        }

        if(ParkSwitch.isPressed())
        {
            ParkCenter = true;
        }
        else
        {
            ParkCenter = false;
        }

        if(Support)
        {
            if(ColorRed)
            {
                if(ParkCenter)
                {
                    telemetry.addData("", "Support, Red, Center");
                    telemetry.update();
                }
                else
                {
                    telemetry.addData("", "Support, Red, Ramp");
                    telemetry.update();
                }
            }
            else
            {
                if(ParkCenter)
                {
                    telemetry.addData("", "Support, Blue, Center");
                    telemetry.update();
                }
                else
                {
                    telemetry.addData("", "Support, Blue, Ramp");
                    telemetry.update();
                }
            }
        }
        else
        {
            if(ColorRed)
            {
                if(BeaconFirst)
                {
                    if(ParkCenter)
                    {
                        telemetry.addData("", "Full, Red, Beacons, Center");
                        telemetry.update();
                    }
                    else
                    {
                        telemetry.addData("", "Full, Red, Beacons, Ramp");
                        telemetry.update();
                    }
                }
                else
                {
                    if(ParkCenter)
                    {
                        telemetry.addData("", "Full, Red, Balls, Center");
                        telemetry.update();
                    }
                    else
                    {
                        telemetry.addData("", "Full, Red, Balls, Ramp");
                        telemetry.update();
                    }
                }
            }
            else
            {
                if(BeaconFirst)
                {
                    if(ParkCenter)
                    {
                        telemetry.addData("", "Full, Blue, Beacons, Center");
                        telemetry.update();
                    }
                    else
                    {
                        telemetry.addData("", "Full, Blue, Beacons, Ramp");
                        telemetry.update();
                    }
                }
                else
                {
                    if(ParkCenter)
                    {
                        telemetry.addData("", "Full, Blue, Balls, Center");
                        telemetry.update();
                    }
                    else
                    {
                        telemetry.addData("", "Full, Blue, Balls, Ramp");
                        telemetry.update();
                    }
                }
            }
        }

        /*telemetry.addData("Support: ", Support);
        telemetry.addData("Red: ", ColorRed);
        telemetry.addData("Center Park: ", ParkCenter);
        telemetry.addData("Beacon First: ", BeaconFirst);
        telemetry.update();
        */


        while (opModeIsActive()) {
            Ballsack.setPower(-0.35);
            if (touch.isPressed()) {
                Ballsack.setPower(0.0);
                break;
            }
        }
        Ballsack.setPower(0.0);

        waitForStart();

        // Support
        if (Support)
        {

            // Support
            if (BeaconFirst == false)
            {

                // Support, Red
                if (ColorRed)
                {
                    gyro.resetZAxisIntegrator();
                    sleep(150);

                    angleZ = gyro.getIntegratedZValue();

                    DriveInches(0.3, 19, true);

                    DriveMotor3.setPower(-0.275);
                    DriveMotor4.setPower(-0.275);

                    while(angleZ > -23)
                    {
                        telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
                        telemetry.update();
                        angleZ = gyro.getIntegratedZValue();
                    }
                    DriveForward(0);
                    sleep(500);

                    Ballsack.setPower(-1.0);
                    sleep(115);

                    Ballsack.setPower(0.0);
                    sleep(1000);

                    while (opModeIsActive())
                    {
                        Ballsack.setPower(-0.35);
                        if (touch.isPressed()) {
                            Ballsack.setPower(0.0);
                            break;
                        }
                    }
                    Ballsack.setPower(0.0);

                    BallLoad.setPosition(0.0);
                    sleep(1250);

                    BallLoad.setPosition(0.8);
                    sleep(700);

                    Ballsack.setPower(-1.0);
                    sleep(115);

                    Ballsack.setPower(0.0);
                    sleep(2200);

                    // Support, Red, Center
                    if(ParkCenter)
                    {
                        DriveMotor1.setPower(0.275);
                        DriveMotor2.setPower(0.275);
                        DriveMotor3.setPower(-0.275);
                        DriveMotor4.setPower(-0.275);

                        while(angleZ > -110)
                        {
                            telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
                            telemetry.update();
                            angleZ = gyro.getIntegratedZValue();
                        }
                        DriveForward(0);
                        sleep(100);


                        DriveInches(0.3, 36, false);
                    }

                    // Support, Red, Corner
                    else
                    {
                        DriveMotor1.setPower(0.275);
                        DriveMotor2.setPower(0.275);
                        DriveMotor3.setPower(-0.275);
                        DriveMotor4.setPower(-0.275);

                        while(angleZ > -55)
                        {
                            telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
                            telemetry.update();
                            angleZ = gyro.getIntegratedZValue();
                        }
                        DriveForward(0);
                        sleep(100);


                        DriveInches(0.3, 65, false);
                    }
                }

                // Support, Blue
                else
                {
                    gyro.resetZAxisIntegrator();
                    sleep(150);

                    DriveInches(0.3, 21, false);

                    angleZ = gyro.getIntegratedZValue();

                    DriveMotor3.setPower(0.275);
                    DriveMotor4.setPower(0.275);

                    while(angleZ < 37)
                    {
                        telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
                        telemetry.update();
                        angleZ = gyro.getIntegratedZValue();
                    }
                    DriveForward(0);
                    sleep(500);

                    Ballsack.setPower(-1.0);
                    sleep(115);

                    Ballsack.setPower(0.0);
                    sleep(1000);

                    while (opModeIsActive())
                    {
                        Ballsack.setPower(-0.35);
                        if (touch.isPressed()) {
                            Ballsack.setPower(0.0);
                            break;
                        }
                    }
                    Ballsack.setPower(0.0);

                    BallLoad.setPosition(0.0);
                    sleep(1250);

                    BallLoad.setPosition(0.8);
                    sleep(700);

                    Ballsack.setPower(-1.0);
                    sleep(115);

                    Ballsack.setPower(0.0);
                    sleep(2200);

                    // Support, Blue, Center
                    if(ParkCenter)
                    {
                        angleZ = gyro.getIntegratedZValue();

                        DriveMotor1.setPower(-0.275);
                        DriveMotor2.setPower(-0.275);
                        DriveMotor3.setPower(0.275);
                        DriveMotor4.setPower(0.275);

                        while(angleZ < 100)
                        {
                            telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
                            telemetry.update();
                            angleZ = gyro.getIntegratedZValue();
                        }
                        DriveForward(0);
                        sleep(500);

                        DriveInches(0.3, 30, true);
                    }

                    // Support, Blue, Corner
                    else
                    {
                        angleZ = gyro.getIntegratedZValue();

                        DriveMotor1.setPower(-0.275);
                        DriveMotor2.setPower(-0.275);
                        DriveMotor3.setPower(0.275);
                        DriveMotor4.setPower(0.275);

                        while(angleZ < 67)
                        {
                            telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
                            telemetry.update();
                            angleZ = gyro.getIntegratedZValue();
                        }
                        DriveForward(0);
                        sleep(500);

                        DriveInches(0.3, 69, true);
                    }
                }
            }

            // Defensive
            else
            {

                // Red Defensive
                if(ColorRed)
                {
                    gyro.resetZAxisIntegrator();
                    sleep(150);

                    angleZ = gyro.getIntegratedZValue();

                    DriveInches(0.3, 19, true);

                    DriveMotor3.setPower(-0.275);
                    DriveMotor4.setPower(-0.275);

                    while(angleZ > -23)
                    {
                        telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
                        telemetry.update();
                        angleZ = gyro.getIntegratedZValue();
                    }
                    DriveForward(0);
                    sleep(500);

                    Ballsack.setPower(-1.0);
                    sleep(115);

                    Ballsack.setPower(0.0);
                    sleep(1000);

                    while (opModeIsActive())
                    {
                        Ballsack.setPower(-0.35);
                        if (touch.isPressed()) {
                            Ballsack.setPower(0.0);
                            break;
                        }
                    }
                    Ballsack.setPower(0.0);

                    BallLoad.setPosition(0.0);
                    sleep(1250);

                    BallLoad.setPosition(0.8);
                    sleep(700);

                    Ballsack.setPower(-1.0);
                    sleep(115);

                    Ballsack.setPower(0.0);
                    sleep(2200);

                    DriveMotor1.setPower(-0.2);
                    DriveMotor2.setPower(-0.2);

                    DriveInches(0.4, 44, true);

                    DriveForward(0);

                    sleep(300);

                    DriveMotor1.setPower(-0.275);
                    DriveMotor2.setPower(-0.275);

                    while(angleZ < 34)
                    {
                        telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
                        telemetry.update();
                        angleZ = gyro.getIntegratedZValue();
                    }

                    DriveForward(0);

                    DriveInches(0.4, 60, true);

                    DriveForward(0);

                    DriveMotor3.setPower(-0.275);
                    DriveMotor4.setPower(-0.275);

                    while(angleZ > 0)
                    {
                        telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
                        telemetry.update();
                        angleZ = gyro.getIntegratedZValue();
                    }
                    DriveForward(0);

                }

                //Blue Defense
                else
                {
                    gyro.resetZAxisIntegrator();
                    sleep(150);

                    DriveInches(0.3, 21, false);

                    angleZ = gyro.getIntegratedZValue();

                    DriveMotor3.setPower(0.275);
                    DriveMotor4.setPower(0.275);

                    while(angleZ < 37)
                    {
                        telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
                        telemetry.update();
                        angleZ = gyro.getIntegratedZValue();
                    }
                    DriveForward(0);
                    sleep(500);

                    Ballsack.setPower(-1.0);
                    sleep(115);

                    Ballsack.setPower(0.0);
                    sleep(1000);

                    while (opModeIsActive())
                    {
                        Ballsack.setPower(-0.35);
                        if (touch.isPressed()) {
                            Ballsack.setPower(0.0);
                            break;
                        }
                    }
                    Ballsack.setPower(0.0);

                    BallLoad.setPosition(0.0);
                    sleep(1250);

                    BallLoad.setPosition(0.8);
                    sleep(700);

                    Ballsack.setPower(-1.0);
                    sleep(115);

                    Ballsack.setPower(0.0);
                    sleep(2200);


                    angleZ = gyro.getIntegratedZValue();

                    DriveMotor1.setPower(0.275);
                    DriveMotor2.setPower(0.275);

                    while(angleZ > 34)
                    {
                        telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
                        telemetry.update();
                        angleZ = gyro.getIntegratedZValue();
                    }

                    DriveInches(0.4, 39, false);

                    DriveForward(0);

                    sleep(300);


                    angleZ = gyro.getIntegratedZValue();

                    DriveMotor1.setPower(0.275);
                    DriveMotor2.setPower(0.275);

                    while(angleZ > -30)
                    {
                        telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
                        telemetry.update();
                        angleZ = gyro.getIntegratedZValue();
                    }

                    DriveForward(0);

                    DriveInches(0.4, 50, false);

                    DriveForward(0);


                    angleZ = gyro.getIntegratedZValue();

                    DriveMotor3.setPower(0.275);
                    DriveMotor4.setPower(0.275);

                    while(angleZ < 0)
                    {
                        telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
                        telemetry.update();
                        angleZ = gyro.getIntegratedZValue();
                    }
                    DriveForward(0);
                }
            }
        }

        // Full
        else
        {

            color.enableLed(false);
            gyro.resetZAxisIntegrator();

            // Full, Red
            if (ColorRed)
            {

                // Full, Red, Beacons
                if (BeaconFirst)
                {
                    gyro.resetZAxisIntegrator();
                    angleZ = gyro.getIntegratedZValue();
                    sleep(150);

                    DriveInches(0.3, 56, true);

                    DriveForward(0);
                    sleep(500);

                    angleZ = gyro.getIntegratedZValue();

                    DriveMotor1.setPower(0.275);
                    DriveMotor2.setPower(0.275);
                    DriveMotor3.setPower(-0.275);
                    DriveMotor4.setPower(-0.275);

                    while (true)
                    {
                        telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
                        telemetry.update();
                        if (angleZ <= -14) {
                            telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
                            telemetry.update();

                            DriveForward(0);
                            break;
                        }

                        angleZ = gyro.getIntegratedZValue();
                    }
                    DriveForward(0);
                    sleep(100);

                    DriveInches(0.375, 23, true);

                    while (opModeIsActive()) {
                        DriveMotor1.setPower(0.11);
                        DriveMotor2.setPower(0.11);
                        DriveMotor3.setPower(0.15);
                        DriveMotor4.setPower(0.15);

                        color.enableLed(false);

                        telemetry.addData("Red   ", color.red());
                        telemetry.addData("Blue  ", color.blue());
                        telemetry.addData("Green ", color.green());
                        telemetry.update();

                        if (color.red() >= 3) {
                            DriveForward(0);
                            break;
                        }


                        double RGBaverage = ((color.red() + color.blue() + color.green()) / 3);
                        telemetry.addData("Final Value", RGBaverage);
                    }
                    DriveInches(0.2, 1, true);

                    DriveForward(0);
                    sleep(250);

                    ButtonPresser.setPosition(0.7);
                    sleep(500);
                    ButtonPresser.setPosition(1.0);

                    color.enableLed(true);
                    DriveForward(-0.2);
                    sleep(1500);

                    while (opModeIsActive()) {
                        DriveMotor1.setPower(-0.11);
                        DriveMotor2.setPower(-0.11);
                        DriveMotor3.setPower(-0.15);
                        DriveMotor4.setPower(-0.15);

                        color.enableLed(false);

                        telemetry.addData("Red   ", color.red());
                        telemetry.addData("Blue  ", color.blue());
                        telemetry.addData("Green ", color.green());
                        telemetry.update();

                        if (color.red() >= 3) {
                            DriveForward(0);
                            break;
                        }

                        double RGBaverage = ((color.red() + color.blue() + color.green()) / 3);
                        telemetry.addData("Final Value", RGBaverage);
                    }
                    DriveForward(0);
                    sleep(250);

                    ButtonPresser.setPosition(0.7);
                    sleep(500);
                    ButtonPresser.setPosition(1.0);

                    color.enableLed(true);
                    sleep(250);

                    DriveForward(0.245);

                    while (opModeIsActive()) {

                        sleep(50);
                        telemetry.addData("Red   ", color.red());
                        telemetry.addData("Blue  ", color.blue());
                        telemetry.addData("Green ", color.green());
                        telemetry.update();

                        if (color.red() <= 1 && color.blue() <= 1) {
                            DriveForward(0);
                            break;
                        }
                    }
                    DriveForward(0);
                    sleep(200);

                    color.enableLed(false);

                    DriveInchesTurn(0.3, 0.2, 36, true);

                    DriveForward(0);
                    sleep(50);
                    gyro.resetZAxisIntegrator();
                    angleZ = gyro.getIntegratedZValue();
                    sleep(250);

                    DriveMotor1.setPower(0.225);
                    DriveMotor2.setPower(0.225);
                    DriveMotor3.setPower(-0.225);
                    DriveMotor4.setPower(-0.225);

                    while (1 == 1) {
                        telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
                        telemetry.update();

                        if (angleZ <= -46) {
                            telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
                            telemetry.update();

                            DriveForward(0);
                            break;
                        }

                        angleZ = gyro.getIntegratedZValue();
                    }
                    DriveForward(0);
                    sleep(250);

                    DriveInches(0.22, 12, true);

                    DriveForward(0);
                    sleep(250);

                    DriveMotor1.setPower(-0.225);
                    DriveMotor2.setPower(-0.225);
                    DriveMotor3.setPower(0.225);
                    DriveMotor4.setPower(0.225);

                    while (1 == 1) {
                        telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
                        telemetry.update();

                        if (angleZ >= 0) {
                            telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
                            telemetry.update();

                            DriveForward(0);
                            break;
                        }

                        angleZ = gyro.getIntegratedZValue();
                    }
                    DriveForward(0);

                    //Shoot balls
                    Ballsack.setPower(-1.0);
                    sleep(115);

                    Ballsack.setPower(0.0);
                    sleep(1000);

                    while (opModeIsActive()) {
                        Ballsack.setPower(-0.35);
                        if (touch.isPressed()) {
                            Ballsack.setPower(0.0);
                            break;
                        }
                    }
                    Ballsack.setPower(0.0);

                    BallLoad.setPosition(0.0);
                    sleep(1250);

                    BallLoad.setPosition(0.8);
                    sleep(700);

                    Ballsack.setPower(-1.0);
                    sleep(115);

                    Ballsack.setPower(0.0);
                    sleep(200);

                    // Full, Red, Beacons, Center
                    if (ParkCenter) {
                        gyro.resetZAxisIntegrator();
                        angleZ = gyro.getIntegratedZValue();
                        sleep(250);

                        DriveMotor1.setPower(0.225);
                        DriveMotor2.setPower(0.225);
                        DriveMotor3.setPower(-0.225);
                        DriveMotor4.setPower(-0.225);

                        while (1 == 1) {
                            telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
                            telemetry.update();

                            if (angleZ <= -69) {
                                telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
                                telemetry.update();

                                DriveForward(0);
                                break;
                            }

                            angleZ = gyro.getIntegratedZValue();
                        }
                        DriveForward(0);
                        sleep(250);

                        DriveInches(0.3, 32, true);
                        DriveForward(0);
                    }

                    // Full, Red, Beacons, Corner
                    else {
                        gyro.resetZAxisIntegrator();
                        angleZ = gyro.getIntegratedZValue();
                        sleep(250);

                        DriveMotor1.setPower(-0.225);
                        DriveMotor2.setPower(-0.225);
                        DriveMotor3.setPower(0.225);
                        DriveMotor4.setPower(0.225);

                        while (1 == 1) {
                            telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
                            telemetry.update();

                            if (angleZ >= 6) {
                                telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
                                telemetry.update();

                                DriveForward(0);
                                break;
                            }

                            angleZ = gyro.getIntegratedZValue();
                        }
                        DriveForward(0);
                        sleep(250);

                        DriveInches(0.3, 58, false);
                        DriveForward(0);
                    }
                }
                // Full, Red, Balls
                else {
                    gyro.resetZAxisIntegrator();

                    angleZ = gyro.getIntegratedZValue();

                    DriveInches(0.2, 34, true);

                    DriveForward(0);
                    sleep(500);

                    Ballsack.setPower(-1.0);
                    sleep(115);

                    Ballsack.setPower(0.0);
                    sleep(1000);

                    while (opModeIsActive()) {
                        Ballsack.setPower(-0.35);
                        if (touch.isPressed()) {
                            Ballsack.setPower(0.0);
                            break;
                        }
                    }
                    Ballsack.setPower(0.0);

                    BallLoad.setPosition(0.0);
                    sleep(1250);

                    BallLoad.setPosition(0.8);
                    sleep(700);

                    Ballsack.setPower(-1.0);
                    sleep(115);

                    Ballsack.setPower(0.0);
                    sleep(700);

                    DriveInches(0.2, 25, true);
                    DriveForward(0);
                    sleep(500);

                    angleZ = gyro.getIntegratedZValue();

                    DriveMotor1.setPower(0.175);
                    DriveMotor2.setPower(0.175);
                    DriveMotor3.setPower(-0.175);
                    DriveMotor4.setPower(-0.175);

                    while (true) {
                        telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
                        telemetry.update();
                        if (angleZ <= -14) {
                            telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
                            telemetry.update();

                            DriveForward(0);
                            break;
                        }

                        angleZ = gyro.getIntegratedZValue();
                    }
                    DriveForward(0);
                    sleep(100);

                    DriveInches(0.2, 23, true);

                    while (opModeIsActive()) {
                        DriveMotor1.setPower(0.11);
                        DriveMotor2.setPower(0.11);
                        DriveMotor3.setPower(0.15);
                        DriveMotor4.setPower(0.15);

                        color.enableLed(false);

                        telemetry.addData("Red   ", color.red());
                        telemetry.addData("Blue  ", color.blue());
                        telemetry.addData("Green ", color.green());
                        telemetry.update();

                        if (color.red() >= 3) {
                            DriveForward(0);
                            break;
                        }


                        double RGBaverage = ((color.red() + color.blue() + color.green()) / 3);
                        telemetry.addData("Final Value", RGBaverage);
                    }
                    DriveForward(0);
                    sleep(250);

                    ButtonPresser.setPosition(0.7);
                    sleep(500);
                    ButtonPresser.setPosition(1.0);

                    color.enableLed(true);
                    DriveForward(-0.18);
                    sleep(1500);

                    while (opModeIsActive()) {
                        DriveMotor1.setPower(-0.09);
                        DriveMotor2.setPower(-0.09);
                        DriveMotor3.setPower(-0.15);
                        DriveMotor4.setPower(-0.15);

                        color.enableLed(false);

                        telemetry.addData("Red   ", color.red());
                        telemetry.addData("Blue  ", color.blue());
                        telemetry.addData("Green ", color.green());
                        telemetry.update();

                        if (color.red() >= 3) {
                            DriveForward(0);
                            break;
                        }

                        double RGBaverage = ((color.red() + color.blue() + color.green()) / 3);
                        telemetry.addData("Final Value", RGBaverage);
                    }
                    DriveForward(0);
                    sleep(250);

                    ButtonPresser.setPosition(0.7);
                    sleep(500);
                    ButtonPresser.setPosition(1.0);

                    color.enableLed(true);
                    sleep(250);

                    DriveForward(0.245);

                    while (opModeIsActive()) {

                        sleep(50);
                        telemetry.addData("Red   ", color.red());
                        telemetry.addData("Blue  ", color.blue());
                        telemetry.addData("Green ", color.green());
                        telemetry.update();

                        if (color.red() <= 1 && color.blue() <= 1) {
                            DriveForward(0);
                            break;
                        }
                    }
                    DriveForward(0);
                    sleep(200);

                    color.enableLed(false);

                    // Full, Red, Balls, Center
                    if (ParkCenter) {
                        DriveInchesTurn(0.3, 0.2, 36, true);

                        DriveForward(0);

                        gyro.resetZAxisIntegrator();

                        angleZ = gyro.getIntegratedZValue();
                        sleep(250);

                        DriveMotor1.setPower(0.225);
                        DriveMotor2.setPower(0.225);
                        DriveMotor3.setPower(-0.225);
                        DriveMotor4.setPower(-0.225);

                        while (1 == 1) {
                            telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
                            telemetry.update();

                            if (angleZ <= -54) {
                                telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
                                telemetry.update();

                                DriveForward(0);
                                break;
                            }

                            angleZ = gyro.getIntegratedZValue();
                        }
                        DriveForward(0);
                        sleep(250);

                        DriveInches(0.3, 45, true);

                    }
                    // Full, Red, Balls, Corner
                    else {
                        gyro.resetZAxisIntegrator();
                        sleep(150);

                        angleZ = gyro.getIntegratedZValue();

                        DriveMotor1.setPower(-0.55);
                        DriveMotor2.setPower(-0.55);
                        DriveMotor3.setPower(0.55);
                        DriveMotor4.setPower(0.55);

                        while (1 == 1) {
                            telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
                            telemetry.update();
                            if (angleZ >= 7) {
                                telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
                                telemetry.update();
                                sleep(100);

                                DriveForward(0);
                                break;
                            }

                            angleZ = gyro.getIntegratedZValue();
                        }
                        DriveForward(0);

                        DriveInches(0.3, 36, false);
                    }
                }
            }
            // Full, Blue
            else
            {
                color.enableLed(false);

                // Full, Blue, Beacons
                if (BeaconFirst) {
                    gyro.resetZAxisIntegrator();
                    angleZ = gyro.getIntegratedZValue();
                    sleep(150);

                    DriveInches(0.3, 56, false);

                    DriveForward(0);
                    sleep(600);

                    angleZ = gyro.getIntegratedZValue();

                    DriveMotor1.setPower(-0.175);
                    DriveMotor2.setPower(-0.175);
                    DriveMotor3.setPower(0.175);
                    DriveMotor4.setPower(0.175);

                    while (true) {
                        telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
                        telemetry.update();
                        if (angleZ >= 16) {
                            telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
                            telemetry.update();

                            DriveForward(0);
                            break;
                        }

                        angleZ = gyro.getIntegratedZValue();
                    }
                    DriveForward(0);
                    sleep(100);

                    DriveInches(0.275, 23, false);

                    while (opModeIsActive()) {
                        DriveMotor1.setPower(-0.11);
                        DriveMotor2.setPower(-0.11);
                        DriveMotor3.setPower(-0.15);
                        DriveMotor4.setPower(-0.15);

                        color.enableLed(false);

                        telemetry.addData("Red   ", color.red());
                        telemetry.addData("Blue  ", color.blue());
                        telemetry.addData("Green ", color.green());
                        telemetry.update();

                        if (color.blue() >= 3) {
                            DriveForward(0);
                            break;
                        }


                        double RGBaverage = ((color.red() + color.blue() + color.green()) / 3);
                        telemetry.addData("Final Value", RGBaverage);
                    }
                    DriveForward(0);
                    sleep(250);

                    ButtonPresser.setPosition(0.7);
                    sleep(500);
                    ButtonPresser.setPosition(1.0);

                    color.enableLed(true);
                    DriveInchesTurn(0.15, 0.2, 10, true);

                    while (opModeIsActive()) {
                        DriveMotor1.setPower(0.11);
                        DriveMotor2.setPower(0.11);
                        DriveMotor3.setPower(0.15);
                        DriveMotor4.setPower(0.15);

                        color.enableLed(false);

                        telemetry.addData("Red   ", color.red());
                        telemetry.addData("Blue  ", color.blue());
                        telemetry.addData("Green ", color.green());
                        telemetry.update();

                        if (color.blue() >= 3) {
                            DriveForward(0);
                            break;
                        }

                        double RGBaverage = ((color.red() + color.blue() + color.green()) / 3);
                        telemetry.addData("Final Value", RGBaverage);
                    }
                    DriveInches(0.2, 1, true);

                    DriveForward(0);
                    sleep(250);

                    ButtonPresser.setPosition(0.7);
                    sleep(500);
                    ButtonPresser.setPosition(1.0);

                    color.enableLed(true);
                    sleep(250);

                    DriveForward(-0.245);

                    while (opModeIsActive()) {

                        sleep(50);
                        telemetry.addData("Red   ", color.red());
                        telemetry.addData("Blue  ", color.blue());
                        telemetry.addData("Green ", color.green());
                        telemetry.update();

                        if (color.red() <= 1 && color.blue() <= 1) {
                            DriveForward(0);
                            break;
                        }
                    }
                    DriveForward(0);
                    sleep(200);

                    color.enableLed(false);

                    DriveInchesTurn(0.3, 0.2, 36, false);

                    DriveForward(0);
                    sleep(50);
                    gyro.resetZAxisIntegrator();
                    angleZ = gyro.getIntegratedZValue();
                    sleep(250);

                    DriveMotor1.setPower(-0.225);
                    DriveMotor2.setPower(-0.225);
                    DriveMotor3.setPower(0.225);
                    DriveMotor4.setPower(0.225);

                    while (1 == 1) {
                        telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
                        telemetry.update();

                        if (angleZ >= 54) {
                            telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
                            telemetry.update();

                            DriveForward(0);
                            break;
                        }

                        angleZ = gyro.getIntegratedZValue();
                    }
                    DriveForward(0);
                    sleep(250);

                    DriveInches(0.22, 10, false);

                    DriveForward(0);
                    sleep(250);

                    DriveMotor1.setPower(0.225);
                    DriveMotor2.setPower(0.225);
                    DriveMotor3.setPower(-0.225);
                    DriveMotor4.setPower(-0.225);

                    while (1 == 1) {
                        telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
                        telemetry.update();

                        if (angleZ <= 20) {
                            telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
                            telemetry.update();

                            DriveForward(0);
                            break;
                        }

                        angleZ = gyro.getIntegratedZValue();
                    }
                    DriveForward(0);

                    //Shoot balls
                    Ballsack.setPower(-1.0);
                    sleep(115);

                    Ballsack.setPower(0.0);
                    sleep(1000);

                    while (opModeIsActive()) {
                        Ballsack.setPower(-0.35);
                        if (touch.isPressed()) {
                            Ballsack.setPower(0.0);
                            break;
                        }
                    }
                    Ballsack.setPower(0.0);

                    BallLoad.setPosition(0.0);
                    sleep(1250);

                    BallLoad.setPosition(0.8);
                    sleep(700);

                    Ballsack.setPower(-1.0);
                    sleep(115);

                    Ballsack.setPower(0.0);
                    sleep(200);

                    // Full, Blue, Beacons, Center
                    if (ParkCenter) {
                        gyro.resetZAxisIntegrator();
                        angleZ = gyro.getIntegratedZValue();
                        sleep(250);

                        DriveMotor1.setPower(-0.225);
                        DriveMotor2.setPower(-0.225);
                        DriveMotor3.setPower(0.225);
                        DriveMotor4.setPower(0.225);

                        while (1 == 1) {
                            telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
                            telemetry.update();

                            if (angleZ >= 56) {
                                telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
                                telemetry.update();

                                DriveForward(0);
                                break;
                            }

                            angleZ = gyro.getIntegratedZValue();
                        }
                        DriveForward(0);
                        sleep(250);

                        DriveInches(0.3, 35, false);
                        DriveForward(0);
                    }
                    // Full, Blue, Beacons, Corner
                    else {
                        gyro.resetZAxisIntegrator();
                        angleZ = gyro.getIntegratedZValue();
                        sleep(250);

                        DriveMotor1.setPower(0.225);
                        DriveMotor2.setPower(0.225);
                        DriveMotor3.setPower(-0.225);
                        DriveMotor4.setPower(-0.225);

                        while (1 == 1) {
                            telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
                            telemetry.update();

                            if (angleZ <= -19) {
                                telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
                                telemetry.update();

                                DriveForward(0);
                                break;
                            }

                            angleZ = gyro.getIntegratedZValue();
                        }
                        DriveForward(0);
                        sleep(250);

                        DriveInches(0.3, 58, true);
                        DriveForward(0);
                    }
                }
                // Full, Blue, Balls
                else {
                    gyro.resetZAxisIntegrator();

                    angleZ = gyro.getIntegratedZValue();

                    DriveInches(0.2, 29, false);

                    DriveForward(0);
                    sleep(500);

                    Ballsack.setPower(-1.0);
                    sleep(115);

                    Ballsack.setPower(0.0);
                    sleep(1000);

                    while (opModeIsActive()) {
                        Ballsack.setPower(-0.35);
                        if (touch.isPressed()) {
                            Ballsack.setPower(0.0);
                            break;
                        }
                    }
                    Ballsack.setPower(0.0);

                    BallLoad.setPosition(0.0);
                    sleep(1250);

                    BallLoad.setPosition(0.8);
                    sleep(700);

                    Ballsack.setPower(-1.0);
                    sleep(115);

                    Ballsack.setPower(0.0);
                    sleep(200);

                    DriveInches(0.2, 33, false);
                    DriveForward(0);
                    sleep(500);

                    angleZ = gyro.getIntegratedZValue();

                    DriveMotor1.setPower(-0.2);
                    DriveMotor2.setPower(-0.2);
                    DriveMotor3.setPower(0.2);
                    DriveMotor4.setPower(0.2);

                    while (1 == 1) {
                        telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
                        telemetry.update();

                        if (angleZ >= 14) {
                            telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
                            telemetry.update();

                            DriveForward(0);
                            break;
                        }

                        angleZ = gyro.getIntegratedZValue();
                    }
                    DriveForward(0);
                    sleep(250);

                    DriveInches(0.2, 27, false);

                    while (opModeIsActive()) {
                        DriveMotor1.setPower(-0.11);
                        DriveMotor2.setPower(-0.11);
                        DriveMotor3.setPower(-0.16);
                        DriveMotor4.setPower(-0.16);

                        color.enableLed(false);

                        telemetry.addData("Red   ", color.red());
                        telemetry.addData("Blue  ", color.blue());
                        telemetry.addData("Green ", color.green());
                        telemetry.update();

                        if (color.blue() >= 3) {
                            DriveForward(0);
                            break;
                        }


                        double RGBaverage = ((color.red() + color.blue() + color.green()) / 3);
                        telemetry.addData("Final Value", RGBaverage);
                    }
                    DriveForward(0);
                    sleep(250);

                    ButtonPresser.setPosition(0.7);
                    sleep(500);
                    ButtonPresser.setPosition(1.0);

                    color.enableLed(true);
                    DriveMotor1.setPower(0.17);
                    DriveMotor2.setPower(0.17);
                    DriveMotor3.setPower(0.22);
                    DriveMotor4.setPower(0.22);
                    sleep(1500);

                    while (opModeIsActive()) {
                        DriveMotor1.setPower(0.11);
                        DriveMotor2.setPower(0.11);
                        DriveMotor3.setPower(0.16);
                        DriveMotor4.setPower(0.16);

                        color.enableLed(false);

                        telemetry.addData("Red   ", color.red());
                        telemetry.addData("Blue  ", color.blue());
                        telemetry.addData("Green ", color.green());
                        telemetry.update();

                        if (color.blue() >= 3) {
                            DriveForward(0);
                            break;
                        }

                        double RGBaverage = ((color.red() + color.blue() + color.green()) / 3);
                        telemetry.addData("Final Value", RGBaverage);
                    }
                    DriveForward(0);
                    sleep(250);

                    ButtonPresser.setPosition(0.7);
                    sleep(500);
                    ButtonPresser.setPosition(1.0);

                    color.enableLed(false);

                    gyro.resetZAxisIntegrator();
                    sleep(150);

                    color.enableLed(true);
                    sleep(250);

                    DriveForward(-0.245);

                    while (opModeIsActive()) {

                        sleep(50);
                        telemetry.addData("Red   ", color.red());
                        telemetry.addData("Blue  ", color.blue());
                        telemetry.addData("Green ", color.green());
                        telemetry.update();

                        if (color.red() <= 1 && color.blue() <= 1) {
                            DriveForward(0);
                            break;
                        }
                    }
                    DriveForward(0);
                    sleep(500);

                    color.enableLed(false);

                    // Full, Blue, Balls, Center
                    if (ParkCenter) {
                        DriveInchesTurn(0.3, 0.2, 36, false);

                        DriveForward(0);

                        gyro.resetZAxisIntegrator();

                        angleZ = gyro.getIntegratedZValue();
                        sleep(250);

                        DriveMotor1.setPower(-0.225);
                        DriveMotor2.setPower(-0.225);
                        DriveMotor3.setPower(0.225);
                        DriveMotor4.setPower(0.225);

                        while (1 == 1) {
                            telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
                            telemetry.update();

                            if (angleZ >= 54) {
                                telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
                                telemetry.update();

                                DriveForward(0);
                                break;
                            }

                            angleZ = gyro.getIntegratedZValue();
                        }
                        DriveForward(0);
                        sleep(250);

                        DriveInches(0.3, 45, false);
                    }

                    // Full, Blue, Balls, Corner
                    else {
                        gyro.resetZAxisIntegrator();
                        sleep(150);

                        angleZ = gyro.getIntegratedZValue();

                        DriveMotor1.setPower(0.55);
                        DriveMotor2.setPower(0.55);
                        DriveMotor3.setPower(-0.55);
                        DriveMotor4.setPower(-0.55);

                        while (1 == 1) {
                            telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
                            telemetry.update();
                            if (angleZ <= -7) {
                                telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
                                telemetry.update();
                                sleep(100);

                                DriveForward(0);
                                break;
                            }

                            angleZ = gyro.getIntegratedZValue();
                        }
                        DriveForward(0);

                        DriveInches(0.3, 36, true);
                    }
                }
            }
        }
    }

    public void DriveForward(double power) {
        DriveMotor2.setPower(power);
        DriveMotor3.setPower(power);
        DriveMotor1.setPower(power);
        DriveMotor4.setPower(power);
    }

    public void DriveInchesTurn(double LeftPower, double RightPower, int inches, boolean fw) {

        DriveMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveMotor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveMotor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(100);

        // Reset the encoders
        DriveMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // 1 wheel rotation = 1120
        // 1 inch works out to be 88.1511213 ticks

        double ticks = inches * 88.1511213;

        // Set target position

        if (fw)
            DriveMotor1.setTargetPosition((int) (ticks));
        else
            DriveMotor1.setTargetPosition((int) (-ticks));

        // Set it to run to target position
        DriveMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // DriveMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start driving the robot
        if (fw)
        {
            DriveMotor1.setPower(LeftPower);
            DriveMotor2.setPower(LeftPower);
            DriveMotor3.setPower(RightPower);
            DriveMotor4.setPower(RightPower);
        }
        else
        {
            DriveMotor1.setPower(-LeftPower);
            DriveMotor2.setPower(-LeftPower);
            DriveMotor3.setPower(-RightPower);
            DriveMotor4.setPower(-RightPower);
        }

        while (DriveMotor1.isBusy()) {
            // Do nothing until it reaches the target position
            telemetry.addData("Encoder1: ", DriveMotor1.getCurrentPosition());
            telemetry.addData("Gyro Angle: ", gyro.getIntegratedZValue());
            telemetry.update();
        }

        // Stop the robot
        DriveForward(0);

        DriveMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveMotor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void DriveInches(double power, int inches, boolean fw) {

        DriveMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveMotor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveMotor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Reset the encoders
        DriveMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // 1 wheel rotation = 1120
        // 1 inch works out to be 88.1511213 ticks

        double ticks = inches * 88.1511213;

        // Set target position

        if (fw)
            DriveMotor1.setTargetPosition((int) ticks);
        else
            DriveMotor1.setTargetPosition((int) -ticks);

        // Set it to run to target position
        DriveMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // DriveMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start driving the robot
        if (fw)
            DriveForward(power);
        else
            DriveForward(-power);

        while (DriveMotor1.isBusy()) {
            // Do nothing until it reaches the target position
            telemetry.addData("Encoder1: ", DriveMotor1.getCurrentPosition());
            // telemetry.addData("Encoder2: ", DriveMotor2.getCurrentPosition());
            telemetry.update();
        }

        // Stop the robot
        DriveForward(0);

        DriveMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveMotor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void TurnOnEncoders() {
        DriveMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveMotor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveMotor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void TurnOffEncoders() {
        DriveMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveMotor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
