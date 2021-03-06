package org.firstinspires.ftc.robotcontroller.external.samples;
import android.app.Activity;
import android.view.View;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by NYAN on 3/10/2016.
 */

public class AutoRed extends LinearOpMode {

    double buttonPos1 = 0.8;
    double buttonPos2 = 0.0;

    ColorSensor color;
    DeviceInterfaceModule cdim;
    DcMotor DriveMotor1;
    DcMotor DriveMotor2;
    DcMotor Ballsack;

    Servo ButtonClick;
    Servo BallLoad;
    Servo arm;
    Servo lock;

    ModernRoboticsI2cGyro gyro;
    TouchSensor touch;

    int heading = 0;
    int angleZ = 0;
    boolean lastResetState = false;
    boolean curResetState  = false;

    double P_RGBaverage = 99999;

    public void runOpMode() throws InterruptedException {
        hardwareMap.logDevices();

        boolean fail = false;
        int count = 0;
        int prevY = 0;
        int y = 0;
        int x = 0;
        int diff;
        int rcnt = 0;

        //Red == 0; Blue == 1;
        int teamType = 0;

        DriveMotor1 = hardwareMap.dcMotor.get("left");
        DriveMotor2 = hardwareMap.dcMotor.get("right");
        Ballsack = hardwareMap.dcMotor.get("ballsack");

        ButtonClick = hardwareMap.servo.get("servo2");
        BallLoad = hardwareMap.servo.get("servo1");
        arm = hardwareMap.servo.get("arm");
        lock = hardwareMap.servo.get("lock");

        BallLoad.setPosition(0.23);
        arm.setPosition(1.0);
        lock.setPosition(0.6);

        ButtonClick.setPosition(buttonPos1);

        cdim = hardwareMap.deviceInterfaceModule.get("dim");

        color = hardwareMap.colorSensor.get("color");

        touch = hardwareMap.touchSensor.get("touch");

        boolean bEnabled = true;

        color.enableLed(false);

        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        // start calibrating the gyro.
        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        gyro.calibrate();

        // make sure the gyro is calibrated.
        while (!isStopRequested() && gyro.isCalibrating())  {
            sleep(50);
            idle();
        }

        waitForStart();

        // Initialize color sensor values
        float hsvValues[] = {0F, 0F, 0F};

        final float values[] = hsvValues;

        // Setup coloring for screen so screen color changes accordingly
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(R.id.RelativeLayout);

        boolean bPrevState = false;
        boolean bCurrState = false;

        //DriveMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //DriveMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm.setPosition(1.0);
        lock.setPosition(0.6);

        if(teamType == 0) {
            DriveMotor1.setPower(0.25);
            DriveMotor2.setPower(0.20);
            sleep(4500);

            while (opModeIsActive()) {
                Ballsack.setPower(0.0);
                DriveMotor1.setPower(0.25);
                DriveMotor2.setPower(0.20);

                color.enableLed(false);

                telemetry.addData("Red   ", color.red());
                telemetry.addData("Blue  ", color.blue());
                telemetry.addData("Green ", color.green());
                telemetry.update();

                if (teamType == 0) {
                    if (color.red() >= 3) {
                        DriveMotor1.setPower(0.0);
                        DriveMotor2.setPower(0.0);
                        break;
                    }


                    double RGBaverage = ((color.red() + color.blue() + color.green()) / 3);
                    telemetry.addData("Final Value", RGBaverage);

                }
            }
        }
        else
        {
            DriveMotor1.setPower(-0.25);
            DriveMotor2.setPower(-0.18);
            sleep(6000);

            while (opModeIsActive()) {
                DriveMotor1.setPower(-0.15);
                DriveMotor2.setPower(-0.15);

                color.enableLed(false);

                telemetry.addData("Red   ", color.red());
                telemetry.addData("Blue  ", color.blue());
                telemetry.addData("Green ", color.green());
                telemetry.update();

                if (color.blue() >= 3) {
                    //DriveMotor1.setPower(0.1);
                    //DriveMotor2.setPower(0.1);
                    //sleep(500);
                    break;
                }

                double RGBaverage = ((color.red() + color.blue() + color.green()) / 3);
                telemetry.addData("Final Value", RGBaverage);
            }
        }
        DriveMotor1.setPower(0.0);
        DriveMotor2.setPower(0.0);
        sleep(500);
        ButtonClick.setPosition(buttonPos2);
        sleep(500);
        ButtonClick.setPosition(buttonPos1);
        sleep(500);

        if(teamType == 0)
        {
            DriveMotor1.setPower(-0.16);
            DriveMotor2.setPower(-0.42);
            sleep(2000);

            while (opModeIsActive()) {
                DriveMotor1.setPower(-0.16);
                DriveMotor2.setPower(-0.21);

                color.enableLed(false);

                telemetry.addData("Red   ", color.red());
                telemetry.addData("Blue  ", color.blue());
                telemetry.addData("Green ", color.green());
                telemetry.update();

                if (color.red() >= 3) {
                    DriveMotor1.setPower(0.0);
                    DriveMotor2.setPower(0.0);
                    break;
                }

                double RGBaverage = ((color.red() + color.blue() + color.green()) / 3);
                telemetry.addData("Final Value", RGBaverage);
            }
        }
        else
        {
            DriveMotor1.setPower(0.11);
            DriveMotor2.setPower(0.25);
            sleep(2000);

            while (opModeIsActive()) {
                DriveMotor1.setPower(0.09);
                DriveMotor2.setPower(0.19);

                color.enableLed(false);

                telemetry.addData("Red   ", color.red());
                telemetry.addData("Blue  ", color.blue());
                telemetry.addData("Green ", color.green());
                telemetry.update();

                if (color.blue() >= 3) {

                    DriveMotor1.setPower(0.0);
                    DriveMotor2.setPower(0.0);
                    break;
                }

                double RGBaverage = ((color.red() + color.blue() + color.green()) / 3);
                telemetry.addData("Final Value", RGBaverage);
            }
        }
        DriveMotor1.setPower(0.0);
        DriveMotor2.setPower(0.0);
        sleep(500);
        ButtonClick.setPosition(buttonPos2);
        sleep(500);
        ButtonClick.setPosition(buttonPos1);
        sleep(500);

        DriveMotor1.setPower(0.0);
        DriveMotor2.setPower(0.0);
        sleep(500);

        color.enableLed(true);

        while (opModeIsActive())
        {
            Ballsack.setPower(0.0);
            if(teamType == 0)
            {
                DriveMotor1.setPower(0.19);
                DriveMotor2.setPower(0.14);
                sleep(500);
                //DriveMotor1.setPower(0.0);
                //DriveMotor2.setPower(0.0);
                //sleep(1000);
                color.enableLed(true);

                DriveMotor1.setPower(0.25);
                DriveMotor2.setPower(0.20);

                telemetry.addData("Red   ", color.red());
                telemetry.addData("Blue  ", color.blue());
                telemetry.addData("Green ", color.green());
                telemetry.update();

                if (color.red() <= 1 && color.blue() <= 1)
                {
                    DriveMotor1.setPower(0.0);
                    DriveMotor2.setPower(0.0);
                    break;
                }
            }
            else
            {
                DriveMotor1.setPower(0.14);
                DriveMotor2.setPower(0.19);
                sleep(500);

                DriveMotor1.setPower(0.14);
                DriveMotor2.setPower(0.19);

                color.enableLed(true);

                telemetry.addData("Red   ", color.red());
                telemetry.addData("Blue  ", color.blue());
                telemetry.addData("Green ", color.green());
                telemetry.update();

                if (color.red() <= 1 && color.blue() <= 1)
                {
                    DriveMotor1.setPower(0.0);
                    DriveMotor2.setPower(0.0);
                    break;
                }
            }


            double RGBaverage = ((color.red() + color.blue() + color.green()) / 3);
            telemetry.addData("Final Value", RGBaverage);
        }
        DriveMotor1.setPower(0.0);
        DriveMotor2.setPower(0.0);
        //telemetry.addData("0", "GOT HERE 1");
        //telemetry.update();
        // sleep(5000);

        gyro.resetZAxisIntegrator();

        if(teamType == 0)
        {
            DriveMotor1.setPower(-0.45);
        }
        else
        {
            DriveMotor1.setPower(-0.35);
            DriveMotor2.setPower(0.35);
        }
        int cnt=0;
        
        while(opModeIsActive())
        {
            telemetry.addData("0", "angleZ=%03d",angleZ);
            //telemetry.addData("1", " cnt=%03d", cnt);
            //telemetry.addData("2", "GOT HERE 2");
            //telemetry.addData("1", "Int. Ang. %03d", angleZ);
            telemetry.update();
            if(teamType == 0)
            {
                if(angleZ >= 55)
                {
                    DriveMotor1.setPower(0.0);
                    break;
                }
            }
            else
            {
                if(angleZ >= 36)
                {
                    DriveMotor1.setPower(0.0);
                    break;
                }
            }

            angleZ  = gyro.getIntegratedZValue();
        }

        DriveMotor1.setPower(0.0);
        DriveMotor2.setPower(0.0);
        sleep(500);

        color.enableLed(false);


        if(teamType == 0)
        {
            DriveMotor1.setPower(-0.25);
            DriveMotor2.setPower(-0.25);
        }
        else
        {
            DriveMotor1.setPower(-0.25);
            DriveMotor2.setPower(-0.25);
        }
        sleep(2100);

        DriveMotor1.setPower(0.0);
        DriveMotor2.setPower(0.0);
        sleep(1000);

        /*
        while(opModeIsActive())
        {
            Ballsack.setPower(-0.35);
            if (!touch.isPressed()) {
                Ballsack.setPower(0.0);
                break;
            }
        }*/
        Ballsack.setPower(-1.0);
        sleep(115);

        Ballsack.setPower(0.0);
        sleep(1000);

        while(opModeIsActive())
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
        BallLoad.setPosition(0.23);
        sleep(700);

        /*while(opModeIsActive())
        {
            Ballsack.setPower(-0.28);
            if (!touch.isPressed()) {
                Ballsack.setPower(0.0);
                break;
            }
        }*/
        Ballsack.setPower(-1.0);
        sleep(115);

        Ballsack.setPower(0.0);
        sleep(200);
        if(teamType == 0)
        {
            DriveMotor1.setPower(-1.0);
            DriveMotor2.setPower(-0.02);
            sleep(2000);
        }
        else
        {
            DriveMotor1.setPower(-0.80);
            DriveMotor2.setPower(-0.06);
        }


    }

}
