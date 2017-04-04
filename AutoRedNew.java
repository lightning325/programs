package org.firstinspires.ftc.robotcontroller.external.samples;
import android.app.Activity;
import android.view.View;
import android.widget.Button;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Created by NYAN on 3/10/2016.
 */

public class AutoRedNew extends LinearOpMode {

    DeviceInterfaceModule cdim;
    DcMotor DriveMotor1;
    DcMotor DriveMotor2;
    DcMotor DriveMotor3;
    DcMotor DriveMotor4;

    Servo ButtonPresser;

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

        ButtonPresser = hardwareMap.servo.get("servobp");

        DriveMotor1.setDirection(DcMotor.Direction.REVERSE);
        DriveMotor2.setDirection(DcMotor.Direction.REVERSE);

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

        color.enableLed(false);

        waitForStart();

        color.enableLed(false);

        gyro.resetZAxisIntegrator();

        DriveMotor3.setPower(0.3);
        DriveMotor4.setPower(0.3);

        while (1 == 1) {
            if (angleZ >= 28) {
                telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
                telemetry.update();

                DriveMotor1.setPower(0.0);
                DriveMotor2.setPower(0.0);
                break;
            }

            angleZ = gyro.getIntegratedZValue();
        }
        DriveMotor3.setPower(0.0);
        DriveMotor4.setPower(0.0);

        sleep(500);

        DriveInches(0.4, 66, true);

        sleep(1000);

        DriveMotor3.setPower(-0.3);
        DriveMotor4.setPower(-0.3);

        while (true) {
            if (angleZ <= 24) {
                telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
                telemetry.update();

                DriveMotor3.setPower(0.0);
                DriveMotor4.setPower(0.0);
                break;
            }

            angleZ = gyro.getIntegratedZValue();
        }
        DriveMotor3.setPower(0.0);
        DriveMotor4.setPower(0.0);
        sleep(1000);

        DriveInches(0.3, 5, true);
        sleep(250);

        DriveMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveMotor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive()) {
            DriveMotor1.setPower(0.15);
            DriveMotor2.setPower(0.15);
            DriveMotor3.setPower(0.20);
            DriveMotor4.setPower(0.20);

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

        ButtonPresser.setPosition(0.7);
        sleep(500);
        ButtonPresser.setPosition(1.0);

        color.enableLed(true);
        DriveForward(-0.2);
        sleep(1500);

        while (opModeIsActive()) {
            DriveMotor1.setPower(-0.15);
            DriveMotor2.setPower(-0.15);
            DriveMotor3.setPower(-0.20);
            DriveMotor4.setPower(-0.20);

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

        ButtonPresser.setPosition(0.7);
        sleep(500);
        ButtonPresser.setPosition(1.0);

        while (opModeIsActive()) {

            DriveForward(0.2);
            sleep(500);
            //DriveMotor1.setPower(0.0);
            //DriveMotor2.setPower(0.0);
            //sleep(1000);
            color.enableLed(true);

            DriveMotor1.setPower(0.15);
            DriveMotor2.setPower(0.15);
            DriveMotor3.setPower(0.20);
            DriveMotor4.setPower(0.20);

            telemetry.addData("Red   ", color.red());
            telemetry.addData("Blue  ", color.blue());
            telemetry.addData("Green ", color.green());
            telemetry.update();

            if (color.red() <= 1 && color.blue() <= 1) {
                DriveForward(0);
                break;
            }
        }
        gyro.resetZAxisIntegrator();

        DriveMotor1.setPower(-0.3);
        DriveMotor2.setPower(-0.3);

        while (1 == 1) {
            if (angleZ >= 28) {
                telemetry.addData("Gyro: ", gyro.getIntegratedZValue());
                telemetry.update();

                DriveMotor1.setPower(0.0);
                DriveMotor2.setPower(0.0);
                break;
            }

            angleZ = gyro.getIntegratedZValue();
        }
        DriveMotor1.setPower(0.0);
        DriveMotor2.setPower(0.0);
        
    }

    public void DriveForward(double power) {
        DriveMotor2.setPower(power);
        DriveMotor3.setPower(power);
        DriveMotor1.setPower(power);
        DriveMotor4.setPower(power);
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
    }

    public void GyroTurn(double power,int degrees, boolean direction) {
        gyro.resetZAxisIntegrator();
        sleep(50);
        angleZ = gyro.getIntegratedZValue();

        //true = right
        if(direction){

            DriveMotor1.setPower(0.0);
            DriveMotor2.setPower(0.0);
            DriveMotor3.setPower(power);
            DriveMotor4.setPower(power);
            while(angleZ < degrees)
            {
                sleep(5);
                angleZ = gyro.getIntegratedZValue();
            }
        }
        else
        {
            DriveMotor1.setPower(power);
            DriveMotor2.setPower(power);
            DriveMotor3.setPower(0.0);
            DriveMotor4.setPower(0.0);
            while(angleZ > degrees)
                    {
                sleep(5);
                angleZ = gyro.getIntegratedZValue();
            }
        }
        DriveMotor1.setPower(0.0);
        DriveMotor2.setPower(0.0);
        DriveMotor3.setPower(0.0);
        DriveMotor4.setPower(0.0);
    }

    public boolean ColorSensorThreshold(int switchcolor, int threshold){
        // red=1 green=2 blue=3
        switch(switchcolor){
            case 1:
                if(color.red() >= threshold){
                    return true;
                }else{return false;}
            case 2:
                if(color.green() >= threshold){
                    return true;
                }else{return false;}
            case 3:
                if(color.blue() >= threshold){
                    return true;
                }else{return false;}
            default: return false;
        }

    }
}