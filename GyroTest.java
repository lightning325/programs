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

public class GyroTest extends LinearOpMode {

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

        waitForStart();

        gyro.resetZAxisIntegrator();
        GyroTurn(.2, 90, true);
        sleep(500);
        GyroTurn(.2, 45, false);
        GyroTurn(.3, 45, false);
        sleep(500);
        GyroTurn(.1, 350, true);
        GyroTurn(.1, 350, true);
        GyroTurn(.1, 20, true);
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
            while(angleZ > degrees)
            {
                sleep(5);
                angleZ = gyro.getIntegratedZValue();
                telemetry.addData("Angle: ", angleZ);
            }
        }
        else
        {
            DriveMotor1.setPower(power);
            DriveMotor2.setPower(power);
            DriveMotor3.setPower(0.0);
            DriveMotor4.setPower(0.0);
            while(angleZ < degrees)
            {
                sleep(5);
                angleZ = gyro.getIntegratedZValue();
                telemetry.addData("Angle: ", angleZ);
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
                if(color.red() >= threshold)
                    return true;
                else
                    return false;
            case 2:
                if(color.green() >= threshold)
                    return true;
                else
                    return false;
            case 3:
                if(color.blue() >= threshold)
                    return true;
                else
                    return false;
            default: return false;
        }

    }
}