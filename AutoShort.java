package org.firstinspires.ftc.robotcontroller.external.samples;
import android.app.Activity;
import android.view.View;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Created by NYAN on 3/10/2016.
 */

public class AutoShort extends LinearOpMode {

    DeviceInterfaceModule cdim;
    DcMotor DriveMotor1;
    DcMotor DriveMotor2;
    DcMotor Ballsack;

    Servo BallLoad;
    Servo arm;
    Servo lock;

    ModernRoboticsI2cGyro gyro;
    TouchSensor touch;

    int heading = 0;
    int angleZ = 0;
    boolean lastResetState = false;
    boolean curResetState  = false;

    public void runOpMode() throws InterruptedException {

        hardwareMap.logDevices();

        DriveMotor1 = hardwareMap.dcMotor.get("left");
        DriveMotor2 = hardwareMap.dcMotor.get("right");
        Ballsack = hardwareMap.dcMotor.get("ballsack");

        BallLoad = hardwareMap.servo.get("servo1");
        arm = hardwareMap.servo.get("arm");
        lock = hardwareMap.servo.get("lock");

        BallLoad.setPosition(0.23);
        arm.setPosition(1.0);
        lock.setPosition(0.6);

        cdim = hardwareMap.deviceInterfaceModule.get("dim");

        touch = hardwareMap.touchSensor.get("touch");

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

        arm.setPosition(1.0);
        lock.setPosition(0.6);

        waitForStart();

        //DriveMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //DriveMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm.setPosition(1.0);
        lock.setPosition(0.6);

        DriveMotor1.setPower(0.0);
        DriveMotor2.setPower(0.0);
        sleep(10000);

        DriveMotor1.setPower(0.2);
        DriveMotor2.setPower(0.2);
        sleep(3250);

        DriveMotor1.setPower(0.0);
        DriveMotor2.setPower(0.0);
        sleep(500);

        DriveMotor1.setPower(-0.5);
        DriveMotor2.setPower(0.5);
        while(opModeIsActive())
        {
            telemetry.addData("0", "angleZ=%03d",angleZ);
            //telemetry.addData("1", " cnt=%03d", cnt);
            //telemetry.addData("2", "GOT HERE 2");
            //telemetry.addData("1", "Int. Ang. %03d", angleZ);
            telemetry.update();

            if(angleZ >= 94)
            {
                DriveMotor1.setPower(0.0);
                DriveMotor2.setPower(0.0);
                break;
            }

            angleZ  = gyro.getIntegratedZValue();
        }
        sleep(500);

        Ballsack.setPower(-1.0);
        sleep(125);
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
        Ballsack.setPower(-1.0);
        sleep(125);
        Ballsack.setPower(0.0);
        sleep(200);



    }

}
