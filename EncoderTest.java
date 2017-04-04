package org.firstinspires.ftc.robotcontroller.external.samples;
import android.app.Activity;
import android.view.View;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import java.util.concurrent.SynchronousQueue;

/**
 * Created by NYAN on 3/10/2016.
 */

@Autonomous(name="EncoderTest")
public class EncoderTest extends LinearOpMode
{
    // ANDYMARK_TICKS_PER_REV = 1120;

    int angleZ = 0;

    DeviceInterfaceModule cdim;

    DcMotor DriveMotor1 = null;
    DcMotor DriveMotor2 = null;
    DcMotor DriveMotor3 = null;
    DcMotor DriveMotor4 = null;

     ModernRoboticsI2cGyro gyro;

    public void runOpMode() throws InterruptedException {
        hardwareMap.logDevices();

        DriveMotor1 = hardwareMap.dcMotor.get("left1");
        DriveMotor2 = hardwareMap.dcMotor.get("left2");
        DriveMotor3 = hardwareMap.dcMotor.get("right1");
        DriveMotor4 = hardwareMap.dcMotor.get("right2");

        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        DriveMotor3.setDirection(DcMotor.Direction.REVERSE);
        DriveMotor4.setDirection(DcMotor.Direction.REVERSE);


        cdim = hardwareMap.deviceInterfaceModule.get("dim");

        // start calibrating the gyro.
        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        gyro.calibrate();

        // make sure the gyro is calibrated.
        while (!isStopRequested() && gyro.isCalibrating()) {
            sleep(50);
            idle();
        }

        waitForStart();

        DriveInches(0.2, 48, true);

        sleep(1500);

        DriveInches(0.2, 48, false);

    }

    public void DriveForward(double power){
        DriveMotor1.setPower(power);
        DriveMotor2.setPower(power);
        DriveMotor3.setPower(power);
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

        sleep(500);

        // Start driving the robot
        if (fw)
            DriveForward(power);
        else
            DriveForward(-power);

        angleZ = gyro.getIntegratedZValue();

        while (DriveMotor1.isBusy() && opModeIsActive()) {
            // Do nothing until it reaches the target position
            telemetry.addData("Encoder1: ", DriveMotor1.getCurrentPosition());
            telemetry.addData("Encoder2: ", DriveMotor2.getCurrentPosition());
            telemetry.addData("Encoder3: ", DriveMotor3.getCurrentPosition());
            telemetry.addData("Encoder4: ", DriveMotor4.getCurrentPosition());
            telemetry.addData("Angle: ", angleZ);
            telemetry.update();

            angleZ = gyro.getIntegratedZValue();
        }

        // Stop the robot

        DriveForward(0);

        DriveMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveMotor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


}