package org.firstinspires.ftc.robotcontroller.external.samples;
import android.app.Activity;
import android.view.View;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by NYAN on 3/10/2016.
 */
public class DriveTest extends LinearOpMode {

    DcMotor DriveMotor1;
    DcMotor DriveMotor2;
    DcMotor DriveMotor3;
    DcMotor DriveMotor4;

    public void runOpMode() throws InterruptedException {
        hardwareMap.logDevices();

        DriveMotor1 = hardwareMap.dcMotor.get("left1");
        DriveMotor2 = hardwareMap.dcMotor.get("left2");
        DriveMotor3 = hardwareMap.dcMotor.get("right1");
        DriveMotor4 = hardwareMap.dcMotor.get("right2");

        DriveMotor3.setDirection(DcMotorSimple.Direction.REVERSE);
        DriveMotor4.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        DriveForward(0.2);
        sleep(3000);

        DriveForward(0);
        sleep(500);

        DriveForward(-0.2);
        sleep(3000);

    }

    public void DriveForward(double power){
        DriveMotor2.setPower(power);
        DriveMotor3.setPower(power);
        DriveMotor1.setPower(power);
        DriveMotor4.setPower(power);
    }
}