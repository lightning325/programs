package org.firstinspires.ftc.robotcontroller.external.samples;
import android.app.Activity;
import android.view.View;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.ftcrobotcontroller.R;
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
public class AutoBallLaunch extends LinearOpMode {

    double buttonPos1 = 0.8;
    double buttonPos2 = 0.0;

    DeviceInterfaceModule cdim;
    DcMotor Ballsack;


    TouchSensor touch;

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

        Ballsack = hardwareMap.dcMotor.get("ballsack");

        touch = hardwareMap.touchSensor.get("touch");

        telemetry.addData("Touch Pressed:", touch.isPressed());

        cdim = hardwareMap.deviceInterfaceModule.get("dim");

        waitForStart();

        while(opModeIsActive())
        {
            Ballsack.setPower(-0.35);
            if(touch.isPressed())
            {
                Ballsack.setPower(0.0);
                break;
            }
        }
        Ballsack.setPower(0.0);

    }
}
