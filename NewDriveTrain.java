/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import android.graphics.Paint;
import android.media.SoundPool;
import android.media.AudioManager;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */

@TeleOp(name="NewDriveTrain")
public class NewDriveTrain extends OpMode {

    DcMotor DriveMotor1;
    DcMotor DriveMotor2;
    DcMotor DriveMotor3;
    DcMotor DriveMotor4;
    DcMotor LaunchMotor;
    DcMotor IntakeMotor;
    DcMotor   LiftMotor;

    Servo ButtonPresser;
    Servo    BallLoader;
    Servo        CapArm;
    Servo       CapLock;

    TouchSensor touch;

    double LeftDrive1   =   0.0;
    double LeftDrive2   =   0.0;
    double RightDrive1  =   0.0;
    double RightDrive2  =   0.0;
    double LaunchDrive  =   0.0;
    double IntakeDrive  =   0.0;
    double LiftDrive    =   0.0;

    int    endToggle;
    boolean runOnceA = false;
    boolean runOnceB = false;

    public NewDriveTrain()
    {
    }

    @Override
    public void init()
    {
        // Hard code the motors to a string value for the config
        DriveMotor1   =    hardwareMap.dcMotor.get("left1");
        DriveMotor2   =    hardwareMap.dcMotor.get("left2");
        DriveMotor3   =   hardwareMap.dcMotor.get("right1");
        DriveMotor4   =   hardwareMap.dcMotor.get("right2");
        LaunchMotor   = hardwareMap.dcMotor.get("ballsack");
        IntakeMotor   =   hardwareMap.dcMotor.get("intake");
        LiftMotor     =     hardwareMap.dcMotor.get("lift");

        ButtonPresser =    hardwareMap.servo.get("servobp");
        BallLoader    =     hardwareMap.servo.get("loader");
        CapArm        =        hardwareMap.servo.get("arm");
        CapLock       =       hardwareMap.servo.get("lock");

        touch         =hardwareMap.touchSensor.get("touch");

        DriveMotor1.setDirection(DcMotor.Direction.REVERSE);
        DriveMotor2.setDirection(DcMotor.Direction.REVERSE);

        ButtonPresser.setPosition(1.0);
        CapArm.setPosition(1.0);
        CapLock.setPosition(0.0);

        endToggle = 0;


        DriveMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        DriveMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        DriveMotor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        DriveMotor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }


    @Override
    public void loop()
    {
        if(gamepad1.start && gamepad1.left_bumper)
        {
            endToggle = 0;
            telemetry.addData("Toggle: ", endToggle);
            telemetry.addData("LBumper: ", gamepad1.left_bumper);
            telemetry.addData("Start: ", gamepad1.start);
            telemetry.update();
        }

        if(gamepad1.start && gamepad1.right_bumper)
        {
            endToggle = 1;
            telemetry.addData("Toggle: ", endToggle);
            telemetry.addData("RBumper: ", gamepad1.right_bumper);
            telemetry.addData("Start: ", gamepad1.start);
            telemetry.update();
        }

        telemetry.addData("LBuumper", gamepad1.left_bumper);
        telemetry.addData("RBumper: ", gamepad1.right_bumper);
        telemetry.addData("Start: ", gamepad1.start);
        telemetry.update();


        // REGULAR MODE

        if(endToggle == 0) {
            CapArm.setPosition(1);
            if(gamepad1.start && gamepad1.left_bumper)
            {
                endToggle = 0;
                telemetry.addData("Toggle: ", endToggle);
                telemetry.addData("LBumper: ", gamepad1.left_bumper);
                telemetry.addData("Start: ", gamepad1.start);
                telemetry.update();
            }

            if(gamepad1.start && gamepad1.right_bumper)
            {
                endToggle = 1;
                telemetry.addData("Toggle: ", endToggle);
                telemetry.addData("RBumper: ", gamepad1.right_bumper);
                telemetry.addData("Start: ", gamepad1.start);
                telemetry.update();
            }

            telemetry.addData("LBuumper", gamepad1.left_bumper);
            telemetry.addData("RBumper: ", gamepad1.right_bumper);
            telemetry.addData("Start: ", gamepad1.start);
            telemetry.update();



            LaunchDrive = gamepad2.right_stick_y;
            IntakeDrive = gamepad2.left_stick_y;

            DriveMotor1.setPower(LeftDrive1);
            DriveMotor2.setPower(LeftDrive2);
            DriveMotor3.setPower(RightDrive1);
            DriveMotor4.setPower(RightDrive2);
            LaunchMotor.setPower(LaunchDrive);
            IntakeMotor.setPower(IntakeDrive);

            while (gamepad1.dpad_up) {
                DriveMotor1.setPower(-0.2);
                DriveMotor2.setPower(-0.2);
                DriveMotor3.setPower(-0.2);
                DriveMotor4.setPower(-0.2);
            }

            while (gamepad1.dpad_down) {
                DriveMotor1.setPower(0.2);
                DriveMotor2.setPower(0.2);
                DriveMotor3.setPower(0.2);
                DriveMotor4.setPower(0.2);
            }

            if (gamepad1.right_bumper)
            {
                DriveMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                DriveMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                DriveMotor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                DriveMotor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                LeftDrive1 = gamepad1.left_stick_y;
                LeftDrive2 = gamepad1.left_stick_y;
                RightDrive1 = gamepad1.right_stick_y;
                RightDrive2 = gamepad1.right_stick_y;
            }
            else
            {
                DriveMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                DriveMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                DriveMotor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                DriveMotor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                LeftDrive1 = gamepad1.left_stick_y / 2;
                LeftDrive2 = gamepad1.left_stick_y / 2;
                RightDrive1 = gamepad1.right_stick_y / 2;
                RightDrive2 = gamepad1.right_stick_y / 2;
            }

            if (gamepad1.a) {
                ButtonPresser.setPosition(0.6);
            } else {
                ButtonPresser.setPosition(1.0);
            }

            if (gamepad2.x) {
                BallLoader.setPosition(0.5);
            } else {
                BallLoader.setPosition(0.8);
            }

            if (gamepad2.y) {
                while (1 == 1) {
                    LaunchMotor.setPower(-0.40);
                    if (touch.isPressed() || gamepad2.dpad_up) {
                        LaunchMotor.setPower(0.0);
                        break;
                    }
                }
                LaunchMotor.setPower(0.0);
            }

            telemetry.addData("Left: ", "" + String.format("%.2f", LeftDrive1));
            telemetry.addData("Right: ", "" + String.format("%.2f", RightDrive1));
            telemetry.addData("Launcher: ", "" + String.format("%.2f", LaunchDrive));
            telemetry.addData("Intake: ", "" + String.format("%.2f", IntakeDrive));
            telemetry.update();
        }


        // LIFT MODE
        else
        {
            if(runOnceA == false){
                CapLock.setPosition(0.4);
                runOnceA = true;
            }
            if(gamepad1.start && gamepad1.left_bumper)
            {
                endToggle = 0;
                telemetry.addData("Toggle: ", endToggle);
                telemetry.addData("LBumper: ", gamepad1.left_bumper);
                telemetry.addData("Start: ", gamepad1.start);
                telemetry.update();
            }

            if(gamepad1.start && gamepad1.right_bumper)
            {
                endToggle = 1;
                telemetry.addData("Toggle: ", endToggle);
                telemetry.addData("RBumper: ", gamepad1.right_bumper);
                telemetry.addData("Start: ", gamepad1.start);
                telemetry.update();
            }

            telemetry.addData("LBuumper", gamepad1.left_bumper);
            telemetry.addData("RBumper: ", gamepad1.right_bumper);
            telemetry.addData("Start: ", gamepad1.start);
            telemetry.update();


            DriveMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            DriveMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            DriveMotor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            DriveMotor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            //switched
            if (gamepad1.right_bumper) {
                LeftDrive1 = gamepad1.right_stick_y / 2;
                LeftDrive2 = gamepad1.right_stick_y / 2;
                RightDrive1 = gamepad1.left_stick_y / 2;
                RightDrive2 = gamepad1.left_stick_y / 2;
            } else {
                LeftDrive1 = gamepad1.right_stick_y / 4;
                LeftDrive2 = gamepad1.right_stick_y / 4;
                RightDrive1 = gamepad1.left_stick_y / 4;
                RightDrive2 = gamepad1.left_stick_y / 4;
            }

            while (gamepad1.dpad_up) {
                DriveMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                DriveMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                DriveMotor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                DriveMotor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                DriveMotor1.setPower(0.125);
                DriveMotor2.setPower(0.125);
                DriveMotor3.setPower(0.125);
                DriveMotor4.setPower(0.125);
            }

            while (gamepad1.dpad_down) {
                DriveMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                DriveMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                DriveMotor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                DriveMotor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                DriveMotor1.setPower(-0.125);
                DriveMotor2.setPower(-0.125);
                DriveMotor3.setPower(-0.125);
                DriveMotor4.setPower(-0.125);
            }

            LiftDrive = gamepad2.right_stick_y;

            DriveMotor1.setPower(-LeftDrive1);
            DriveMotor2.setPower(-LeftDrive2);
            DriveMotor3.setPower(-RightDrive1);
            DriveMotor4.setPower(-RightDrive2);
            LiftMotor.setPower(LiftDrive);

            if(gamepad2.x)
            {
                CapArm.setPosition(0.0);
                telemetry.addData("Arm: ", "" + String.format("%.2f", CapArm.getPosition()));
                telemetry.update();
            }
            else
            {
                CapArm.setPosition(1.0);
                telemetry.addData("Arm: ", "" + String.format("%.2f", CapArm.getPosition()));
                telemetry.update();
            }
        }
    }

    /*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
    @Override
    public void stop() {

        DriveMotor1.setPower(0.0);
        DriveMotor2.setPower(0.0);
        DriveMotor3.setPower(0.0);
        DriveMotor4.setPower(0.0);
        LaunchMotor.setPower(0.0);
        IntakeMotor.setPower(0.0);

    }


    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of `array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }

}
