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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import android.media.SoundPool;
import android.media.AudioManager;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */

public class ChainDrive extends OpMode {


    double buttonPos2 = 0.8;
    double buttonPos1 = 0.0;
    double armpos     = 0.0;

    final static double ARM_MIN_RANGE  = 0.0;
    final static double ARM_MAX_RANGE  = 1.0;

    final static double BALL_MIN_RANGE = 0.0;
    final static double BALL_MAX_RANGE = 1.0;

    DcMotor DriveMotor1;
    DcMotor DriveMotor2;
    DcMotor IntakeMotor;
    DcMotor LaunchMotor;
    DcMotor LiftMotor;

    ColorSensor color;
    TouchSensor touch;

    Servo ballLoad;
    Servo ButtonClick;
    Servo lock;
    Servo arm;

    TouchSensor LaunchTouch;

    double LeftDrive   = 0.0;
    double RightDrive  = 0.0;
    double IntakeDrive = 0.0;
    double LaunchDrive = 0.0;
    double LiftDrive   = 0.0;

    double ballLPosition;
    double pballLPosition;

    int endToggle;

    // Sound variables
    public SoundPool mySound;
    public int kobe;
    public int hightide;
    public int penguin1;

    /**
     * Constructor
     *
     */
    public ChainDrive()
    {
    }

    /*
     * Code to run when the op mode is first enabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init()
    {

        mySound = new SoundPool(1, AudioManager.STREAM_MUSIC, 0);
        kobe = mySound.load(hardwareMap.appContext, R.raw.kobe, 1);
        hightide = mySound.load(hardwareMap.appContext, R.raw.hightide, 1);
        penguin1 = mySound.load(hardwareMap.appContext, R.raw.whatisthis, 1);

		/*
		 * Use the hardwareMap to get the dc motors and servos by name. Note
		 * that the names of the devices must match the names used when you
		 * configured your robot and created the configuration file.
		 */

		/*
		 * For the demo Tetrix K9 bot we assume the following,
		 *   There are two motors "motor_1" and "motor_2"
		 *   "motor_1" is on the right side of the bot.
		 *   "motor_2" is on the left side of the bot and reversed.
		 *
		 * We also assume that there are two servos "servo_1" and "servo_6"
		 *    "servo_1" controls the arm joint of the manipulator.
		 *    "servo_6" controls the claw joint of the manipulator.
		 */

        // Hard code the motors to a string value for the config
        DriveMotor1 = hardwareMap.dcMotor.get("left");
        DriveMotor2 = hardwareMap.dcMotor.get("right");
        IntakeMotor = hardwareMap.dcMotor.get("intake");
        LaunchMotor = hardwareMap.dcMotor.get("ballsack");
        LiftMotor = hardwareMap.dcMotor.get("lift");

        ballLoad = hardwareMap.servo.get("servo1");
        ButtonClick = hardwareMap.servo.get("servo2");
        arm = hardwareMap.servo.get("arm");
        lock = hardwareMap.servo.get("lock");

        color = hardwareMap.colorSensor.get("color");
        touch = hardwareMap.touchSensor.get("touch");

       // LaunchTouch = hardwareMap.touchSensor.get("touch");

        //ButtonClick.setPosition(buttonPos1);

        boolean bEnabled = true;

        color.enableLed(false);

        DriveMotor2.setDirection(DcMotor.Direction.REVERSE);

        ballLPosition = 0.4;
        arm.setPosition(1.0);
        armpos = 1.0;

        float hsvValues[] = {0F, 0F, 0F};

        final float values[] = hsvValues;

        endToggle = 0;
        lock.setPosition(0.6);

        DriveMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        DriveMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */


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

        if(endToggle == 0) {
            lock.setPosition(0.6);
            arm.setPosition(1.0);
            armpos = 1.0;

            telemetry.addData("Toggle: ", endToggle);
            telemetry.update();

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

            // Assign joysticks to motors
            LeftDrive = -gamepad1.left_stick_y;
            RightDrive = gamepad1.right_stick_y;

            DriveMotor1.setPower(LeftDrive);
            DriveMotor2.setPower(RightDrive);

            LaunchDrive = gamepad2.right_stick_y;
            IntakeDrive = gamepad2.left_stick_y;

            if (gamepad2.x) {
                ballLPosition = 0.0;
                telemetry.addData("SPosition", ballLoad.getPosition());
                telemetry.update();
            } else {
                ballLPosition = 0.23;
                telemetry.addData("SPosition", ballLoad.getPosition());
                telemetry.update();
            }
            if (gamepad2.right_bumper) {
                while (true) {
                    LaunchDrive = gamepad2.right_stick_y;
                    IntakeDrive = gamepad2.left_stick_y;
                    LeftDrive = -gamepad1.left_stick_y;
                    RightDrive = gamepad1.right_stick_y;
                    LaunchMotor.setPower(0.3);
                    if (LaunchTouch.isPressed()) {
                        LaunchMotor.setPower(0.0);
                        ballLPosition = 0.4;
                        break;
                    }
                }
                LaunchMotor.setPower(0.0);
            }

            if (gamepad2.y) {
                while (1 == 1) {
                    LaunchMotor.setPower(-0.64);
                    if (touch.isPressed() || gamepad2.dpad_up) {
                        LaunchMotor.setPower(0.0);
                        break;
                    }
                }
                LaunchMotor.setPower(0.0);
            }

            // Half speed for drive train
            if (gamepad1.right_bumper) {
                DriveMotor1.setPower(LeftDrive);
                DriveMotor2.setPower(RightDrive);

            } else {
                DriveMotor1.setPower(LeftDrive / 2);
                DriveMotor2.setPower(RightDrive / 2);

            }
            // Half speed for intake system
            if (gamepad2.right_bumper) {
                IntakeMotor.setPower(IntakeDrive / 2);
            } else {
                IntakeMotor.setPower(IntakeDrive);
            }
            if (gamepad1.y) {
                mySound.play(kobe, 1, 1, 1, 0, 1);
            }
            if (gamepad1.x) {
                mySound.play(penguin1, 1, 1, 1, 0, 1);
            }
            if (gamepad1.dpad_left) {
                ButtonClick.setPosition(buttonPos1);
            } else {
                ButtonClick.setPosition(buttonPos2);
            }

            while (gamepad1.dpad_up) {
                color.enableLed(false);
                if (color.red() >= 3) {
                    DriveMotor1.setPower(0.0);
                    DriveMotor2.setPower(0.0);
                    break;
                } else if (color.blue() >= 6) {
                    DriveMotor1.setPower(0.0);
                    DriveMotor2.setPower(0.0);
                    break;
                } else {
                    DriveMotor1.setPower(0.18);
                    DriveMotor2.setPower(-0.13);
                }
            }
            if (!gamepad1.dpad_up) {
                LeftDrive = -gamepad1.left_stick_y;
                RightDrive = gamepad1.right_stick_y;
            }

            while (gamepad1.dpad_down) {
                color.enableLed(false);
                if (color.red() >= 3) {
                    DriveMotor1.setPower(0.0);
                    DriveMotor2.setPower(0.0);
                    break;
                } else if (color.blue() >= 6) {
                    DriveMotor1.setPower(0.0);
                    DriveMotor2.setPower(0.0);
                    break;
                } else {
                    DriveMotor1.setPower(-0.18);
                    DriveMotor2.setPower(0.13);
                }
            }
            if (!gamepad1.dpad_down) {
                LeftDrive = -gamepad1.left_stick_y;
                RightDrive = gamepad1.right_stick_y;
            }
        }


        // endToggle == 1 :: LiftDrive
        if(endToggle == 1)
        {
            telemetry.addData("Toggle: ", endToggle);
            telemetry.update();

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

            // Assign controls to motors
            LiftDrive = gamepad2.right_stick_y;
            LeftDrive = gamepad1.right_stick_y;
            RightDrive = -gamepad1.left_stick_y;

            if(LiftDrive > 0)
            {
                lock.setPosition(0.0);
            }


            if(gamepad2.a)
            {
                lock.setPosition(0.0);
            }
            if(gamepad2.b)
            {
                lock.setPosition(0.6);
            }


            if(gamepad1.right_bumper)
            {
                DriveMotor1.setPower(LeftDrive / 1.5);
                DriveMotor2.setPower(RightDrive / 1.5);
            }
            else
            {
                DriveMotor1.setPower(LeftDrive / 2);
                DriveMotor2.setPower(RightDrive / 2);
            }
            if(gamepad2.x)
            {
                armpos = 0.0;
                telemetry.addData("Arm: ", "" + String.format("%.2f", armpos));
                telemetry.update();
            }
            else
            {
                armpos = 1.0;
                telemetry.addData("Arm: ", "" + String.format("%.2f", armpos));
                telemetry.update();
            }

            if (gamepad2.y) {
                while (1 == 1) {
                    LaunchMotor.setPower(-0.64);
                    if (touch.isPressed() || gamepad2.dpad_up) {
                        telemetry.addData("Touch: ", touch.isPressed());
                        telemetry.addData("DPAD: ", gamepad2.dpad_up);
                        telemetry.update();
                        LaunchMotor.setPower(0.0);

                        break;
                    }
                }
                LaunchMotor.setPower(0.0);
            }
        }


        LiftMotor.setPower(LiftDrive);
        LaunchMotor.setPower(LaunchDrive);

		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */
        telemetry.addData("Left: ", "" + String.format("%.2f", LeftDrive));
        telemetry.addData("Right: ", "" + String.format("%.2f", RightDrive));
        telemetry.addData("Intake: ", "" + String.format("%.2f", IntakeDrive));
        telemetry.addData("Ballsac: ", "" + String.format("%.2f", LaunchDrive));
        telemetry.addData("Lift: ", "" + String.format("%.2f", LiftDrive));
        telemetry.addData("LBuumper", gamepad1.left_bumper);
        telemetry.addData("RBumper: ", gamepad1.right_bumper);
        telemetry.addData("Start: ", gamepad1.start);
        telemetry.update();




        ballLPosition = Range.clip(ballLPosition, BALL_MIN_RANGE, BALL_MAX_RANGE);
        armpos = Range.clip(armpos, ARM_MIN_RANGE, ARM_MAX_RANGE);

        arm.setPosition(armpos);
        ballLoad.setPosition(ballLPosition);

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
        IntakeMotor.setPower(0.0);
        LaunchMotor.setPower(0.0);
        LiftMotor.setPower(0.0);
        arm.setPosition(0.0);

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

        // index cannot exceed size of array minus 1.
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
