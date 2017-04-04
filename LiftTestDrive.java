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

import android.media.AudioManager;
import android.media.SoundPool;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class LiftTestDrive extends OpMode {

    double armpos = 0.0;

    final static double ARM_MIN_RANGE = 0.0;
    final static double ARM_MAX_RANGE = 1.0;

    DcMotor LiftMotor;
    DcMotor DriveMotor1;
    DcMotor DriveMotor2;

    Servo arm;
    Servo lock;

    double LiftDrive = 0.0;
    double LeftDrive = 0.0;
    double RightDrive = 0.0;


    public LiftTestDrive() {
    }

    /*
     * Code to run when the op mode is first enabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init() {

     // Hard code the motors to a string value for the config

        LiftMotor = hardwareMap.dcMotor.get("lift");
        DriveMotor1 = hardwareMap.dcMotor.get("left");
        DriveMotor2 = hardwareMap.dcMotor.get("right");

        arm = hardwareMap.servo.get("arm");
        lock = hardwareMap.servo.get("lock");

        arm.setPosition(0.0);
        lock.setPosition(0.6);

        DriveMotor2.setDirection(DcMotor.Direction.REVERSE);

    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */

    boolean toggle;

    @Override
    public void loop() {

        // Assign controls to motors
        LiftDrive = gamepad2.right_stick_y;
        LeftDrive = gamepad1.right_stick_y;
        RightDrive = -gamepad1.left_stick_y;

        if(gamepad1.right_bumper)
        {
            DriveMotor1.setPower(LeftDrive / 2);
            DriveMotor2.setPower(RightDrive / 2);
        }
        else
        {
            DriveMotor1.setPower(LeftDrive / 4);
            DriveMotor2.setPower(RightDrive / 4);
        }

        if(gamepad2.a)
        {
            lock.setPosition(0.0);
        }
        if(gamepad2.b)
        {
            lock.setPosition(0.6);
        }


        if(gamepad2.y)
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

        LiftMotor.setPower(LiftDrive);

		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		*/
        telemetry.addData("Lift: ", "" + String.format("%.2f", LiftDrive));
        telemetry.addData("Left: ", "" + String.format("%.2f", LeftDrive));
        telemetry.addData("Right: ", "" + String.format("%.2f", RightDrive));
        telemetry.addData("Arm: ", "" + String.format("%.2f", armpos));
        telemetry.update();

        armpos = Range.clip(armpos, ARM_MIN_RANGE, ARM_MAX_RANGE);

        arm.setPosition(armpos);

    }

    /*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
    @Override
    public void stop() {

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
