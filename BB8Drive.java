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
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

PROPERTY OF NYAN ROBOTICS @ Danny Donahoe
*/

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class BB8Drive extends OpMode {

    DcMotor motor1;
    DcMotor motor2;
    DcMotor motor3;
    //Servo servoHead;

    double motor1_drive = 0.0;
    double motor2_drive = 0.0;
    double motor3_drive = 0.0;

    double motorvars;

    /**
     * Constructor
     */
    public BB8Drive() {

    }

    /*
     * Code to run when the op mode is first enabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init() {

        // Initialize the hardware.
        motor1 = hardwareMap.dcMotor.get("motor_1");
        motor2 = hardwareMap.dcMotor.get("motor_2");
        motor3 = hardwareMap.dcMotor.get("motor_3");
        //servoHead = hardwareMap.servo.get("servohead");
        motor2.setDirection(DcMotor.Direction.REVERSE);

    }



    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop()
    {
        motor1_drive = gamepad1.left_stick_y;
        motor2_drive = gamepad1.left_stick_y;
        motor3_drive = gamepad1.right_stick_x;


        if(gamepad1.left_bumper)
        {
            motor1.setPower(motor1_drive / 4);
            telemetry.addData("Motor1 Power", motor1_drive / 2);
            motor2.setPower(motor2_drive / 4);
            telemetry.addData("Motor2 Power", motor2_drive / 2);
            //motor1.setPower(motor1_drive / 4);
            //telemetry.addData("Motor1 Power", (motor1_drive / 4));
            //motor2.setPower(motor2_drive / 4);
            //telemetry.addData("Motor2 Power", motor2_drive / 4);

        }
        if(!gamepad1.left_bumper) {
            motor1.setPower(motor1_drive/2);
            telemetry.addData("Motor1 Power", motor1_drive/4);
            motor2.setPower(motor2_drive/2);
            telemetry.addData("Motor2 Power", motor2_drive/4);
            //motor1.setPower(motor1_drive / 2);
            //telemetry.addData("Motor1 Power", motor1_drive / 2);
            //motor2.setPower(motor2_drive / 2);
            //telemetry.addData("Motor2 Power", motor2_drive / 2);
        }

        if(gamepad1.right_bumper)
        {
            
            motor3.setPower(motor3_drive / 2);
            telemetry.addData("Motor3 Power", (motor3_drive / 2));
        }
        else
        {
            motor3.setPower(motor3_drive);
            telemetry.addData("Motor3 Power", motor3_drive);
        }
    }


    private double scaleInput(double dVal) {
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

    /*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
    @Override
    public void stop() {
        motor1.setPower(0.0);
        motor2.setPower(0.0);
        motor3.setPower(0.0);
    }
}
