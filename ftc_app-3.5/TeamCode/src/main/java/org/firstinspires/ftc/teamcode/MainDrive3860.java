 /* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//Before running the app for the first time, install the "OpenCV Manager" from the Google Play Store to enable Vision processing.

@TeleOp(name="3860 Main Drive Program", group="Iterative Opmode")

public class MainDrive3860 extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

   // OpenGLMatrix lastLocation = null;
   // VuforiaLocalizer vuforia;

    Servo rightArm;
    Servo leftArm;

    private DcMotor rightDrive;
    private DcMotor leftDrive;
    //private DcMotor middleDrive;
    private DcMotor liftDrive;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

      //  int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
      //  VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        telemetry.addData("Status", "Initialized");

       // parameters.vuforiaLicenseKey = "AbIVQOL/////AAAAGZkdkXA/G0ozhlFC64dTobAEKNYrYjGJj1qJRO1pj/9S+bA6tErf1GcodqHqYzSmXse47lIF+SUUJ+tMb3OD6rpDDzHdZWGQ20XWRrVn6wByfph4hPG3bNeyKXHp43D/VltBYxo+EK2J6bnpRLKIaEAhpVdhSSf4BUR86Hvi9Pduy7sswLJUpNbENvgs3AoP+RYh/KYF2CPtfWkM9FQAUIjriW2+6YUoKY62xW0LfUeR1VAUpNsIDsWFAVOZV/cKBplC31tWDuCgd3aKEjEZdBQElKQPu9zore+GsrkFZrhazIkgvULcfPMFaHtG1/LfpcgELIldFu8ITfdDqwYdQgYFJZlwlz433Blajj7xDYdT";

        rightArm = hardwareMap.servo.get("rightArm");
        leftArm = hardwareMap.servo.get("leftArm");
        leftDrive = hardwareMap.dcMotor.get("left");
        rightDrive = hardwareMap.dcMotor.get("right");
        //middleDrive = hardwareMap.dcMotor.get("Hdrive");
        liftDrive = hardwareMap.dcMotor.get("lift");

        // Tell the driver that initialization is complete.
        //telemetry.addData("Status", "Initialized");
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

       // parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
       // this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

    }



    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;
        //double middlePower;
        //double liftPower;
        double gp1Rtrigger;
        double gp1Ltrigger;
        double gp2Rtrigger;
        double gp2Ltrigger;

        gp1Ltrigger = gamepad1.left_trigger;
        gp1Rtrigger = gamepad1.right_trigger;
        gp2Ltrigger = gamepad2.left_trigger;
        gp2Rtrigger = gamepad2.right_trigger;

        if (gamepad2.right_trigger >= 0.1) {

            rightArm.setPosition(0);
            leftArm.setPosition(1);

        }

        if (gamepad2.left_trigger >= 0.1) {

            rightArm.setPosition(.8);
            leftArm.setPosition(.2);

        }


        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        leftPower  = -gamepad1.left_stick_y ;
        rightPower = -gamepad1.right_stick_y;
        //middlePower = -gamepad1.left_stick_x;

        if (gamepad2.left_stick_y >= 0.3){

            liftDrive.setPower(-.2);

        } else {

            liftDrive.setPower(0);

        }

        if (gamepad2.left_stick_y <= -0.3){

            liftDrive.setPower(1);

        } else {

            liftDrive.setPower(0);

        }

          /*if (gamepad1.right_trigger >= 0.1) {

            middleDrive.setPower(-1);

        } else if (gamepad1.left_trigger >= 0.1) {

            middleDrive.setPower(1);

        } else {

            middleDrive.setPower(0);

        }*/



        // Send calculated power to wheels
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);

        // Show the elapsed game time and wheel power.
        telemetry.addLine("FTC 3860 Main Drive Program");
        telemetry.addLine("Last Updated: 12/2/17");
        telemetry.addLine("Last Updated By: Matthew V");
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.addData("GP1 Trigger L: " , gp1Ltrigger);
        telemetry.addData("GP1 Trigger R: " , gp1Rtrigger);
        telemetry.addData("GP2 Trigger L: " , gp2Ltrigger);
        telemetry.addData("GP2 Trigger R: " , gp2Rtrigger);
        telemetry.update();

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
