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

import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.teamcode.R.layout.servo;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="3860 Main Drive Program", group="Iterative Opmode")

public class Hdrive_Teleop extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //DcMotor leftDrive = hardwareMap.dcMotor.get("left");
    //DcMotor rightDrive = hardwareMap.dcMotor.get("right");
    //DcMotor middleDrive = hardwareMap.dcMotor.get("Hdrive");
    //DcMotor liftDrive = hardwareMap.dcMotor.get("lift");

    Servo rightArm;
    Servo leftArm;

    private DcMotor rightDrive;
    private DcMotor leftDrive;
    private DcMotor middleDrive;
    private DcMotor liftDrive;
    /*double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    boolean rampUp = true;

    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position*/


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery


        //servoLeft.setDirection(Servo.Direction.);

        rightArm = hardwareMap.servo.get("rightArm");
        leftArm = hardwareMap.servo.get("leftArm");
        leftDrive = hardwareMap.dcMotor.get("left");
        rightDrive = hardwareMap.dcMotor.get("right");
        middleDrive = hardwareMap.dcMotor.get("Hdrive");
        liftDrive = hardwareMap.dcMotor.get("lift");

        // Tell the driver that initialization is complete.
        //telemetry.addData("Status", "Initialized");
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */

    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;
        double middlePower;
        double liftPower;
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

            rightArm.setPosition(1);
            leftArm.setPosition(0);

        }


        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        leftPower  = -gamepad1.left_stick_y ;
        rightPower = -gamepad1.right_stick_y;
        middlePower = -gamepad1.left_stick_x;
        liftPower = -gamepad2.right_stick_y;

        if (gamepad1.right_trigger >= 0.1) {

            middleDrive.setPower(-gp1Rtrigger);

        } else {

            middleDrive.setPower(0);

        }

        if (gamepad1.left_trigger >= 0.1) {

            middleDrive.setPower(gp1Ltrigger);

        } else {

            middleDrive.setPower(0);

        }

        // Send calculated power to wheels
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        liftDrive.setPower(liftPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f), Hdrive(%.2f)", leftPower, rightPower, middlePower);
        telemetry.addLine("Updated");
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
