package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.android.dx.command.DxConsole;

/**
 * Created by Gavin on 11/4/2017.
 */

@TeleOp(name="360 Main Drive Program", group="Iterative OpMode")

public class MainDrive360 extends OpMode {

    private DcMotor Motor0;
    private DcMotor Motor1;
    private DcMotor Motor2;
    private DcMotor Motor3;
    private DcMotor ArmLift;

    Servo leftArm;
    Servo rightArm;

    @Override
    public void init() {

        telemetry.addData("Current Status", "Initialized");
        telemetry.update();

        //Map The Motors To The Expansion Hub
        Motor0 = hardwareMap.get(DcMotor.class, "motor1");
        Motor1 = hardwareMap.get(DcMotor.class, "motor2");
        Motor2 = hardwareMap.get(DcMotor.class, "motor3");
        Motor3 = hardwareMap.get(DcMotor.class, "motor4");
        ArmLift = hardwareMap.get(DcMotor.class, "motor5");
        rightArm = hardwareMap.servo.get("rightArm");
        leftArm = hardwareMap.servo.get("leftArm");
        //Set Motor Directions
        Motor0.setDirection(DcMotorSimple.Direction.FORWARD);
        Motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        Motor2.setDirection(DcMotorSimple.Direction.FORWARD);
        Motor3.setDirection(DcMotorSimple.Direction.FORWARD);
        //Wait For Robot To Be Initialized

    }

    @Override
    public void loop() {
        float gamepad1LeftY = -gamepad1.left_stick_y;
        float gamepad1LeftX = gamepad1.left_stick_x;
        float gamepad1RightX = gamepad1.right_stick_x;
        double TriggerPowerR;
        double TriggerPowerL;
        double gp1Rtrigger;
        double gp1Ltrigger;
        double gp2Rtrigger;
        double gp2Ltrigger;

        gp1Ltrigger = gamepad1.left_trigger;
        gp1Rtrigger = gamepad1.right_trigger;
        gp2Ltrigger = gamepad2.left_trigger;
        gp2Rtrigger = gamepad2.right_trigger;

        float FrontLeft = -gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
        float FrontRight = gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
        float BackRight = gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
        float BackLeft = -gamepad1LeftY + gamepad1LeftX - gamepad1RightX;

        FrontRight = Range.clip(FrontRight, -1, 1);
        FrontLeft = Range.clip(FrontLeft, -1, 1);
        BackLeft = Range.clip(BackLeft, -1, 1);
        BackRight = Range.clip(BackRight, -1, 1);

        //back right on the robot
        Motor0.setPower(-BackRight);
        Motor1.setPower(-FrontRight);
        Motor2.setPower(-BackLeft);
        Motor3.setPower(-FrontLeft);


            //Right Turn Shortcut
            TriggerPowerR = gamepad1.right_trigger;

            if (gamepad1.right_trigger >= 0.1) {

                Motor0.setPower(-TriggerPowerR);
                Motor1.setPower(-TriggerPowerR);
                Motor2.setPower(-TriggerPowerR);
                Motor3.setPower(-TriggerPowerR);

            }
            //Left Turn Shortcut
            TriggerPowerL = gamepad1.left_trigger;

            if (gamepad1.left_trigger >= 0.1) {

                Motor0.setPower(TriggerPowerL);
                Motor1.setPower(TriggerPowerL);
                Motor2.setPower(TriggerPowerL);
                Motor3.setPower(TriggerPowerL);
            }

            if (gamepad2.right_trigger >= 0.1) {

                rightArm.setPosition(0);
                leftArm.setPosition(1);

            }

            if (gamepad2.left_trigger >= 0.1) {

                rightArm.setPosition(1);
                leftArm.setPosition(0);

            }

            if (gamepad2.right_stick_y >= 0.3) {

                ArmLift.setPower(1);

            } else {

                ArmLift.setPower(0);

            }

            if (gamepad2.right_stick_y <= -0.3) {

                ArmLift.setPower(-1);

            } else {

                ArmLift.setPower(0);

            }
            telemetry.addLine("FTC 360 Main Drive Program");
            telemetry.addLine("Last Updated: 12/5/17");
            telemetry.addLine("Last Updated By: Gavin");
            telemetry.addData("GP1 Trigger L: ", gp1Ltrigger);
            telemetry.addData("GP1 Trigger R: ", gp1Rtrigger);
            telemetry.addData("GP2 Trigger L: ", gp2Ltrigger);
            telemetry.addData("GP2 Trigger R: ", gp2Rtrigger);
            telemetry.update();
        }
    }

