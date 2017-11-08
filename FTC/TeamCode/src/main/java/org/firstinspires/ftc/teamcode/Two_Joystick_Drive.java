package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

// Created By Gavin on September 12, 2017

//Create The Mode
@TeleOp(name="MultiOP Mode", group="Linear OpMode")

public class Two_Joystick_Drive extends LinearOpMode {

    //Runtime
    private ElapsedTime runtime = new ElapsedTime();

    //Define The Motors
    private DcMotor Motor0;
    private DcMotor Motor1;
    private DcMotor Motor2;
    private DcMotor Motor3;

    Servo Arm;

    @Override
    public void runOpMode() {

        //Telemetry Setup
        telemetry.addData("Current Status", "Initialized");
        telemetry.update();

        //Map The Motors To The Expansion Hub
        Motor0 = hardwareMap.get(DcMotor.class, "motor1");
        Motor1 = hardwareMap.get(DcMotor.class, "motor2");
        Motor2 = hardwareMap.get(DcMotor.class, "motor3");
        Motor3 = hardwareMap.get(DcMotor.class, "motor4");

        //Set Motor Directions
        Motor0.setDirection(DcMotorSimple.Direction.FORWARD);
        Motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        Motor2.setDirection(DcMotorSimple.Direction.FORWARD);
        Motor3.setDirection(DcMotorSimple.Direction.FORWARD);

        //Wait For Robot To Be Initialized
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            boolean running = true;
            //Define Ints for Control Modes
            boolean mode = false;

            //Define Motor Power Variables
            double Motor0Power;
            double Motor1Power;
            double Motor2Power;
            double Motor3Power;
            double TriggerPowerR;
            double TriggerPowerL;

            double JoystickYvalue;
            double JoystickXvalue;

            while (running == true) {
                //Controller 1
                //Standard Operation
                if (mode == false) {
                    //Set Stick to Left Stick
                    Motor0Power = gamepad1.left_stick_y;
                    Motor1Power = -gamepad1.left_stick_y;
                    //Main Drive (Left Joystick)
                    Motor0.setPower(Motor0Power);
                    Motor1.setPower(Motor1Power);
                    //Set Stick to Right Stick
                    Motor2Power = gamepad1.right_stick_y;
                    Motor3Power = -gamepad1.right_stick_y;
                    //Secondary Drive and Turning
                    Motor2.setPower(Motor2Power);
                    Motor3.setPower(Motor3Power);
                    //Turning Shortcuts
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

                    //1 Stick Operation
                    if (gamepad1.a) {

                        mode = true;

                    }

                }
                //1 Stick
                if (mode == true) {

                    Motor0Power = gamepad1.left_stick_y;
                    Motor1Power = -gamepad1.left_stick_x;
                    Motor2Power = gamepad1.left_stick_y;
                    Motor3Power = -gamepad1.left_stick_x;
                    Motor0.setPower(Motor0Power);
                    Motor1.setPower(Motor1Power);
                    Motor2.setPower(Motor2Power);
                    Motor3.setPower(Motor3Power);

                    /*//Set Stick to Left Stick
                    Motor0Power = gamepad1.left_stick_y;
                    Motor1Power = -gamepad1.left_stick_y;
                    //Main Drive (Left Joystick)
                    Motor0.setPower(Motor0Power);
                    Motor1.setPower(Motor1Power);
                    //Set Stick to Right Stick
                    Motor2Power = gamepad1.left_stick_x;
                    Motor3Power = -gamepad1.left_stick_x;
                    //Secondary Drive and Turning
                    Motor2.setPower(Motor2Power);
                    Motor3.setPower(Motor3Power);
                    */
                    //Turning Shortcuts
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

                    //2 Stick Operation
                    if (gamepad1.b) {

                        mode = false;

                    }
                    //1 Stick Operation
                }


                //Telemetry Data
                telemetry.addData("Current Status", "Up Time: " + runtime.toString());
                telemetry.addData("Is in one stick operation: ", mode);
                telemetry.update();
                idle();
            }
        }
    }
}
