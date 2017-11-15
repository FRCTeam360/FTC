package org.firstinspires.ftc.teamcode;

import android.widget.Switch;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static android.os.SystemClock.sleep;


/**
 * Created by Gavin on 11/14/2017.
 * I just want to say that the way that java handles strings is absolute crap.
 */

@Autonomous(name="Auto Drive 360 Test", group="Linear Opmode")

public class AutoDrive360Test extends OpMode {

    double Motor0Power;
    double Motor1Power;
    double Motor2Power;
    double Motor3Power;
    private DcMotor Motor0;
    private DcMotor Motor1;
    private DcMotor Motor2;
    private DcMotor Motor3;
    private DcMotor ArmLift;

    Servo leftArm;
    Servo rightArm;
    Servo jemArm;

    public void telemetryStatus(int choice){
        if (choice == 1) {
            telemetry.addLine("Current Instruction: Left");
            telemetry.update();
        } else if (choice == 2) {
            telemetry.addLine("Current Instruction: Right");
            telemetry.update();
        } else if (choice == 3) {
            telemetry.addLine("Current Instruction: Forward");
            telemetry.update();
        } else if (choice == 4) {
            telemetry.addLine("Current Instruction: Backwards");
            telemetry.update();
        } else if (choice == 5) {
            telemetry.addLine("Current Instruction: Left Angle");
            telemetry.update();
        } else if (choice == 6) {
            telemetry.addLine("Current Instruction: Right Angle");
            telemetry.update();
        } else if (choice == 7) {
            telemetry.addLine("Current Instruction: Forward Angle");
            telemetry.update();
        } else if (choice == 8) {
            telemetry.addLine("Current Instruction: Backwards Angle");
            telemetry.update();
        } else {
            telemetry.addLine("You can not have a null direction");
            telemetry.update();
        }
    }

    public void KillMotors() {

        Motor0Power = 0;
        Motor1Power = 0;
        Motor2Power = 0;
        Motor3Power = 0;
        Motor0.setPower(Motor0Power);
        Motor1.setPower(Motor1Power);
        Motor2.setPower(Motor2Power);
        Motor3.setPower(Motor3Power);

    }


    public void drive(int timeval, double power, int timeafterval, boolean angleDrive, String direction) {

        int choice = 0;

        if (direction == "left") {
            choice = 1;
        } else if (direction == "right") {
            choice = 2;
        } else if (direction == "forward") {
            choice = 3;
        } else if (direction == "back") {
            choice = 4;
        } else {
            telemetry.addLine("You can not have a null direction");
            telemetry.update();
        }

        Motor0Power = power;
        Motor1Power = power;
        Motor2Power = power;
        Motor3Power = power;

        if (angleDrive == false) {
            switch (choice) {
                case 1:
                    Motor0.setPower(Motor0Power);
                    Motor1.setPower(-Motor1Power);
                    Motor2.setPower(Motor2Power);
                    Motor3.setPower(-Motor3Power);
                    telemetryStatus(1);
                    sleep(timeval);
                    break;
                case 2:
                    Motor0.setPower(-Motor0Power);
                    Motor1.setPower(Motor1Power);
                    Motor2.setPower(-Motor2Power);
                    Motor3.setPower(Motor3Power);
                    telemetryStatus(2);
                    sleep(timeval);
                    break;
                case 3:
                    Motor0.setPower(Motor0Power);
                    Motor1.setPower(Motor1Power);
                    Motor2.setPower(-Motor2Power);
                    Motor3.setPower(-Motor3Power);
                    telemetryStatus(3);
                    sleep(timeval);
                    break;
                case 4:
                    Motor0.setPower(-Motor0Power);
                    Motor1.setPower(-Motor1Power);
                    Motor2.setPower(Motor2Power);
                    Motor3.setPower(Motor3Power);
                    telemetryStatus(4);
                    sleep(timeval);
                    break;
            }
        }
        if (angleDrive == true){
            switch (choice){
                case 1:
                    Motor0.setPower(Motor0Power);
                    Motor1.setPower(-Motor1Power);
                    telemetryStatus(5);
                    sleep(timeval);
                    break;
                case 2:
                    Motor0.setPower(-Motor0Power);
                    Motor1.setPower(Motor1Power);
                    telemetryStatus(6);
                    sleep(timeval);
                    break;
                case 3:
                    Motor2.setPower(Motor2Power);
                    Motor3.setPower(-Motor3Power);
                    telemetryStatus(7);
                    sleep(timeval);
                    break;
                case 4:
                    Motor2.setPower(-Motor2Power);
                    Motor3.setPower(Motor3Power);
                    telemetryStatus(8);
                    sleep(timeval);
                    break;
            }
        }
        KillMotors();
        sleep(timeafterval);
    }
    //Not Intended to be used anymore (Use at your own risk)
    public void MovementX(int timeval, double power, boolean direction) {
        if (direction == true) {
            Motor0Power = power;
            Motor1Power = power;
            Motor2Power = power;
            Motor3Power = power;
            //set speed
            Motor0.setPower(-Motor0Power);
            Motor1.setPower(-Motor1Power);
            Motor2.setPower(Motor2Power);
            Motor3.setPower(Motor3Power);
            sleep(timeval);
        }
        if (direction == false) {
            Motor0Power = power;
            Motor1Power = power;
            Motor2Power = power;
            Motor3Power = power;
            //set speed
            Motor0.setPower(Motor0Power);
            Motor1.setPower(Motor1Power);
            Motor2.setPower(-Motor2Power);
            Motor3.setPower(-Motor3Power);
            sleep(timeval);
        }
    }

    public void MovementY(int timeval, double power, boolean direction) {
        if (direction == true) {
            Motor0Power = power;
            Motor1Power = power;
            Motor2Power = power;
            Motor3Power = power;
            //set speed
            Motor0.setPower(Motor0Power);
            Motor1.setPower(-Motor1Power);
            Motor2.setPower(Motor2Power);
            Motor3.setPower(-Motor3Power);
            sleep(timeval);
        }
        if (direction == false) {
            Motor0Power = power;
            Motor1Power = power;
            Motor2Power = power;
            Motor3Power = power;
            //set speed
            Motor0.setPower(-Motor0Power);
            Motor1.setPower(Motor1Power);
            Motor2.setPower(-Motor2Power);
            Motor3.setPower(Motor3Power);
            sleep(timeval);
        }
    }


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
        jemArm = hardwareMap.servo.get("jemArm");
        //Set Motor Directions
        Motor0.setDirection(DcMotorSimple.Direction.FORWARD);
        Motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        Motor2.setDirection(DcMotorSimple.Direction.FORWARD);
        Motor3.setDirection(DcMotorSimple.Direction.FORWARD);
        //Wait For Robot To Be Initialized

    }

    @Override
    public void loop() {
    //Don't Delete This. Strings in Java are super clunky without them
    //Possible directions
        String left = "left";
        String right = "right";
        String forward = "forward";
        String back = "back";
    //Write Drive Code Here
    //Syntax for Drive Function == drive(timeval (int), power (double), aftertimeval (int), angleDrive (boolean), direction (string);

        drive(500, 1, 500, false, left);

    }
}