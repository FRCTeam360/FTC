package org.firstinspires.ftc.teamcode;

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
    //Y direction format //// True = left, False = Right

        MovementX(200, .5, true);
        MovementY(200, .5, true);
        MovementX(200, .5, false);
        MovementY(200, .5, false);

    }
}
