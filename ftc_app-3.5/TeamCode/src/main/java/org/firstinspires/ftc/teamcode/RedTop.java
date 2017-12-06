package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;


/**
 * Created by Gavin on 11/14/2017.
 */

@Autonomous(name="RED Top", group="Linear Opmode")

public class RedTop extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
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
    Servo blockArm;

    ColorSensor sensorColor;
    DistanceSensor sensorDistance;

    public void armGrip(int direction) {
        switch (direction){
            case 1:
                servo(-1, 2, 500);
                servo(1, 3, 500);
                break;
            case 2:
                servo(-1, 2, 500);
                servo(1, 3, 500);
                break;
        }

    }


    public void colorDistance(int runs, int timebetween, float hsvValues[]) {

        int dummy = 0;
        final double SCALE_FACTOR = 255;
        final float values[] = hsvValues;

        while (dummy <= runs) {
            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);
            int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
            final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
            telemetry.addData("Distance (cm)", String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
            telemetry.addData("Alpha", sensorColor.alpha());
            telemetry.addData("Red  ", sensorColor.red());
            telemetry.addData("Green", sensorColor.green());
            telemetry.addData("Blue ", sensorColor.blue());
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.update();
            dummy = dummy + 1;
            sleep(timebetween);
        }
    }


    public void KillMotors() {
        double power = 0;
        Motor0Power = power;
        Motor1Power = power;
        Motor2Power = power;
        Motor3Power = power;
        Motor0.setPower(Motor0Power);
        Motor1.setPower(Motor1Power);
        Motor2.setPower(Motor2Power);
        Motor3.setPower(Motor3Power);
        sleep(100);

    }


    public void servo(double pos, int choice, int sleep) {

        switch (choice) {
            case 1:
                rightArm.setPosition(pos);
                if (rightArm.getPosition() == pos) {
                    sleep(sleep);
                    break;
                }
            case 2:
                leftArm.setPosition(pos);
                if (leftArm.getPosition() == pos) {
                    sleep(sleep);
                    break;
                }
            case 3:
                jemArm.setPosition(pos);
                if (jemArm.getPosition() == pos) {
                    sleep(sleep);
                    break;
                }
            case 4:
                blockArm.setPosition(pos);
                if (blockArm.getPosition() == pos) {
                    sleep(sleep);
                    break;
                }
        }
    }


    public void drive(int timeval, int timeafterval,  double power, boolean angleDrive, int choice) {

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
                    break;
                case 2:
                    Motor0.setPower(-Motor0Power);
                    Motor1.setPower(Motor1Power);
                    Motor2.setPower(-Motor2Power);
                    Motor3.setPower(Motor3Power);
                    break;
                case 3:
                    Motor0.setPower(Motor0Power);
                    Motor1.setPower(Motor1Power);
                    Motor2.setPower(-Motor2Power);
                    Motor3.setPower(-Motor3Power);
                    break;
                case 4:
                    Motor0.setPower(-Motor0Power);
                    Motor1.setPower(-Motor1Power);
                    Motor2.setPower(Motor2Power);
                    Motor3.setPower(Motor3Power);
                    break;
            }
        }
        if (angleDrive == true) {
            switch (choice) {
                case 1:
                    Motor0.setPower(Motor0Power);
                    Motor1.setPower(-Motor1Power);
                    break;
                case 2:
                    Motor0.setPower(-Motor0Power);
                    Motor1.setPower(Motor1Power);
                    break;
                case 3:
                    Motor2.setPower(Motor2Power);
                    Motor3.setPower(-Motor3Power);
                    break;
                case 4:
                    Motor2.setPower(-Motor2Power);
                    Motor3.setPower(Motor3Power);
                    break;
            }
        }
        sleep(timeval);
        KillMotors();
        sleep(timeafterval);
        return;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Current Status", "Initialized");
        telemetry.update();
        //Map The Motors To The Expansion Hub
        Motor0 = hardwareMap.get(DcMotor.class, "motor1");
        Motor1 = hardwareMap.get(DcMotor.class, "motor2");
        Motor2 = hardwareMap.get(DcMotor.class, "motor3");
        Motor3 = hardwareMap.get(DcMotor.class, "motor4");
        ArmLift = hardwareMap.get(DcMotor.class, "motor5");
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor");
        rightArm = hardwareMap.servo.get("rightArm");
        leftArm = hardwareMap.servo.get("leftArm");
        jemArm = hardwareMap.servo.get("jemArm");
        blockArm = hardwareMap.servo.get("blockArm");
        //Set Motor Directions
        Motor0.setDirection(DcMotorSimple.Direction.FORWARD);
        Motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        Motor2.setDirection(DcMotorSimple.Direction.FORWARD);
        Motor3.setDirection(DcMotorSimple.Direction.FORWARD);
        float hsvValues[] = {0F, 0F, 0F};
        boolean colorFound = false;
        waitForStart();

        while (opModeIsActive()) {

            servo(.5, 3, 1000);
            sleep(2100);
            drive(100, 2000, .5, false, 1);

            while (colorFound == false) {
                colorDistance(1, 100, hsvValues);
                if (sensorColor.red() >= 20) {
                    servo(1, 3, 1000);
                    colorFound = true;
                } else if (sensorColor.blue() >= 20) {
                    servo(0, 3, 1000);
                    colorFound = true;
                }
            }

            drive(200, 1000, .5, false, 2);
            KillMotors();
            sleep(1000);
            drive(1000, 2000, 1, false, 3);
            KillMotors();
            sleep(1000);
            drive(600, 2000, 1, false, 2);
            KillMotors();
            servo(0.8, 4, 10000);
        }
    }
}
