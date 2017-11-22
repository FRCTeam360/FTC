package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.Locale;
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
/*
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;


    public void colerDistance() {
        float hsvValues[] = {0F, 0F, 0F};
        final double SCALE_FACTOR = 255;
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        telemetry.addData("Distance (cm)", String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
        telemetry.update();
    }
*/

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


    public void servoControl(double pos, int choice, int sleep) {

        switch (choice) {
            case 1:
                rightArm.setPosition(pos);
                if (rightArm.getPosition() == pos){
                    sleep(sleep);
                    break;
                }
            case 2:
                leftArm.setPosition(pos);
                if (leftArm.getPosition() == pos){
                    sleep(sleep);
                    break;
                }
            case 3:
                jemArm.setPosition(pos);
                if (jemArm.getPosition() == pos){
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
    public void init() {

        telemetry.addData("Current Status", "Initialized");
        telemetry.update();

        //Map The Motors To The Expansion Hub
        Motor0 = hardwareMap.get(DcMotor.class, "motor1");
        Motor1 = hardwareMap.get(DcMotor.class, "motor2");
        Motor2 = hardwareMap.get(DcMotor.class, "motor3");
        Motor3 = hardwareMap.get(DcMotor.class, "motor4");
        ArmLift = hardwareMap.get(DcMotor.class, "motor5");
        //sensorColor = hardwareMap.get(ColorSensor.class, "sensor");
        //sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor");
        rightArm = hardwareMap.servo.get("rightArm");
        leftArm = hardwareMap.servo.get("leftArm");
        jemArm = hardwareMap.servo.get("jemArm");
        //Set Motor Directions
        Motor0.setDirection(DcMotorSimple.Direction.FORWARD);
        Motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        Motor2.setDirection(DcMotorSimple.Direction.FORWARD);
        Motor3.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    @Override
    public void loop() {
        drive(1000, 1000,  1, false, 1);
        drive(1000, 1000,  1, false, 2);
    }
}
