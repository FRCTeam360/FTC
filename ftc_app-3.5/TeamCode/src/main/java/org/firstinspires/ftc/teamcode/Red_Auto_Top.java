package org.firstinspires.ftc.teamcode;

/**
 * Created by Eric on 4/20/69
 */
import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.Locale;


/**
 * Created by Gavin on 11/14/2017.
 */

@Autonomous(name="Red_Auto_Top", group="Linear Opmode")
@Disabled
public class Red_Auto_Top extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    double leftPower;
    double rightPower;
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    //private DcMotor liftDrive;

    //Servo leftArm;
    //Servo rightArm;
    Servo jemArm;
    Servo colorArm;
    //Servo blockArm;

    ColorSensor sensorColor;
    DistanceSensor sensorDistance;

    public void armGrip(int direction) {
        switch (direction){
            case 1:
                servo(-1, 3, 500);
                servo(1, 4, 500);
                break;
            case 2:
                servo(1, 3, 500);
                servo(-1, 4, 500);
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
        leftPower = power;
        rightPower = power;
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        sleep(100);

    }


    public void servo(double pos, int choice, int sleep) {

        switch (choice) {
            /*case 1:
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
                }*/
            case 3:
                jemArm.setPosition(pos);
                if (jemArm.getPosition() == pos) {
                    sleep(sleep);
                    break;
                }
            case 4:
                colorArm.setPosition(pos);
                if (colorArm.getPosition() == pos) {
                    sleep(sleep);
                    break;
                }
        }
    }


    public void drive(int timeval, int timeafterval,  double power, int choice) {

        leftPower = power;
        rightPower = power;


        switch (choice) {
            case 1:
                leftDrive.setPower(leftPower);
                rightDrive.setPower(-rightPower);
                break;
            case 2:
                rightDrive.setPower(-leftPower);
                rightDrive.setPower(rightPower);
                break;
            case 3:
                leftDrive.setPower(leftPower);
                rightDrive.setPower(rightPower);
                break;
            case 4:
                leftDrive.setPower(-leftPower);
                rightDrive.setPower(-rightPower);
                break;
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
        leftDrive = hardwareMap.get(DcMotor.class, "left");
        rightDrive = hardwareMap.get(DcMotor.class, "right");
        //liftDrive = hardwareMap.get(DcMotor.class, "lift");
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor");
        //rightArm = hardwareMap.servo.get("rightArm");
        //leftArm = hardwareMap.servo.get("leftArm");
        jemArm = hardwareMap.servo.get("jemArm");
        colorArm = hardwareMap.servo.get("colorArm");

        //Set Motor Directions
        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        float hsvValues[] = {0F, 0F, 0F};
        boolean colorFound = false;

        telemetry.addData("Data", colorArm.getPosition());
        telemetry.update();


        waitForStart();


        while (opModeIsActive()) {

            telemetry.addData("Jem arm position", jemArm.getPosition());
            telemetry.addData("Color arm position", colorArm.getPosition());
            telemetry.update();
            sleep(3000);
            //drive(100, 3000, .5, 1);

            servo(0.7, 3, 1000);
            telemetry.addLine("First");
            telemetry.addData("Jem arm position", jemArm.getPosition());
            telemetry.addData("Color arm position", colorArm.getPosition());
            telemetry.update();
            sleep(3000);

            servo(0.5, 4, 1000);
            telemetry.addLine("Second");
            telemetry.addData("Jem arm position", jemArm.getPosition());
            telemetry.addData("Color arm position", colorArm.getPosition());
            telemetry.update();
            sleep(3000);

            while (colorFound == false && opModeIsActive()) {
                colorDistance(1, 100, hsvValues);
                if (sensorColor.red() >= 20 && sensorColor.red() > sensorColor.blue()) {
                    telemetry.addLine("Red");
                    telemetry.update();
                    sleep(3000);
                    servo(1, 3, 1000);
                    //servo(1, 4, 1000);
                    drive(3,0,1,3);
                    colorFound = true;
                }
                else if (sensorColor.blue() >= 20 && sensorColor.blue() > sensorColor.red()) {
                    telemetry.addLine("Blue");
                    telemetry.update();
                    servo(0, 3, 1000);
                    //servo(1, 4, 1000);
                    drive(3,0,1,3);
                    colorFound = true;
                }
            }
            break;
        }

        stop();

    }
}

