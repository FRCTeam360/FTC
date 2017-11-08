package org.firstinspires.ftc.teamcode;

//Include
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;




/**
 * Created by Gavin on 10/19/2017.
 */

@TeleOp(name="360 Main Drive Program", group="Linear OpMode")

public class TankMultiOP extends LinearOpMode {

    //Create Timer for runtime
    private ElapsedTime runtime = new ElapsedTime();

    //Motors
    private DcMotor RightTank;
    private DcMotor LeftTank;
    private DcMotor Lift;
    private DcMotor Hdrive;

    //Servos
    Servo RightGrip;
    Servo LeftGrip;

    @Override
    public void runOpMode() {
        //Motors


        //Servos
        RightGrip = hardwareMap.servo.get("RightGrip");
        LeftGrip = hardwareMap.servo.get("LeftGrip");

        RightTank.setDirection(DcMotorSimple.Direction.FORWARD);
        LeftTank.setDirection(DcMotorSimple.Direction.FORWARD);
        Lift.setDirection(DcMotorSimple.Direction.FORWARD);
        Hdrive.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Current Status: ", "Motors Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            boolean JoyStickMode = false;

            double RightDrive;
            double LeftDrive;
            double Lift;
            double Hdrive;

            if (JoyStickMode == false) {

                //Controller 1
                RightDrive = -gamepad1.left_stick_y;
                LeftDrive = gamepad1.right_stick_y;

                RightTank.setPower(RightDrive);
                LeftTank.setPower(LeftDrive);

                //Controller 2
                if (gamepad2.right_trigger >= 0.1) {

                    RightGrip.setPosition(0);
                    LeftGrip.setPosition(1);

                } else {

                    RightGrip.setPosition(1);
                    LeftGrip.setPosition(0);

                }

                if (gamepad1.start){

                    JoyStickMode = true;

                }
            }

            if (JoyStickMode == true) {

                RightDrive = -gamepad1.left_stick_y;
                LeftDrive = gamepad1.left_stick_y;

                RightTank.setPower(RightDrive);
                LeftTank.setPower(LeftDrive);

                //Controller 2
                if (gamepad2.right_trigger >= 0.1) {

                    RightGrip.setPosition(0);
                    LeftGrip.setPosition(1);

                } else {

                    RightGrip.setPosition(1);
                    LeftGrip.setPosition(0);

                }

                if (gamepad1.start){

                    JoyStickMode = false;

                }

            }
            idle();
        }
    }
}

