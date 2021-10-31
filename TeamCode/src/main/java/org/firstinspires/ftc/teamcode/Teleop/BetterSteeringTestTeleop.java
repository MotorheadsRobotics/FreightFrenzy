package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;

//import org.firstinspires.ftc.teamcode.src.main.java.org.firstinspires.ftc.teamcode.DriveOnlyHardware;


@TeleOp(name="Testing Driving but betterer", group="TeleopTest")

@Disabled

public class BetterSteeringTestTeleop extends OpMode {

    Hardware robot = new Hardware();

    private float stickAxesThreshold = .2f;

    private float intakeMotorPower = .8f;
    //private float BRDrive = 1f;
    private float carouselMotorPower = .5f;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init()
    {
        //Initialize the hardware variables.
        //The init() method of the hardware class does all the work here
        robot.init(hardwareMap);


    }

    @Override
    public void loop() {
        // Intake Motor: LB and RB
        if (gamepad1.left_bumper) {
            robot.intakeMotor.setPower(intakeMotorPower);
        } else if (gamepad1.right_bumper) {
            robot.intakeMotor.setPower(-intakeMotorPower);
        } else {
            robot.intakeMotor.setPower(0);
        }

        // Carousel Motor: LT and RT
        if (gamepad1.left_trigger > 0.3) {
            robot.carouselMotor.setPower(carouselMotorPower);
        } else if (gamepad1.right_trigger > 0.3) {
            robot.carouselMotor.setPower(-carouselMotorPower);
        } else {
            robot.carouselMotor.setPower(0);
        }
        drive();
    }

    public void drive() {
        if (Math.abs(gamepad1.left_stick_y) > stickAxesThreshold || Math.abs(gamepad1.right_stick_x) > stickAxesThreshold) {
            double power = gamepad1.left_stick_y;
            power *= .85;
            double turn = gamepad1.right_stick_x;
            if (turn < 0) {
                turn = -Math.sqrt(Math.abs(turn));
            }
            else {
                turn = Math.sqrt(Math.abs(turn));
            }
            turn *= .7;
            double rightPower = power + turn;
            double leftPower = power - turn;
            telemetry.addData("Right Power", rightPower);
            telemetry.addData("Left Power", leftPower);
            telemetry.update();
            robot.fRMotor.setPower(rightPower);
            robot.fLMotor.setPower(leftPower);
            robot.bLMotor.setPower(leftPower);
            robot.bRMotor.setPower(rightPower);
        }
        else {
            robot.fRMotor.setPower(0);
            robot.fLMotor.setPower(0);
            robot.bLMotor.setPower(0);
            robot.bRMotor.setPower(0);
        }
    }
}