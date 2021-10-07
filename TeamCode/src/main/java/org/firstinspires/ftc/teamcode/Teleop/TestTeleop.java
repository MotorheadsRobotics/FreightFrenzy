package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;

//import org.firstinspires.ftc.teamcode.src.main.java.org.firstinspires.ftc.teamcode.DriveOnlyHardware;


@TeleOp(name="Testing Driving", group="Teleop")

//@Disabled

public class TestTeleop extends OpMode {

    Hardware robot = new Hardware();

    private float drivePower = 0.8f;
    private float stickAxesThreshold = .1f;

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

        // Forward Drive: Left Stick
        if (gamepad1.left_stick_y > stickAxesThreshold) {
            drive(drivePower);
        } else if (gamepad1.left_stick_y < -stickAxesThreshold) {
            drive(-drivePower);
        } else {
            stopMotion();
        }
    }

    public void drive(double power) {
        double turn = gamepad1.right_stick_x;
        turn /= 2;
        if (Math.abs(turn) > stickAxesThreshold) {
            if (turn > 0) {
                robot.fLMotor.setPower(power + turn);
                robot.fRMotor.setPower(power);
                robot.bLMotor.setPower(power + turn);
                robot.bRMotor.setPower(power);
            }
            else {
                robot.fLMotor.setPower(power);
                robot.fRMotor.setPower(power + turn);
                robot.bLMotor.setPower(power);
                robot.bRMotor.setPower(power + turn);
            }
        }
        else {
            robot.fLMotor.setPower(power);
            robot.fRMotor.setPower(power);
            robot.bLMotor.setPower(power);
            robot.bRMotor.setPower(power);
        }
    }
    public void stopMotion() {
        drive(0);
    }
}