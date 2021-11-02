package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;

//import org.firstinspires.ftc.teamcode.src.main.java.org.firstinspires.ftc.teamcode.DriveOnlyHardware;


@TeleOp(name="Testing Teleop", group="TeleopTest")

//@Disabled

public class TestTeleop extends OpMode {

    Hardware robot = new Hardware();

    private float drivePower = 0.65f;
    private float stickAxesThreshold = .1f;

    private float intakeMotorPower = .8f;
    //private float BRDrive = 1f;
    private float carouselMotorPower = .5f;

    private float pulleyMotorPower = .2f;

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
        // Pulley Motors: D-pad Up and D-pad Down
        if (gamepad1.dpad_up) {
            robot.pulleyMotorR.setPower(pulleyMotorPower);
            robot.pulleyMotorL.setPower(pulleyMotorPower);
        } else if (gamepad1.dpad_down) {
            robot.pulleyMotorR.setPower(-pulleyMotorPower);
            robot.pulleyMotorL.setPower(-pulleyMotorPower);
        } else {
            robot.pulleyMotorR.setPower(0);
            robot.pulleyMotorL.setPower(0);
        }

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
        if (Math.abs(gamepad1.right_stick_x) > stickAxesThreshold) {
            turn(gamepad1.right_stick_x);
        } else if (Math.abs(gamepad1.left_stick_y) > stickAxesThreshold) {
            drive(gamepad1.left_stick_y);
        } else {
            stopMotion();
        }
    }

    public void drive(double power) {
        robot.fLMotor.setPower(power);
        robot.fRMotor.setPower(power);
        robot.bLMotor.setPower(power);
        robot.bRMotor.setPower(power);
    }
    public void turn(double power) {
        robot.fLMotor.setPower(power);
        robot.bLMotor.setPower(power);
        robot.bRMotor.setPower(-power);
        robot.fRMotor.setPower(-power);
    }
    public void stopMotion() {
        drive(0);
    }
}