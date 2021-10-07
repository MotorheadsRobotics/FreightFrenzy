package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.HardwareIntake;

//import org.firstinspires.ftc.teamcode.src.main.java.org.firstinspires.ftc.teamcode.DriveOnlyHardware;


@TeleOp(name="BasicDrive&Intake", group="Teleop")

//@Disabled

public class DriveOnlyTeleop extends OpMode {

    Hardware robot = new Hardware();

    private float drivePower = 1.0f;
    private float turnPower = 1.0f;
    private float stickAxesThreshold = .4f;

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
    public void loop()

    {
        // Intake Motor: LB and RB
        if(gamepad1.left_bumper)
        {
            robot.intakeMotor.setPower(intakeMotorPower);
        }
        else if(gamepad1.right_bumper)
        {
            robot.intakeMotor.setPower(-intakeMotorPower);
        }
        else
        {
            robot.intakeMotor.setPower(0);
        }

        // Carousel Motor: LT and RT
        if(gamepad1.left_trigger > 0.3)
        {
            robot.carouselMotor.setPower(carouselMotorPower);
        }
        else if(gamepad1.right_trigger > 0.3)
        {
            robot.carouselMotor.setPower(-carouselMotorPower);
        }
        else
        {
            robot.carouselMotor.setPower(0);
        }

        // Forward Drive: Left Stick
        if(gamepad1.left_stick_y > stickAxesThreshold)
        {
            standardDrive(drivePower);
        }
        else if(gamepad1.left_stick_y < -stickAxesThreshold)
        {
            standardDrive(-drivePower);
        }
        else
        {
            standardDrive(0);
        }

        // Drive Turning: Right Stick
        if(gamepad1.right_stick_x > stickAxesThreshold)
        {
            standardDrive(-turnPower, turnPower);
        }
        else if(gamepad1.right_stick_x < -stickAxesThreshold)
        {
            standardDrive(turnPower, -turnPower);
        }
        else
        {
            standardDrive(0);
        }

        // Strafe :) - D-pad
        if(gamepad1.dpad_left)
        {
            strafeDrive(drivePower);
        }

        //mecanumMove();

    }

    public void mecanumMove()
    {
        //variables
        double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = -gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

//        robot.fLMotor.setPower(-drive * v1);
//        robot.fRMotor.setPower(-drive * v2);
//        robot.bLMotor.setPower(-drive * v3);
//        robot.bRMotor.setPower(-drive * v4);

        telemetry.addData("fLPower", -drivePower * v1);
        telemetry.addData("fRPower", -drivePower * v2);
        telemetry.addData("bLPower", -drivePower * v3);
        telemetry.addData("bRPower", -drivePower * v4);

//        telemetry.addData("Encoder port 1 back left",  robot.bLMotor.getCurrentPosition());
//        telemetry.addData("Encoder port 2 front right", robot.fRMotor.getCurrentPosition());
//        telemetry.addData("Encoder port 3 back right", robot.bRMotor.getCurrentPosition());

        //telemetry.addData("MagnetLimitSwitch", robot.magnetLimit.isPressed());

        telemetry.update();
    }

    public void standardDrive(float lPower, float rPower)
    {
        robot.fLMotor.setPower(lPower);
        robot.fRMotor.setPower(rPower);
        robot.bLMotor.setPower(lPower);
        robot.bRMotor.setPower(rPower);
    }

    private void strafeDrive(float drivePower)
    {
        int strafeTime = 150;
        runtime.reset();
        robot.bRMotor.setPower(0);
        robot.fRMotor.setPower(0);
        robot.bLMotor.setPower(0);
        robot.fLMotor.setPower(0);
        while (runtime.milliseconds() < strafeTime) {
            robot.bRMotor.setPower(-drivePower);
            robot.fRMotor.setPower(-drivePower);
        }
        robot.bRMotor.setPower(0);
        robot.fRMotor.setPower(0);
        runtime.reset();
        while (runtime.milliseconds() < strafeTime) {
            robot.bLMotor.setPower(-drivePower);
            robot.fLMotor.setPower(-drivePower);
        }
        robot.bLMotor.setPower(0);
        robot.fLMotor.setPower(0);
        runtime.reset();
        while (runtime.milliseconds() < strafeTime) {
            robot.bRMotor.setPower(drivePower);
            robot.fRMotor.setPower(drivePower);
        }
        robot.bRMotor.setPower(0);
        robot.fRMotor.setPower(0);
        runtime.reset();
        while (runtime.milliseconds() < strafeTime + 40) {
            robot.bLMotor.setPower(drivePower);
            robot.fLMotor.setPower(drivePower);
        }
        robot.bLMotor.setPower(0);
        robot.fLMotor.setPower(0);
    }

    public void standardDrive(float power)
    {
        robot.fLMotor.setPower(power);
        robot.fRMotor.setPower(power);
        robot.bLMotor.setPower(power);
        robot.bRMotor.setPower(power);
    }
}