package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.HardwarePartial;

//import org.firstinspires.ftc.teamcode.src.main.java.org.firstinspires.ftc.teamcode.DriveOnlyHardware;


@TeleOp(name="Final Teleop Solo Partial", group="TeleopTest")

//@Disabled

public class FinalTeleopSoloPartial extends OpMode {

    HardwarePartial robot = new HardwarePartial();

    private float drivePower = 0.9f;
    private float stickAxesThreshold = .1f;

    private float intakeMotorPower = 1.0f;
    //private float BRDrive = 1f;
    private float carouselMotorPower = 1.0f;

    private float pulleyMotorPower = 1.0f;
    private float pulleyMotorPowerDown = .65f;

    private int directionMult = 1;

    private boolean forwardDrive = true;
    private boolean previousBumper = false;

    private double startTime = 0;
    private double deltaTime = 0;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init()
    {
        //Initialize the hardware variables.
        //The init() method of the hardware class does all the work here
        robot.init(hardwareMap);

//        robot.bLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.bRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.fRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    @Override
    public void loop() {
        // Pulley Motors: D-pad Up and D-pad Down
        if (gamepad1.dpad_up) {
//            startTime = runtime.seconds();
//            deltaTime += runtime.seconds() - startTime;
            robot.pulleyMotorR.setPower(pulleyMotorPower);
//            robot.pulleyMotorL.setPower(pulleyMotorPower);
        } else if (gamepad1.dpad_down) {
//            startTime = runtime.seconds();
//            deltaTime += runtime.seconds() - startTime;
            robot.pulleyMotorR.setPower(-pulleyMotorPowerDown);
//            robot.pulleyMotorL.setPower(-pulleyMotorPower);
        } else {
            robot.pulleyMotorR.setPower(0);
//            robot.pulleyMotorL.setPower(0);
//            startTime = 0;
        }


        // Intake Motor: LB and RB
        if (gamepad1.left_bumper && gamepad1.x) {
            robot.intakeMotor.setPower(-intakeMotorPower);
        } else if (gamepad1.left_bumper) {
            robot.intakeMotor.setPower(intakeMotorPower);
        } else {
            robot.intakeMotor.setPower(0);
        }

        //these carousel functions are experimental because we need to use the gobilda
        //servo programmer to set them to be CR servos
        // Carousel Motor: LT and RT
        if (gamepad1.left_trigger > 0.3) {
            robot.lCarousel.setPower(carouselMotorPower);
            robot.rCarousel.setPower(carouselMotorPower);
        } else if (gamepad1.right_trigger > 0.3) {
            robot.lCarousel.setPower(-carouselMotorPower);
            robot.rCarousel.setPower(-carouselMotorPower);
        } else {
            robot.lCarousel.setPower(0);
            robot.rCarousel.setPower(0);
        }

         //Carousel Motor fast: LT and RT + x
        if(carouselMotorPower <= 0.5 && carouselMotorPower >= -0.5 && gamepad1.x) {
            if (gamepad1.left_trigger > 0.3) {
                robot.lCarousel.setPower(carouselMotorPower * 2);
                robot.rCarousel.setPower(carouselMotorPower * 2);
            } else if (gamepad1.right_trigger > 0.3) {
                robot.lCarousel.setPower(carouselMotorPower * 2);
                robot.rCarousel.setPower(carouselMotorPower * 2);
            } else {
                robot.lCarousel.setPower(0);
                robot.rCarousel.setPower(0);
            }
      }

        if(gamepad1.right_bumper && !previousBumper)
        {
            previousBumper = true;
            directionMult *= -1;
            forwardDrive = !forwardDrive;
            deltaTime = 0;
        }
        else if(!gamepad1.right_bumper)
        {
            previousBumper = false;
        }

        // Drive: Left and Right Stick
//        if (Math.abs(gamepad1.right_stick_x) > stickAxesThreshold) {
//            turn(gamepad1.right_stick_x);
//        }
//        else {
//            stopMotion();
//        }

        mecanumMove();

        telemetry.addData("Left Encoder", robot.bLMotor.getCurrentPosition());
        telemetry.addData("Mid Encoder", -robot.bRMotor.getCurrentPosition());
        telemetry.addData("Right Encoder", -robot.fRMotor.getCurrentPosition());

        // Close Hatch Servo: A
        if (gamepad1.a) {
            robot.bucketServo.setPosition(1);
        } else {
            robot.bucketServo.setPosition(0);
        }


        //telemetry.addData("Lift Limit", robot.liftLimit.getState());
        telemetry.addData("fLDist", robot.fLDist.getDistance(DistanceUnit.INCH));
        telemetry.addData("fRDist", robot.fRDist.getDistance(DistanceUnit.INCH));
        telemetry.addData("bLDist", robot.bLDist.getDistance(DistanceUnit.INCH));
        telemetry.addData("bRDist", robot.bRDist.getDistance(DistanceUnit.INCH));
        //telemetry.addData("DeltaTime", deltaTime);
        telemetry.update();
    }

    public void drive(double power) {
        if(gamepad1.b)
        {
            power*=0.2;
        }
        power*=directionMult;
        robot.fLMotor.setPower(power);
        robot.fRMotor.setPower(power);
        robot.bLMotor.setPower(power);
        robot.bRMotor.setPower(power);
    }
    public void turn(double power) {
        if(gamepad1.b)
        {
            power*=0.2;
        }
        robot.fLMotor.setPower(power);
        robot.bLMotor.setPower(power);
        robot.bRMotor.setPower(-power);
        robot.fRMotor.setPower(-power);
    }
    public void stopMotion() {
        drive(0);
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

        robot.fLMotor.setPower(-drivePower * v1 * directionMult);
        robot.fRMotor.setPower(-drivePower * v2 * directionMult);
        robot.bLMotor.setPower(-drivePower * v3 * directionMult);
        robot.bRMotor.setPower(-drivePower * v4 * directionMult);

//        telemetry.addData("fLPower", -drivePower * v1 * directionMult);
//        telemetry.addData("fRPower", -drivePower * v2 * directionMult);
//        telemetry.addData("bLPower", -drivePower * v3 * directionMult);
//        telemetry.addData("bRPower", -drivePower * v4 * directionMult);

//        telemetry.addData("Encoder port 1 back left",  robot.bLMotor.getCurrentPosition());
//        telemetry.addData("Encoder port 2 front right", robot.fRMotor.getCurrentPosition());
//        telemetry.addData("Encoder port 3 back right", robot.bRMotor.getCurrentPosition());

        //telemetry.addData("MagnetLimitSwitch", robot.magnetLimit.isPressed());

        telemetry.update();
    }
}