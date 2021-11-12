package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

//import org.firstinspires.ftc.teamcode.src.main.java.org.firstinspires.ftc.teamcode.DriveOnlyHardware;


@TeleOp(name="Final Teleop", group="TeleopTest")

//@Disabled

public class FinalTeleop extends OpMode {

    Hardware robot = new Hardware();

    private float drivePower = 0.65f;
    private float stickAxesThreshold = .1f;

    private float intakeMotorPower = .8f;
    //private float BRDrive = 1f;
    private float carouselMotorPower = .8f;

    private float pulleyMotorPower = .65f;

    private int directionMult = 1;

    private boolean forwardDrive = true;
    private boolean previousBumper = false;

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
        if (gamepad2.left_bumper) {
            robot.intakeMotor.setPower(-intakeMotorPower);
        } else if (gamepad2.right_bumper) {
            robot.intakeMotor.setPower(intakeMotorPower);
        } else {
            robot.intakeMotor.setPower(0);
        }

        // Carousel Motor: LT and RT
        if (gamepad2.left_trigger > 0.3) {
            robot.carouselMotor.setPower(carouselMotorPower);
        } else if (gamepad2.right_trigger > 0.3) {
            robot.carouselMotor.setPower(-carouselMotorPower);
        } else {
            robot.carouselMotor.setPower(0);
        }
        // Carousel Motor fast: LT and RT + x
//        if(carouselMotorPower <= 0.5 && carouselMotorPower >= -0.5 && gamepad1.x) {
//            if (gamepad1.left_trigger > 0.3) {
//                robot.carouselMotor.setPower(carouselMotorPower * 2);
//            } else if (gamepad1.right_trigger > 0.3) {
//                robot.carouselMotor.setPower(-carouselMotorPower * 2);
//            } else {
//                robot.carouselMotor.setPower(0);
//            }
//      }

        if(gamepad1.right_bumper && !previousBumper)
        {
            previousBumper = true;
            directionMult *= -1;
            forwardDrive = !forwardDrive;

        }
        else if(!gamepad1.right_bumper)
        {
            previousBumper = false;
        }

        // Drive: Left and Right Stick
        if (Math.abs(gamepad1.right_stick_x) > stickAxesThreshold) {
            turn(gamepad1.right_stick_x);
        } else if (Math.abs(gamepad1.left_stick_y) > stickAxesThreshold) {
            drive(-gamepad1.left_stick_y);
        } else {
            stopMotion();
        }

        // Close Hatch Servo: A
        if (gamepad2.a) {
            robot.bucketServo.setPosition(1);
        } else {
            robot.bucketServo.setPosition(0);
        }


        //telemetry.addData("Lift Limit", robot.liftLimit.getState());
        telemetry.addData("fLDist", robot.fLDist.getDistance(DistanceUnit.INCH));
        telemetry.addData("fRDist", robot.fRDist.getDistance(DistanceUnit.INCH));
        telemetry.addData("bLDist", robot.bLDist.getDistance(DistanceUnit.INCH));
        telemetry.addData("bRDist", robot.bRDist.getDistance(DistanceUnit.INCH));
        telemetry.update();
    }

    public void drive(double power) {
        if(gamepad1.left_bumper)
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
        if(gamepad1.left_bumper)
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
}