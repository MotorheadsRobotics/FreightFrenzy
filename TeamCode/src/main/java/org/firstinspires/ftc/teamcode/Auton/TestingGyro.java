/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

import java.util.List;

@Autonomous(name="Testing Auton Gyro", group="test")
//@Disabled
public class TestingGyro extends AutonDriving {

    /* Declare OpMode members. */
    Hardware robot = new Hardware();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     FORWARD_SPEED = 0.5;
    static double     LAUNCHER_SPEED = 0.62;

    public String xyz = "z";

    public static final double     COUNTS_PER_MOTOR_REV = 383.6;    // Currently: Andymark Neverest 40
    public static final double     COUNTS_PER_REV_ARM = 1440;
    public static final double     COUNTS_PER_INCH_ARM = COUNTS_PER_REV_ARM/4;
    public static final double     DRIVE_GEAR_REDUCTION = .666;     // This is < 1.0 if geared UP //On OUR CENTER MOTOR THE GEAR REDUCTION IS .5
    public static final double     WHEEL_DIAMETER_INCHES = 3.7795;     // For figuring circumference
    public static final double     COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    BNO055IMU imu;

    public TestingGyro() {
    }

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        robot.init(hardwareMap);

        waitForStart();

        runtime.reset();

        turnToPosition(90, "z", 0.2, 5, true);
        sleep(500);
        turnToPosition(0, "z", 0.5, 5, true);
    }
    public void turnToPosition (double target, String xyz, double topPower, double timeoutS, boolean isCorrection) {
        //Write code to correct to a target position (NOT FINISHED)
        target *= -1;
        double originalAngle = readAngle(xyz);


        runtime.reset();

        double angle = readAngle(xyz); //variable for gyro correction around z axis
        double error = angle - target;
        double powerScaled = topPower;
        do {
            angle = readAngle(xyz);
            error = angle - target;
            if (!isCorrection) {
                powerScaled = topPower * error/(originalAngle - target);
            }

            //double powerScaled = power*pidMultiplier(error);
            telemetry.addData("original angle", originalAngle);
            telemetry.addData("target", target);
            telemetry.addData("PowerScaled", powerScaled);
            telemetry.addData("current angle", readAngle(xyz));
            telemetry.addData("error", error);
            telemetry.update();
            if (error > 0) {
                if (xyz.equals("z")) {
                    normalDrive(powerScaled, -powerScaled);
                }
            } else if (error < 0) {
                if (xyz.equals("z")) {
                    normalDrive(-powerScaled, powerScaled);
                }
            }
        } while (opModeIsActive() && (Math.abs(error) > 0.3) && (runtime.seconds() < timeoutS));
        normalDrive(0, 0);

    }

    public double pidMultiplierDriving(double error) {
        //equation for power multiplier is x/sqrt(x^2 + C)
        int C = 100;
        return Math.abs(error / Math.sqrt((error * error) + C));
    }
    public double pidMultiplierTurning(double error) {
        //equation for power multiplier is x/sqrt(x^2 + C)
        double C = .1;
        return Math.abs(error / Math.sqrt((error * error) + C));
    }
    public void pathComplete(int millisec)
    {
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(millisec);
    }
    public double readAngle(String xyz) {
        Orientation angles;
//        Acceleration gravity;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if (xyz.equals("x")) {
            return angles.thirdAngle;
        } else if (xyz.equals("y")) {
            return angles.secondAngle;
        } else if (xyz.equals("z")) {
            return angles.firstAngle;
        } else {
            return 0;
        }
    }
    public void normalDrive(double lpower, double rpower) {

        if (opModeIsActive()) {
            robot.fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fLMotor.setPower(lpower);
            robot.fRMotor.setPower(rpower);
            robot.bLMotor.setPower(lpower);
            robot.bRMotor.setPower(rpower);
        }
    }
    public void encoderDrive(double speed, char direction, double inches, double timeoutS) {

        int newFrontLeftTarget = 0;
        int newBackLeftTarget = 0;
        int newFrontRightTarget = 0;
        int newBackRightTarget = 0;

        int error = getSpeedError(speed);

        boolean directionIsTrue = true;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            switch (direction) {
                case 'f':
                    newFrontLeftTarget = robot.fLMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH) - error;
                    newFrontRightTarget = robot.fRMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH)- error;
                    newBackLeftTarget = robot.bLMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH) - error;
                    newBackRightTarget = robot.bRMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH) - error;
                    robot.fLMotor.setTargetPosition(newFrontLeftTarget);
                    robot.fRMotor.setTargetPosition(newFrontRightTarget);
                    robot.bLMotor.setTargetPosition(newBackLeftTarget);
                    robot.bRMotor.setTargetPosition(newBackRightTarget);
                    break;
                case 'b':
                    newFrontLeftTarget = robot.fLMotor.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH) + error;
                    newFrontRightTarget = robot.fRMotor.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH) + error;
                    newBackLeftTarget = robot.bLMotor.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH) + error;
                    newBackRightTarget = robot.bRMotor.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH) + error;
                    robot.fLMotor.setTargetPosition(newFrontLeftTarget);
                    robot.fRMotor.setTargetPosition(newFrontRightTarget);
                    robot.bLMotor.setTargetPosition(newBackLeftTarget);
                    robot.bRMotor.setTargetPosition(newBackRightTarget);
                    break;
                case 'l':
                    newFrontLeftTarget = robot.fLMotor.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH) + error;
                    newFrontRightTarget = robot.fRMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH) - error;
                    newBackLeftTarget = robot.bLMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH) - error;
                    newBackRightTarget = robot.bRMotor.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH) + error;
                    robot.fLMotor.setTargetPosition(newFrontLeftTarget);
                    robot.fRMotor.setTargetPosition(newFrontRightTarget);
                    robot.bLMotor.setTargetPosition(newBackLeftTarget);
                    robot.bRMotor.setTargetPosition(newBackRightTarget);
                    break;
                case 'r':
                    newFrontLeftTarget = robot.fLMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH) - error;
                    newFrontRightTarget = robot.fRMotor.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH) + error;
                    newBackLeftTarget = robot.bLMotor.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH) + error;
                    newBackRightTarget = robot.bRMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH) - error;
                    robot.fLMotor.setTargetPosition(newFrontLeftTarget);
                    robot.fRMotor.setTargetPosition(newFrontRightTarget);
                    robot.bLMotor.setTargetPosition(newBackLeftTarget);
                    robot.bRMotor.setTargetPosition(newBackRightTarget);
                    break;
                default:
                    directionIsTrue = false;
            }
            // Determine new target position, and pass to motor controller

            // Turn On RUN_TO_POSITION
            robot.fLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.fRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            if (directionIsTrue) {
                robot.fLMotor.setPower(Math.abs(speed));
                robot.fRMotor.setPower(Math.abs(speed));
                robot.bLMotor.setPower(Math.abs(speed));
                robot.bRMotor.setPower(Math.abs(speed));
            }

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (directionIsTrue) &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.fLMotor.isBusy() && robot.fRMotor.isBusy() && robot.bLMotor.isBusy() && robot.bRMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Target Positions",  "Running to fL: %7d fR: %7d bL: %7d bR: %7d",
                        newFrontLeftTarget,
                        newFrontRightTarget,
                        newBackLeftTarget,
                        newBackRightTarget);
                telemetry.addData("Current Positions",  "Running at fL: %7d fR: %7d bL: %7d bR: %7d",
                        robot.fLMotor.getCurrentPosition(),
                        robot.fRMotor.getCurrentPosition(),
                        robot.bLMotor.getCurrentPosition(),
                        robot.bRMotor.getCurrentPosition());
                telemetry.update();

                if (Math.abs(newFrontLeftTarget - robot.fLMotor.getCurrentPosition()) < 50 ||
                        Math.abs(newFrontRightTarget - robot.fRMotor.getCurrentPosition()) < 50 ||
                        Math.abs(newBackLeftTarget - robot.bLMotor.getCurrentPosition()) < 50 ||
                        Math.abs(newBackRightTarget - robot.bRMotor.getCurrentPosition()) < 50) {
                    break;
                }
            }

            // Stop all motion;
            robot.fLMotor.setPower(0);
            robot.fRMotor.setPower(0);
            robot.bLMotor.setPower(0);
            robot.bRMotor.setPower(0);

            // Turn off RUN_TO_POSITIOn

            robot.fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



            sleep(500);   // optional pause after each move
        }
    }
    public int getSpeedError(double speed) { //me being stupid
        return (int) (speed * 200);
    }
}