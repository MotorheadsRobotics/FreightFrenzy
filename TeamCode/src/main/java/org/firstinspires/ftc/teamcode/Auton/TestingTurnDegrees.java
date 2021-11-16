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

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

@Autonomous(name="Testing Turn Degrees", group="test")
//@Disabled
public class TestingTurnDegrees extends AutonDriving {

    /* Declare OpMode members. */
    Hardware robot = new Hardware();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     FORWARD_SPEED = 0.5;
    static double     LAUNCHER_SPEED = 0.62;

    public String xyz = "z";

    public static final double     COUNTS_PER_MOTOR_REV = 384.5;    //Gobilda something motors 5204?
    public static final double     DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP //On OUR CENTER MOTOR THE GEAR REDUCTION IS .5
    public static final double     WHEEL_DIAMETER_INCHES = 4.65;     // For figuring circumference
    public static final double     COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    BNO055IMU imu;

    public TestingTurnDegrees() {
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

        turnDegrees(90, xyz, .4, 5000);
        turnDegrees(-90, xyz, .4, 5000);
        turnDegrees(-120, xyz, .4, 5000);
        turnDegrees(-120, xyz, .4, 5000);
        turnDegrees(-120, xyz, .4, 5000);
    }
    public void turnDegrees(double degrees, String xyz, double maxSpeed, double timeoutMS) {
        if (Math.abs(degrees) <= 180  && Math.abs(maxSpeed) <= 1) {
            double originalAngle = readAngle(xyz);
            double target = originalAngle + degrees;
            double originalError = findDistanceOfAngle(originalAngle, target);
            double error = target - originalAngle;
            if (Math.abs(target) > 180) {
                if (target < 0) {
                    target += 360;
                } else {
                    target -= 360;
                }
            }
            runtime.reset();
            while (runtime.milliseconds() < timeoutMS) {
                double angle = readAngle(xyz);
                error = findDistanceOfAngle(angle, target);
                double powerScaled = maxSpeed * ((error)/(originalError));
                if (Math.abs(error) < 2) {
                    break;
                }
                if (error < 0) {
                    normalDrive(powerScaled, -powerScaled, false);
                }
                else {
                    normalDrive(-powerScaled, powerScaled, false);
                }
            }
        }
        normalDrive(0,0,false);
        sleep(250);
    }
    public void normalDrive(double lpower, double rpower, boolean encoder)
    {

        if (opModeIsActive()) {
            //stopAndReset();
            if(encoder)
            {
                //TryMotors();
                robot.fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.bLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.bRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.fLMotor.setPower(lpower);
                robot.fRMotor.setPower(rpower);
                robot.bLMotor.setPower(lpower);
                robot.bRMotor.setPower(rpower);
            }
            else
            {
                //TryMotors();

                robot.fLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.fRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.bLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.bRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.fLMotor.setPower(lpower);
                robot.fRMotor.setPower(rpower);
                robot.bLMotor.setPower(lpower);
                robot.bRMotor.setPower(rpower);
            }
        }
    }
    public double findDistanceOfAngle(double currentAngle, double targetAngle) {
        double error = targetAngle - currentAngle;
        if (Math.abs(error) > 180) {
            if (error < 0) {
                error += 360;
            }
            else {
                error -= 360;
            }
        }
        return error;
    }
    public double readAngle(String xyz) {
        //Orientation angles;
        Acceleration gravity;
        //angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        updateAngles();
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
    public void encoderDrive(double speed, char direction, double inches, double timeoutS) {

        int newFrontLeftTarget = 0;
        int newBackLeftTarget = 0;
        int newFrontRightTarget = 0;
        int newBackRightTarget = 0;

        stopAndReset();

        int fLOriginal = robot.fLMotor.getCurrentPosition();
        int fROriginal = robot.fLMotor.getCurrentPosition();
        int bLOriginal = robot.fLMotor.getCurrentPosition();
        int bROriginal = robot.fLMotor.getCurrentPosition();

        int targetCounts = (int)(inches * COUNTS_PER_INCH);

        //int error = getErrorEncoder(speed);

        //int error =

        boolean directionIsTrue = true;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            switch (direction) {
                case 'f':
                    //these four statements original had (- error) appended. Seems like a jank fix
                    newFrontLeftTarget = robot.fLMotor.getCurrentPosition() + targetCounts;
                    newFrontRightTarget = robot.fRMotor.getCurrentPosition() + targetCounts;
                    newBackLeftTarget = robot.bLMotor.getCurrentPosition() + targetCounts;
                    newBackRightTarget = robot.bRMotor.getCurrentPosition() + targetCounts;
                    robot.fLMotor.setTargetPosition(newFrontLeftTarget);
                    robot.fRMotor.setTargetPosition(newFrontRightTarget);
                    robot.bLMotor.setTargetPosition(newBackLeftTarget);
                    robot.bRMotor.setTargetPosition(newBackRightTarget);
                    break;
                case 'b':
                    //same as f
                    newFrontLeftTarget = robot.fLMotor.getCurrentPosition() - targetCounts;
                    newFrontRightTarget = robot.fRMotor.getCurrentPosition() - targetCounts;
                    newBackLeftTarget = robot.bLMotor.getCurrentPosition() - targetCounts;
                    newBackRightTarget = robot.bRMotor.getCurrentPosition() - targetCounts;
                    robot.fLMotor.setTargetPosition(newFrontLeftTarget);
                    robot.fRMotor.setTargetPosition(newFrontRightTarget);
                    robot.bLMotor.setTargetPosition(newBackLeftTarget);
                    robot.bRMotor.setTargetPosition(newBackRightTarget);
                    break;
                /*case 'l':
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
                    break;*/
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


                int fRError = targetCounts - robot.fRMotor.getCurrentPosition();
                int fLError = targetCounts - robot.fLMotor.getCurrentPosition();
                int bRError = targetCounts - robot.bRMotor.getCurrentPosition();
                int bLError = targetCounts - robot.bLMotor.getCurrentPosition();

                int errors[] = {fRError, fLError, bRError, bLError};

                double fRSpeedAdjust = 1, fLSpeedAdjust = 1, bRSpeedAdjust = 1, bLSpeedAdjust = 1;
                double speedAdjusts[] = {fRSpeedAdjust, fLSpeedAdjust, bRSpeedAdjust, bLSpeedAdjust};

                int inchSlowThresh = 5;



                int fRInchesTrav = (int) Math.abs(((robot.fRMotor.getCurrentPosition() - fROriginal) / COUNTS_PER_INCH));
                int fLInchesTrav = (int) Math.abs(((robot.fLMotor.getCurrentPosition() - fLOriginal) / COUNTS_PER_INCH));
                int bRInchesTrav = (int) Math.abs(((robot.bRMotor.getCurrentPosition() - bROriginal) / COUNTS_PER_INCH));
                int bLInchesTrav = (int) Math.abs(((robot.bLMotor.getCurrentPosition() - bLOriginal) / COUNTS_PER_INCH));

                int inchesTrav[] = {fRInchesTrav, fLInchesTrav, bRInchesTrav, bLInchesTrav};

                for(int i = 0; i < errors.length; i++)
                {
                    speedAdjusts[i] = Math.abs((errors[i]/COUNTS_PER_INCH)/inches);
                    if(speedAdjusts[i] > 1 && inchesTrav[i] > 1)
                    {
                        speedAdjusts[i] = inchesTrav[i]/speedAdjusts[i];
                    }
                }

                /*
                fRSpeedAdjust = Math.abs((fRError/COUNTS_PER_INCH)/inches);
                if(fRSpeedAdjust > 1)
                {
                    fRSpeedAdjust = fRInchesTrav/(fRSpeedAdjust);
                }
                fLSpeedAdjust = Math.abs((fLError/COUNTS_PER_INCH)/inches);
                if(fLSpeedAdjust > 1)
                {
                    fLSpeedAdjust = fLInchesTrav/(fLSpeedAdjust);
                }
                bRSpeedAdjust = Math.abs((bRError/COUNTS_PER_INCH)/inches);
                if(bRSpeedAdjust > 1)
                {
                    bRSpeedAdjust = bRInchesTrav/(bRSpeedAdjust);
                }
                bLSpeedAdjust = Math.abs((bLError/COUNTS_PER_INCH)/inches);
                if(bLSpeedAdjust > 1)
                {
                    bLSpeedAdjust = bLInchesTrav/(bLSpeedAdjust);
                }
                */


                telemetry.addData("Speed Adjust FR", fRSpeedAdjust);
                telemetry.addData("Speed Adjust FL", fLSpeedAdjust);
                telemetry.addData("Speed Adjust BR", bRSpeedAdjust);
                telemetry.addData("Speed Adjust BL", bLSpeedAdjust);
                telemetry.update();

                robot.fLMotor.setPower(Math.abs(speed) * fLSpeedAdjust); // * fLError
                robot.fRMotor.setPower(Math.abs(speed) * fRSpeedAdjust); // * fRError
                robot.bLMotor.setPower(Math.abs(speed) * bLSpeedAdjust); // * bLError
                robot.bRMotor.setPower(Math.abs(speed) * bRSpeedAdjust); // * bRError

                int countThresh = 10;
                if (Math.abs(newFrontLeftTarget - robot.fLMotor.getCurrentPosition()) < countThresh ||
                        Math.abs(newFrontRightTarget - robot.fRMotor.getCurrentPosition()) < countThresh ||
                        Math.abs(newBackLeftTarget - robot.bLMotor.getCurrentPosition()) < countThresh ||
                        Math.abs(newBackRightTarget - robot.bRMotor.getCurrentPosition()) < countThresh) {
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



            sleep(100);   // optional pause after each move
        }
    }
}