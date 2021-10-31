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
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

@Autonomous(name="Auton Red Far", group="Test")
//@Disabled
public class AutonRedFar extends AutonDriving {

    /* Declare OpMode members. */
    //org.firstinspires.ftc.teamcode.Hardware.Hardware robot = new org.firstinspires.ftc.teamcode.Hardware.Hardware();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     FORWARD_SPEED = 0.5;
    static double     LAUNCHER_SPEED = 0.62;

    private boolean objectInVision = false;

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    public String xyz = "z";

    public static final double     COUNTS_PER_MOTOR_REV = 384.5;    // Currently: Andymark Neverest 40
    public static final double     COUNTS_PER_REV_ARM = 1440;
    public static final double     COUNTS_PER_INCH_ARM = COUNTS_PER_REV_ARM/4;
    public static final double     DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP //On OUR CENTER MOTOR THE GEAR REDUCTION IS .5
    public static final double     WHEEL_DIAMETER_INCHES = 4.65;     // For figuring circumference
    public static final double     COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    private String ringLabel = " ";

    BNO055IMU imu;

    public static final String VUFORIA_KEY =
            "AYy6NYn/////AAABmTW3q+TyLUMbg/IXWlIG3BkMMq0okH0hLmwj3CxhPhvUlEZHaOAmESqfePJ57KC2g6UdWLN7OYvc8ihGAZSUJ2JPWAsHQGv6GUAj4BlrMCjHvqhY0w3tV/Azw2wlPmls4FcUCRTzidzVEDy+dtxqQ7U5ZtiQhjBZetAcnLsCYb58dgwZEjTx2+36jiqcFYvS+FlNJBpbwmnPUyEEb32YBBZj4ra5jB0v4IW4wYYRKTNijAQKxco33VYSCbH0at99SqhXECURA55dtmmJxYpFlT/sMmj0iblOqoG/auapQmmyEEXt/T8hv9StyirabxhbVVSe7fPsAueiXOWVm0kCPO+KN/TyWYB9Hg/mSfnNu9i9";

    public AutonRedFar() {
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

//        this.initVuforia(); //this should ensure that it calls the Vuforia of this class not the one from the AutonDrivingWIP class. This is a test given an error that appeared to happen during Vuforia initialization.
//        initTfod();
//
//        if (tfod != null) {
//            tfod.activate();
//        }

        robot.init(hardwareMap);

        waitForStart();

        //encoderDrive(0.275, 'f', 14, 5);
        /*turnToPosition(90, "z", .15, 5);
        encoderDrive(0.5, 'f', 5, 10);
        turnToPosition(0, "z", 0.15, 5); // position is absolute, turnDegrees is relative
        encoderDrive(0.5, 'b', 5, 10);
        */
        encoderDrive(.5, 'f', 5, 5);
        turnToPosition(100, "z", .5, 5);
        encoderDrive(.5, 'f', 5, 5);
        CarouselSpin(.5, true, 2);
        encoderDrive(.5, 'b', 5, 5);
        turnToPosition(90, "z", .5, 5);
        encoderDrive(1.0, 'b', 40, 5);
//        if (opModeIsActive()) {
//            runtime.reset();
//            do {
//                telemetry.addData("Runtime", runtime.milliseconds());
//                if (tfod != null) {
//                    // getUpdatedRecognitions() will return null if no new information is available since
//                    // the last time that call was made.
//                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
//                    if (updatedRecognitions != null) {
//                        telemetry.addData("# Object Detected", updatedRecognitions.size());
//                        // step through the list of recognitions and display boundary info.
//                        int i = 0;
//                        for (Recognition recognition : updatedRecognitions) {
//                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
//
//                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f", recognition.getLeft(), recognition.getTop());
//                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f", recognition.getRight(), recognition.getBottom());
//
//                            telemetry.update();
//
//                            if (recognition.getLabel().equals("Quad") || recognition.getLabel().equals("Single")) {
//                                ringLabel = recognition.getLabel();
//                                objectInVision = true;
//                            }
//                        }
//                    }
//                }
//                telemetry.update();
//            } while ((runtime.milliseconds() < 5000 && !(objectInVision)) || runtime.milliseconds() < 1000);
//        }
//        if (ringLabel.equals("Quad")) {
//            telemetry.addData("Target Zone", "C");
//            telemetry.update();
//            encoderDrive(FORWARD_SPEED,'f',100,10);
//            turnToPosition(90,xyz,0.8,2.5,false);
//            encoderDrive(FORWARD_SPEED,'f',24,5);
//            turnToPosition(0, xyz, 0.8, 2.5, false);
//            encoderDrive(FORWARD_SPEED, 'b', 52, 4);
//            encoderDrive(FORWARD_SPEED, 'l', 32, 4);
//            turnToPosition(-7, xyz, 0.8, 2, false);
//        }
//        else if (ringLabel.equals("Single")) {
//            telemetry.addData("Target Zone", "B");
//            telemetry.update();
//            encoderDrive(FORWARD_SPEED,'f',81,7);
//            turnToPosition(90,xyz,0.8,2.5,false);
//            encoderDrive(FORWARD_SPEED,'f',8,5);
//            turnToPosition(0, xyz, 0.8, 2.5, false);
//            encoderDrive(FORWARD_SPEED, 'b', 36, 4);
//            turnToPosition(-7, xyz, 0.8, 2, false);
//        }
//        else {
//            telemetry.addData("Target Zone", "A");
//            telemetry.update();
//            encoderDrive(FORWARD_SPEED,'f',52,5);
//            turnToPosition(90,xyz,0.8,2.5,false);
//            encoderDrive(FORWARD_SPEED,'f',24,5);
//            turnToPosition(0, xyz, 0.8, 2.5, false);
//            encoderDrive(FORWARD_SPEED, 'b', 10, 4);
//            encoderDrive(FORWARD_SPEED, 'l', 40, 4);
//            turnToPosition(-7, xyz, 0.8, 2, false);
//        }


        //robot.launcherMotor.setPower(LAUNCHER_SPEED);

//        sleep(2250);
//
//        for (int i = 0; i < 3; i++) {
//            encoderDrive(FORWARD_SPEED, 'l', 8, 2.5);
//            //shoot();
//            //robot.launcherMotor.setPower(LAUNCHER_SPEED += (.005 * i));
//        }
//        encoderDrive(FORWARD_SPEED,'f',12,5);
    }

    public void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.9f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
//    public void shoot() {
//        robot.launcherServo.setPosition(0.7);
//        sleep(1250);
//        robot.launcherServo.setPosition(0.2746);
//        sleep(1250);
//    }
    /*
    public void turnToPositionAutonDriving (double pos, String xyz, double topPower, double timeoutS, boolean gyroDrive)
    {
        //COUNTER CLOCKWISE IS POSITIVE; CLOCKWISE IS NEGATIVE om nom nom ;)

        stopAndReset();
        double originalAngle = readAngle(xyz);
        double target = pos;

        runtime.reset();

        double angle = readAngle(xyz); //variable for gyro correction around z axis
        double error = angle - target;
        double powerScaled = topPower;
        double degreesTurned;

        if(target < 0)
        {
            target += 3;
        }
        else if(target > 0)
        {
            target += .5;
        }
        do {
            //salient values
            angle = readAngle(xyz);
            error = angle - target;
            degreesTurned = angle - originalAngle;
            powerScaled = topPower * Math.abs(error/90) * pidMultiplierTurning(error);

            //prevents extreme slowing towards end of turn
//            if(-6 < error && error < 0)
//            {
//                powerScaled += gyroTurnModLeft;
//            }
//            else if (0 < error && error < 6)
//            {
//                powerScaled += gyroTurnModRight;
//            }

            //telementry
            telemetry.addData("original angle", originalAngle);
            telemetry.addData("current angle", readAngle(xyz));
            telemetry.addData("error", error);
            telemetry.addData("degrees turned", degreesTurned);
            telemetry.addData("target", target);
            telemetry.update();

            //direction handling
            if (error > 0)
            {
                normalDrive(powerScaled, -powerScaled);
            }
            else if (error < 0)
            {

                normalDrive(-powerScaled, powerScaled);
            }

            updateAngles();
        }
        while (opModeIsActive() && ((Math.abs(error) > gyroTurnThreshold) || (gyroDrive && ((Math.abs(error) >= 1.75) && Math.abs(error) <= 2.25))) && (runtime.seconds() < timeoutS));

        //stop turning and reset for next action
        normalDrive(0, 0);
        stopAndReset();
        updateAngles();
    }*/

   /* public void turnToPosition (double target, String xyz, double topPower, double timeoutS, boolean isCorrection) {
        //Write code to correct to a target position (NOT FINISHED)
        target*= -1;
        double originalAngle = readAngle(xyz);


        runtime.reset();

        double angle = readAngle(xyz); //variable for gyro correction around z axis
        double error = angle - target;
        double powerScaled = topPower;
        do {
            angle = readAngle(xyz);
            error = angle - target;
            if (!isCorrection) {
                powerScaled = topPower * (error / 180) * pidMultiplierTurning(error);
            }

            //double powerScaled = power*pidMultiplier(error);
            telemetry.addData("original angle", originalAngle);
            telemetry.addData("current angle", readAngle(xyz));
            telemetry.addData("error", error);
            telemetry.update();
            if (error > 0) {
                if (xyz.equals("z")) {
                    normalDrive(powerScaled, -powerScaled);
                }
                if (xyz.equals("y")) {
                    if (opModeIsActive()) {
                        robot.fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.bLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.bRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.fLMotor.setPower(powerScaled);
                        robot.fRMotor.setPower(powerScaled);
                        robot.bLMotor.setPower(powerScaled);
                        robot.bRMotor.setPower(powerScaled);
                    }
                }
            } else if (error < 0) {
                if (xyz.equals("z")) {
                    normalDrive(powerScaled, -powerScaled);
                }
                if (xyz.equals("y")) {
                    if (opModeIsActive()) {
                        robot.fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.bLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.bRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.fLMotor.setPower(-powerScaled);
                        robot.fRMotor.setPower(-powerScaled);
                        robot.bLMotor.setPower(-powerScaled);
                        robot.bRMotor.setPower(-powerScaled);
                    }
                }
            }
        } while (opModeIsActive() && ((error > .3) || (error < -0.3)) && (runtime.seconds() < timeoutS));
        normalDrive(0, 0);

    }
*/
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
        Acceleration gravity;
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
    public int getSpeedError(double speed) { //me being stupid
        return (int) (speed * 200);
    }
}