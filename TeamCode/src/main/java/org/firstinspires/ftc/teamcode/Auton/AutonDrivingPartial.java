package org.firstinspires.ftc.teamcode.Auton;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.HardwarePartial;

import java.util.ArrayList;
import java.util.List;

//@Autonomous(name="AutonDrivingDriveOnly", group="AutonTesting")
public class AutonDrivingPartial extends LinearOpMode {

    /* Declare OpMode members. */
    public HardwarePartial robot = new HardwarePartial();
    public ElapsedTime runtime = new ElapsedTime();
    public String xyz = "z";
    //CONTAINS ALL METHODS AND VARIABlES TO BE EXTENDED BY OTHER AUTON CLASSES
    static final double     COUNTS_PER_MOTOR_REV = 384.5;    // GoBilda 5202 Series 13.7:1
    //static final double
    // COUNTS_PER_REV_ARM = 1495; //torquenado
    //static final double     PULLEY_DIAMETER = 1.3;
    // static final double     COUNTS_PER_INCH_ARM = COUNTS_PER_REV_ARM/(PULLEY_DIAMETER * Math.PI);
    static final double     DRIVE_GEAR_REDUCTION = 1.0;    // This is < 1.0 if geared UP //On OUR CENTER MOTOR THE GEAR REDUCTION IS .5
    static final double     WHEEL_DIAMETER_INCHES = 4.65;     // For figuring circumference
    static final double     COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable
    static final double     MIN_TURN_POWER = .000001;

    public BNO055IMU imu;

    public static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    public static final boolean PHONE_IS_PORTRAIT = false  ;

    public static final String VUFORIA_KEY =
            "AYy6NYn/////AAABmTW3q+TyLUMbg/IXWlIG3BkMMq0okH0hLmwj3CxhPhvUlEZHaOAmESqfePJ57KC2g6UdWLN7OYvc8ihGAZSUJ2JPWAsHQGv6GUAj4BlrMCjHvqhY0w3tV/Azw2wlPmls4FcUCRTzidzVEDy+dtxqQ7U5ZtiQhjBZetAcnLsCYb58dgwZEjTx2+36jiqcFYvS+FlNJBpbwmnPUyEEb32YBBZj4ra5jB0v4IW4wYYRKTNijAQKxco33VYSCbH0at99SqhXECURA55dtmmJxYpFlT/sMmj0iblOqoG/auapQmmyEEXt/T8hv9StyirabxhbVVSe7fPsAueiXOWVm0kCPO+KN/TyWYB9Hg/mSfnNu9i9";

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    public static final float mmPerInch        = 25.4f;
    // the height of the center of the target image above the floor

    // Constant for Stone Target
    public static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets

    // Class Members
    public OpenGLMatrix lastLocation = null;
    public VuforiaLocalizer vuforia = null; // Vuforia is for planar images

    public TFObjectDetector tfod; // TensorFlow for 3d detection

    public WebcamName webcamName = null;

    public boolean targetVisible = false;
    public float phoneXRotate    = 0;
    public float phoneYRotate    = 0;
    public float phoneZRotate    = 0;
    public List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
    public VuforiaTrackables targetsSkyStone;


    public Orientation angles = new Orientation();
    public Acceleration gravity;
    public double startAngle = 0;

    public enum MarkerPlacement {
        LEFT,
        MIDDLE,
        RIGHT
    }

    //gyro drive variables
    public double gyroDriveThreshold = 10;
    public double gyroDriveSpeedSlow = .27;
    public double gyroDriveSpeed = .32;
    public double gyroDriveSpeedFast = .35;
    private double gyroDriveInitBoost = .15;
    public double slow = .3;
    public double moderate = .5;
    public double fast = .7;
    public boolean right = true;
    public boolean left = false;

    //gyro turn variables
    private double gyroTurnThreshold = .7;
    private double degreeError = 2;
    public double slowTurnSpeed = .5;
    public double turnSpeed = .3;
    public double axisTurnSpeed = 1;
    private double gyroTurnBoost = .07;

    public double NORTH = 0;
    public double SOUTH = 180;
    public double EAST = 90;
    public double WEST = -90;



    public double strafeSpeed = .8;

    public double clawPos = .35;

    @Override
    public void runOpMode() {
    }

    public double avg(double... args) {
        double average = 0;
        for (double arg: args) {
            average += arg;
        }
        return average / args.length;
    }

    public void gyroDrive (double distance, double angle, boolean initBoost, double speed, double speedMult, double timeoutS)
    {
        runtime.reset();
        angle *= -1;
        stopAndReset();

        int fLTarget;
        int fRTarget;
        int bLTarget;
        int bRTarget;

        int fLInit = robot.fLMotor.getCurrentPosition();
        int fRInit = robot.fRMotor.getCurrentPosition();
        int bLInit = robot.bLMotor.getCurrentPosition();
        int bRInit = robot.bRMotor.getCurrentPosition();

        //int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            fLTarget = robot.fLMotor.getCurrentPosition() + moveCounts;
            fRTarget = robot.fRMotor.getCurrentPosition() + moveCounts;
            bLTarget = robot.bLMotor.getCurrentPosition() + moveCounts;
            bRTarget = robot.bLMotor.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.fLMotor.setTargetPosition(fLTarget);
            robot.fRMotor.setTargetPosition(fRTarget);
            robot.bLMotor.setTargetPosition(bLTarget);
            robot.bRMotor.setTargetPosition(bRTarget);


            robot.fLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.fRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            //gyroDriveSpeed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.fLMotor.setPower(speed);
            robot.fRMotor.setPower(speed);
            robot.bLMotor.setPower(speed);
            robot.bRMotor.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.fLMotor.isBusy() && robot.fRMotor.isBusy() && robot.bLMotor.isBusy() && robot.bRMotor.isBusy()) && runtime.seconds() < timeoutS) {

                // adjust relative speed based on heading error.
                error = getError(angle);

                if(Math.abs(error) < gyroDriveThreshold)
                {
                    error = 0;
                }

                //changes powers in order to correct for turning while driving
                steer = -getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed + steer;
                rightSpeed = speed - steer;

                //makes first 100 counts faster bc drive takes a lot of time to accelerate
                if(initBoost) {
                    if (Math.abs(robot.fLMotor.getCurrentPosition() - fLInit) < 100 && Math.abs(robot.bLMotor.getCurrentPosition() - bLInit) < 100)
                    {
                        leftSpeed += gyroDriveInitBoost;
                    }
                    if (Math.abs(robot.fRMotor.getCurrentPosition() - fRInit) < 100 && Math.abs(robot.bRMotor.getCurrentPosition() - bRInit) < 100)
                    {
                        rightSpeed += gyroDriveInitBoost;
                    }
                }
                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                leftSpeed *= speedMult;
                rightSpeed *= speedMult;

                robot.fLMotor.setPower(leftSpeed);
                robot.fRMotor.setPower(rightSpeed);
                robot.bLMotor.setPower(leftSpeed);
                robot.bRMotor.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d:%7d:%7d",      fLTarget,  fRTarget, bLTarget, bRTarget);
                telemetry.addData("Actual",  "%7d:%7d:%7d:%7d",      robot.fLMotor.getCurrentPosition(),
                        robot.fRMotor.getCurrentPosition(), robot.bLMotor.getCurrentPosition(), robot.bRMotor.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.addData("Angle", angle);
                telemetry.update();
            }

            // Stop all motion;
            normalDrive(0, 0, false);

            //correct for drift during drive
            //turnToPosition(-angle, "z", turnSpeed, 3);

            // Turn off RUN_TO_POSITION
            stopAndReset();
        }
    }

    public String vuforia(List<VuforiaTrackable> allTrackables, VuforiaTrackables targetsSkyStone)
    {
        //vuforia function and returns string w/ skystone position
        runtime.reset();
        String skystonePosition = "null";
        targetsSkyStone.activate();
        while (runtime.seconds() <= 5 && (skystonePosition.equals("null") || skystonePosition.equals("left"))) {

            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
                telemetry.addData("Position", translation.get(1)/ mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                //telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                double pos = translation.get(1);
                if(pos > 0)
                {
                    skystonePosition = "right";
                }
                else if(-pos > 9)
                {
                    skystonePosition = "center";
                }
                else
                {
                    skystonePosition = "left";
                }
            }
            else {
                telemetry.addData("Visible Target", "none");
                skystonePosition = "left";
            }
            if(!skystonePosition.equals("null") && !skystonePosition.equals("left") && runtime.seconds() >= 2)
            {
                return skystonePosition;
            }
            telemetry.addData("Skystone Position", skystonePosition);
            telemetry.update();
        }
        telemetry.addData("stopped", "stopped");
        telemetry.update();
        //sleep(2500);

        // Disable Tracking when we are done;
        targetsSkyStone.deactivate();
        return skystonePosition;
    }


    public void setDir()
    {
        //sets referential directions based on the starting position of the robot to use w/ turn to position
        NORTH = readAngle("z");
        SOUTH = NORTH + 178;
        EAST = NORTH + 90;
        WEST = NORTH - 90;
    }

    public static double counts(double inches)
    {
        double newInches = (inches - 3.7959) / 1.1239;
        return newInches;
    }


    public void updateAngles()
    {
        try {
            angles = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }
        catch (NullPointerException e)
        {
            telemetry.addData("Null Pointer Exception", "true");
        }
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

    public void normalDriveTurn(double lpower, double rpower, boolean encoder)
    {

        if (opModeIsActive()) {
            //stopAndReset();
            if(encoder)
            {
                //robot.fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //robot.fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                telemetry.addData("Reached normalDriveTurn", "True");
//                telemetry.update();
//                TryMotors();
//                if((robot.bLMotor != null) && (robot.bRMotor != null))
//                {
//
//                }

                robot.bLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.bRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //robot.fLMotor.setPower(lpower);
                //robot.fRMotor.setPower(rpower);
                robot.bLMotor.setPower(lpower);
                robot.bRMotor.setPower(rpower);
            }
            else
            {
//                TryMotors();
//                if((robot.bLMotor != null) && (robot.bRMotor != null))
//                {
//
//                }

                robot.bLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.bRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                telemetry.addData("gabeTest", "True");
//                telemetry.update();
                //robot.fLMotor.setPower(lpower);
                //robot.fRMotor.setPower(rpower);
                robot.bLMotor.setPower(lpower);
                robot.bRMotor.setPower(rpower);
            }
        }
    }

//    public void turnToPosition (double target, String xyz, double topPower, double timeoutS) {
//        //stopAndReset();
//        target*= -1;
//        double originalAngle = readAngle("z");
//
//        //wild
//        /*if (target > 0)
//        {
//            target -= degreeError;
//        }
//        else
//        {
//            target += degreeError;
//        }*/
//        //telemetry.addData("hello", "1");
//        //telemetry.update();
//
//        runtime.reset();
//
//        double angle; //variable for gyro correction around z axis
//        double error;
//        double powerScaled;
//        do {
//            //telemetry.addData("hello", "2");
//            //sleep(1000);
//            //telemetry.update();
//            angle = readAngle("z");
//            error = angle - target;
//            powerScaled = topPower * Math.abs(error / 180) * pidMultiplierTurning(error);
//            /*if(error < 6)
//            {
//                powerScaled += gyroTurnBoost;
//            }*/
//
//            //double powerScaled = power*pidMultiplier(error);
//
//            if (error > 0)
//            {
//                normalDrive(powerScaled, -powerScaled);
//            }
//            else if (error < 0)
//            {
//                normalDrive(-powerScaled, powerScaled);
//            }
//            telemetry.addData("original angle", originalAngle);
//            telemetry.addData("current angle", angle);
//            telemetry.addData("error", error);
//            telemetry.addData("target", target);
//            telemetry.update();
//        } while (opModeIsActive() && (Math.abs(error) > gyroTurnThreshold) && (runtime.seconds() < timeoutS));
//        normalDrive(0, 0);
//        //stopAndReset();
//        updateAngles();
//    }

    public void turnDegrees (double degrees, String xyz, double topPower, double timeoutS) {
        //stopAndReset();

        degrees *= -1;
//        if(degrees < 0)
//        {
//            degrees += degreeError;
//        }
//        else
//        {
//            degrees -= degreeError;
//        }
        double originalAngle = readAngle(xyz);

        double target = originalAngle + degrees;

        //wild


        //telemetry.addData("hello", "1");
        //telemetry.update();

        runtime.reset();

        double angle = readAngle(xyz); //variable for gyro correction around z axis
        double error = angle - target;
        double powerScaled = topPower;
        double degreesTurned;
        do {
            //telemetry.addData("hello", "2");
            //sleep(1000);
            //telemetry.update();
            angle = readAngle(xyz);
            error = angle - target;
            degreesTurned = angle - originalAngle;

            powerScaled = topPower * Math.abs(error/180) * pidMultiplierTurning(error);
            if(error < 6)
            {
                powerScaled += gyroTurnBoost + .01;
            }
            else if (degreesTurned < 6)
            {
                powerScaled += gyroTurnBoost + .01;
            }

            //double powerScaled = power*pidMultiplier(error);
            telemetry.addData("original angle", originalAngle);
            telemetry.addData("current angle", readAngle(xyz));
            telemetry.addData("error", error);
            telemetry.addData("target", target);
            telemetry.addData("degrees", degrees);
            telemetry.addData("degrees turned", degreesTurned);
            telemetry.update();
            if (error > 0)
            {
                normalDrive(-powerScaled, powerScaled, false);
            }
            else if (error < 0)
            {

                normalDrive(powerScaled, -powerScaled, false);
            }
        } while (opModeIsActive() && (Math.abs(error) > 10) && (Math.abs(degrees - degreesTurned) > 10) && (runtime.seconds() < timeoutS));
        normalDrive(0, 0, false);
        //stopAndReset();

        sleep(300);

        updateAngles();

    }

    public void turnToPosition (double degrees, String xyz, double topPower, double timeoutS) {
        //stopAndReset();\
        //degrees *= -1;

        double originalAngle = readAngle(xyz);

        double target = degrees;
        double targetABS = Math.abs(target);

        //wild
// the line above sounds like Ria

        //telemetry.addData("hello", "1");
        //telemetry.update();

        runtime.reset();

        double angle = readAngle(xyz); //variable for gyro correction around z axis
        double error = target - angle;
        final double originalError = error;
        double errorABS = Math.abs(error);
        double powerScaled = topPower;
        double degreesTurned;
        double degreesTurnedABS;

        double maxPower = topPower;
        double minPower = 0.15;
        double beginSlowing = error/2; // degrees before target at which point the robot slows down
        double slowRate = 0.085;

        telemetry.addData("original angle", originalAngle);
        telemetry.addData("current angle", readAngle(xyz));
        telemetry.addData("error", error);
        telemetry.addData("target", target);
        telemetry.update();
        sleep(250);
        do {
            //telemetry.addData("hello", "2");
            //sleep(1000);
            //telemetry.update();
            angle = readAngle(xyz);
            error = target - angle;
            errorABS = Math.abs(error);

            degreesTurned = angle - originalAngle;
            degreesTurnedABS = Math.abs(degreesTurned);

//            double functionalPower = minPower + (maxPower - minPower) * Math.exp(-slowRate * (-((targetABS - degreesTurnedABS) - beginSlowing)));
//            double adjustedPower = Math.min(maxPower, functionalPower); xander code obliterate :)
            double adjuster = Math.abs(error/originalError);
            adjuster += minPower;
            adjuster = Math.min(1, adjuster);
            double adjustedPower = adjuster * maxPower;

            //double powerScaled = power*pidMultiplier(error);
            telemetry.addData("original angle", originalAngle);
            telemetry.addData("current angle", readAngle(xyz));
            telemetry.addData("error", error);
            telemetry.addData("target", target);
//            telemetry.addData("Functional Power", functionalPower);
//            telemetry.addData("Motor Power", adjustedPower);
            //telemetry.addData("degrees", degrees);
            telemetry.update();

            //!!!!!!!!!!!!!!!!!!!!! EXPERIMENTAL
            //TODO: SEEMS LIKE A STRANGE THING WHERE THE ANGLE TO TURN TO IS ADDED BY WHERE IT SHOULD STOP SLOWING
            //TODO: ALSO DOESNT SLOW
//            adjustedPower = functionalPower;

            if (error < 0)
            {
                normalDrive((adjustedPower), -(adjustedPower), false);
            }
            else if (error > 0)
            {
                normalDrive(-(adjustedPower), (adjustedPower), false);
            }
        } while (opModeIsActive() && (Math.abs(error) > 0.5) && (runtime.seconds() < timeoutS));

        normalDrive(0, 0, false);
        //stopAndReset();
        updateAngles();
        sleep(500);
    }

//    public void turnToPosition (double degrees, String xyz, double topPower, double timeoutS, boolean fast, boolean clock) {
//        //stopAndReset();
//
//        //forced direction turn to position in case the direction handling doesn't work
//        degrees *= -1;
//        if(degrees < 0)
//        {
//            degrees += degreeError;
//        }
//        else
//        {
//            degrees -= degreeError;
//        }
//        double originalAngle = readAngle(xyz);
//
//        double target = degrees;
//
//        //wild
//
//
//        //telemetry.addData("hello", "1");
//        //telemetry.update();
//
//        runtime.reset();
//
//        double angle = readAngle(xyz); //variable for gyro correction around z axis
//        double error = target - angle;
//        double powerScaled = topPower;
//        double degreesTurned;
//        do {
//            //telemetry.addData("hello", "2");
//            //sleep(1000);
//            //telemetry.update();
//            angle = readAngle(xyz);
//            error = angle - target;
//            degreesTurned = angle - originalAngle;
//
//            //adds boosts in certain places to make the turn faster
//            if(!fast)
//            {
//                powerScaled = topPower * Math.abs(error/180) * pidMultiplierTurning(error);
//            }
//            else
//            {
//                powerScaled = topPower * Math.abs(error/90) * pidMultiplierTurning(error);
//            }
//            if(error < 6)
//            {
//                powerScaled += gyroTurnBoost;
//            }
//            else if (Math.abs(degreesTurned) < 6)
//            {
//                powerScaled += gyroTurnBoost;
//            }
//
//            //double powerScaled = power*pidMultiplier(error);
//            telemetry.addData("original angle", originalAngle);
//            telemetry.addData("current angle", readAngle(xyz));
//            telemetry.addData("error", error);
//            telemetry.addData("target", target);
//            //telemetry.addData("degrees", degrees);
//            telemetry.update();
//                //normalDrive(powerScaled, -powerScaled);
//
//            if(clock)
//                normalDrive(powerScaled, -powerScaled);
//            else normalDrive(-powerScaled, powerScaled);
//        } while (opModeIsActive() && (Math.abs(error) > gyroTurnThreshold) && (runtime.seconds() < timeoutS));
//        normalDrive(0, 0);
//        //stopAndReset();
//        updateAngles();
//
//    }

    public void axisTurn (double target, String xyz, double topPower, double timeoutS) {
        //stopAndReset();
        //turn centered around the side of the robot
        target*= -1;
        double originalAngle = readAngle(xyz);

        //wild
        if (target > 0)
        {
            target -= degreeError;
        }
        else
        {
            target += degreeError;
        }
        //telemetry.addData("hello", "1");
        //telemetry.update();

        runtime.reset();

        double angle = readAngle(xyz); //variable for gyro correction around z axis
        double error = angle - target;
        double powerScaled = topPower;
        do {
            //telemetry.addData("hello", "2");
            //sleep(1000);
            //telemetry.update();
            angle = readAngle(xyz);
            error = angle - target;
            powerScaled = topPower * (error / 180) * pidMultiplierTurning(error);
            if (error < 15)
            {
                powerScaled = .3;
            }
            //powerScaled *= -1;

            //double powerScaled = power*pidMultiplier(error);
            telemetry.addData("original angle", originalAngle);
            telemetry.addData("current angle", readAngle(xyz));
            telemetry.addData("error", error);
            telemetry.update();
            if (error > 0)
            {
                normalDrive(0, powerScaled, false);
            }
            else if (error < 0)
            {
                normalDrive(powerScaled, 0, false);
            }
        } while (opModeIsActive() && (Math.abs(error) > gyroTurnThreshold) && (runtime.seconds() < timeoutS));
        normalDrive(0, 0, false);
        //stopAndReset();

    }

    public void stopAndReset()
    {
        robot.bRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public double pidMultiplierDriving(double error) {
        //equation for power multiplier is x/sqrt(x^2 + C)
        int C = 500;
        return Math.abs(error / Math.sqrt((error * error) + C));
    }
    public double pidMultiplierTurning(double error) {
        //equation for power multiplier is x/sqrt(x^2 + C)
        double C = .001;
        return Math.abs(error / Math.sqrt((error * error) + C));
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



    /*public void gyroStrafe (double distance, double angle, boolean initBoost, double speed, double speedMult)
    {
        stopAndReset();

        int fLTarget;
        int fRTarget;
        int bLTarget;
        int bRTarget;

        int fLInit = robot.fLMotor.getCurrentPosition();
        int fRInit = robot.fRMotor.getCurrentPosition();
        int bLInit = robot.bLMotor.getCurrentPosition();
        int bRInit = robot.bRMotor.getCurrentPosition();

        //int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            fLTarget = (robot.fLMotor.getCurrentPosition() + moveCounts);
            fRTarget = (robot.fRMotor.getCurrentPosition()) + moveCounts;
            bLTarget = (robot.bLMotor.getCurrentPosition() + moveCounts);
            bRTarget = (robot.bLMotor.getCurrentPosition() + moveCounts);

            // Set Target and Turn On RUN_TO_POSITION
            robot.fLMotor.setTargetPosition(fLTarget);
            robot.fRMotor.setTargetPosition(-fRTarget);
            robot.bLMotor.setTargetPosition(-bLTarget);
            robot.bRMotor.setTargetPosition(bRTarget);


            robot.fLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.fRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            //gyroDriveSpeed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.fLMotor.setPower(speed);
            robot.fRMotor.setPower(speed);
            robot.bLMotor.setPower(speed);
            robot.bRMotor.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.fLMotor.isBusy() && robot.fRMotor.isBusy() && robot.bLMotor.isBusy() && robot.bRMotor.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);

                if(Math.abs(error) < gyroDriveThreshold)
                {
                    error = 0;
                }
                steer = -getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed + steer;
                rightSpeed = speed - steer;

                //makes first 100 counts faster bc drive takes a lot of time to accelerate
                if(initBoost) {
                    if (Math.abs(robot.fLMotor.getCurrentPosition() - fLInit) < 100 && Math.abs(robot.bLMotor.getCurrentPosition() - bLInit) < 100)
                    {
                        leftSpeed += gyroDriveInitBoost;
                    }
                    if (Math.abs(robot.fRMotor.getCurrentPosition() - fRInit) < 100 && Math.abs(robot.bRMotor.getCurrentPosition() - bRInit) < 100)
                    {
                        rightSpeed += gyroDriveInitBoost;
                    }
                }
                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                leftSpeed *= speedMult;
                rightSpeed *= speedMult;

                robot.fLMotor.setPower(rightSpeed);
                robot.fRMotor.setPower(-rightSpeed);
                robot.bLMotor.setPower(-leftSpeed);
                robot.bRMotor.setPower(leftSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d:%7d:%7d",      fLTarget,  fRTarget, bLTarget, bRTarget);
                telemetry.addData("Actual",  "%7d:%7d:%7d:%7d",      robot.fLMotor.getCurrentPosition(),
                        robot.fRMotor.getCurrentPosition(), robot.bLMotor.getCurrentPosition(), robot.bRMotor.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.addData("Angle", angle);
                telemetry.update();
            }

            // Stop all motion;
            normalDrive(0, 0);

            //correct for drift during drive
            turnToPosition(-angle, "z", turnSpeed, 2);

            // Turn off RUN_TO_POSITION
            stopAndReset();
        }
    }*/

    public void strafe (int iterations, double speed, boolean isRight, double balanceReduction, double milliseconds, double moreBalance, double Angle)
    {
        //balance reduction: add to this if the back of the robot is faster than the front, subtract for opposite
        //more balance: add to this if the robot is driving slightly backwards, subtract for opposite
        stopAndReset();
        runtime.reset();
        //int     newRightTarget;
        double fLSpeed = 0;
        double fRSpeed = 0;
        double bLSpeed = 0;
        double bRSpeed = 0;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            for(int i = 0; i < iterations; i++) {
                ElapsedTime clock = new ElapsedTime();
//                moveCounts = (int) (distance * COUNTS_PER_INCH)/10;
//                fLTarget = (robot.fLMotor.getCurrentPosition() + moveCounts);
//                fRTarget = (robot.fRMotor.getCurrentPosition() + moveCounts);
//                bLTarget = (robot.bLMotor.getCurrentPosition() + moveCounts);
//                bRTarget = (robot.bLMotor.getCurrentPosition() + moveCounts);
//
//                // Set Target and Turn On RUN_TO_POSITION
//                robot.fLMotor.setTargetPosition(fLTarget);
//                robot.fRMotor.setTargetPosition(-fRTarget);
//                robot.bLMotor.setTargetPosition(-bLTarget);
//                robot.bRMotor.setTargetPosition(bRTarget);


                robot.fLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.fRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.bLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.bRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


                // start motion.
                //gyroDriveSpeed = Range.clip(Math.abs(speed), 0.0, 1.0);
                while(clock.milliseconds() < milliseconds)
                {
                    fLSpeed = speed;
                    fRSpeed = speed;
                    bLSpeed = speed;
                    bRSpeed = speed;

                    if(isRight)
                    {
                        bRSpeed -= balanceReduction;
                        fLSpeed += balanceReduction;
                        fRSpeed += balanceReduction - moreBalance;
                        bLSpeed -= balanceReduction - moreBalance;

                        bLSpeed *= -1;
                        fRSpeed *= -1;

                    }
                    else
                    {
                        bRSpeed -= balanceReduction - moreBalance;
                        fLSpeed += balanceReduction - moreBalance;
                        fRSpeed += balanceReduction;
                        bLSpeed -= balanceReduction;

                        fLSpeed *= -1;
                        bRSpeed *= -1;

                    }
                    double max = Math.max(Math.max(fLSpeed, fRSpeed), Math.max(bLSpeed, bRSpeed));
                    if(max > 1)
                    {
                        fLSpeed/=max;
                        fRSpeed/=max;
                        bLSpeed/=max;
                        bRSpeed/=max;
                    }
                    robot.fLMotor.setPower(fLSpeed);
                    robot.fRMotor.setPower(fRSpeed);
                    robot.bLMotor.setPower(bLSpeed);
                    robot.bRMotor.setPower(bRSpeed);

                    telemetry.addData("fL Speed", fLSpeed);
                    telemetry.addData("fR Speed", fRSpeed);
                    telemetry.addData("bL Speed", bLSpeed);
                    telemetry.addData("bR Speed", bRSpeed);
                    telemetry.addData("milliseconds", milliseconds);
                    telemetry.addData("clock", clock.milliseconds());
                    telemetry.addData("iteration", i);
                    telemetry.update();

                }

                //slow down bc jerk causes drift and turning
                double inc = .85;
                for(int j = 0; j < 15; j++)
                {
                    robot.fLMotor.setPower(robot.fLMotor.getPower()*inc);
                    robot.fRMotor.setPower(robot.fRMotor.getPower()*inc);
                    robot.bLMotor.setPower(robot.bLMotor.getPower()*inc);
                    robot.bRMotor.setPower(robot.bRMotor.getPower()*inc);
                }
                normalDrive(0, 0, false); // stops it after 1 second
                turnToPosition(Angle, "z", turnSpeed-.05, 500);
                //turnToPosition(-angle, "z", turnSpeed, 4); //corrects at the end of each motion set
                sleep(300);
                //telemetry.addData("Target", "%7d:%7d:%7d:%7d", fLTarget, fRTarget, bLTarget, bRTarget);
            }

            // Stop all motion;
            normalDrive(0, 0, false);

            //correct for drift during drive
            //turnToPosition(-angle, "z", turnSpeed, 3);

            // Turn off RUN_TO_POSITION
            stopAndReset();
        }
    }



    public void pathComplete(int millisec)
    {
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(millisec);
    }


    /*
    public void gyroDriveWithC (double inches, double angle, String heading, double timeoutS)
    {
        //THIS IS BROKEN AS FUCK
        //WHAT A RIP

        stopAndReset();
        runtime.reset();

        //inches *= .5;
        int TargetFL = 0;
        int TargetFR = 0;
        int TargetBL = 0;
        int TargetBR = 0;
        double errorFL = 0;
        double errorFR = 0;
        double errorBL = 0;
        double errorBR = 0;
        double powerFL = 0;
        double powerFR = 0;
        double powerBL = 0;
        double powerBR = 0;
        //int newLeftTarget;
        //int newRightTarget;
        //int counts;
        double  max;
        double  angleError;
        double  steer;
        //double  leftSpeed;
        //double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive())
        {
            if(heading == "f")
            {
                TargetFL = robot.fLMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);
                TargetFR = robot.fRMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);
                TargetBL = robot.bLMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);
                TargetBR = robot.bRMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);

            }

            else if(heading == "b")
            {
                TargetFL = robot.fLMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH);
                TargetFR = robot.fRMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH);
                TargetBL = robot.bLMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH);
                TargetBR = robot.bRMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH);


            }

            else
            {
                telemetry.addData("not a valid direction", heading );
            }

            // Set Target and Turn On RUN_TO_POSITION
            robot.fLMotor.setTargetPosition(TargetFL);
            robot.fRMotor.setTargetPosition(TargetFR);
            robot.bLMotor.setTargetPosition(TargetBL);
            robot.bRMotor.setTargetPosition(TargetBR);

            robot.fLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.fRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.fLMotor.isBusy() && robot.fRMotor.isBusy() && robot.bLMotor.isBusy() && robot.bRMotor.isBusy()) && runtime.seconds() <= timeoutS) {

                // adjust relative speed based on heading error.

                //prevent over-correcting by having a threshold
                angleError = getError(angle);
                if(Math.abs(angleError) > 7)
                {
                    steer = getSteer(angleError, P_DRIVE_COEFF);
                }
                else
                {
                    steer = 0;
                }
                //steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (heading.equals("b"))
                {
                    steer *= -1.0;
                }
                else if(!heading.equals("f"))
                {
                    steer = 0;
                }

               // powerFL = (topPower - steer) * speed;
               // rightSpeed = (speed + steer) * speed;
                errorFL = TargetFL - robot.fLMotor.getCurrentPosition();
                errorFR = TargetFR - robot.fRMotor.getCurrentPosition();
                errorBL = TargetBL - robot.bLMotor.getCurrentPosition();
                errorBR = TargetBR - robot.bRMotor.getCurrentPosition();

                //steer *= 1.2;

                powerFL = gyroDriveSpeed - steer;
                powerFR = gyroDriveSpeed + steer;
                powerBL = gyroDriveSpeed - steer;
                powerBR = gyroDriveSpeed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.max(Math.abs(powerFL), Math.abs(powerFR)), Math.max(powerBL, powerBR));
                if (max > 1)
                {
                    powerFL /= max;
                    powerFR /= max;
                    powerBL /= max;
                    powerBR /= max;
                }

                powerFL *= pidMultiplierDriving(errorFL);
                powerFR *= pidMultiplierDriving(errorFR);
                powerBL *= pidMultiplierDriving(errorBL);
                powerBR *= pidMultiplierDriving(errorBR);

                robot.fLMotor.setPower(powerFL * gyroDriveSpeed);
                robot.bLMotor.setPower(powerFR * gyroDriveSpeed);
                robot.fRMotor.setPower(powerBL * gyroDriveSpeed);
                robot.bRMotor.setPower(powerBR * gyroDriveSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  angleError, steer);
                telemetry.addData("Target",  "%7d:%7d:%7d:%7d", TargetFL,  TargetFR, TargetBL, TargetBR);
                telemetry.addData("Current Pos",  "%7d:%7d:%7d:%7d", robot.fLMotor.getCurrentPosition(),  robot.fLMotor.getCurrentPosition(), robot.fLMotor.getCurrentPosition(), robot.fLMotor.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f:%5.2f:%5.2f",  powerFL, powerFR, powerBL, powerBR);
                telemetry.update();
            }

            // Stop all motion;
            normalDrive(0 ,0);

            // Turn off RUN_TO_POSITION
            stopAndReset();
        }
        stopAndReset();
    }*/

    public double getError(double targetAngle)
    {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - readAngle("z");
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
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
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
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
    public int getErrorEncoder(double speed) { //me being stupid
        return (int) (speed * 200);
    }
}