package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//public class package org.firstinspires.ftc.teamcode.src;

//import com.qualcomm.robotcore.hardware.DistanceSensor;


public class VirChassisHardware
{
    public DcMotor fLMotor;
    public DcMotor fRMotor;
    public DcMotor bLMotor;
    public DcMotor bRMotor;

    /*public DcMotor intakeMotor;
    public DcMotor carouselMotor;

    public DcMotor pulleyMotorL;
    public DcMotor pulleyMotorR;

    public Servo bucketServo;

    public DcMotor[] motors =           {fLMotor,   fRMotor,    bLMotor,    bRMotor,    intakeMotor,    carouselMotor,      pulleyMotorL,   pulleyMotorR};
    public String[] motorNames =        {"fLMotor", "fRMotor",  "bLMotor",  "bRMotor",  "intakeMotor",  "carouselMotor",    "pulleyMotorL", "pulleyMotorR"};
    public boolean[] motorDirections =  {false,     true,       false,      true,       true,           true,               true,           true};
*/

    //    public DcMotor launcherMotor;
//
//    public DcMotor liftMotor;
//
//    public Servo launcherServo;
//
//    public Servo claw;
//
//    public DcMotor intakeMotor;
//
//    public DcMotor conveyorMotor;
//
//    public ColorSensor leftColor;
//    public ColorSensor rightColor;
//    public TouchSensor magnetLimit;

    //declaring values for use with encoders
    public String xyz = "z";

    public static final double     COUNTS_PER_MOTOR_REV = 383.6;    // Currently: Andymark Neverest 40
    public static final double     COUNTS_PER_REV_ARM = 1440;
    public static final double     COUNTS_PER_INCH_ARM = COUNTS_PER_REV_ARM/4;
    public static final double     DRIVE_GEAR_REDUCTION = .666;     // This is < 1.0 if geared UP //On OUR CENTER MOTOR THE GEAR REDUCTION IS .5
    public static final double     WHEEL_DIAMETER_INCHES = 3.7795;     // For figuring circumference
    public static final double     COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);


    /* Local OpMode members. */
    HardwareMap hwMap;


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Set up Motors
        /*for(int i = 0; i < motors.length; i++)
        {
            motors[i] = hwMap.get(DcMotor.class, motorNames[i]);
            motors[i].setPower(0);
            motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if(motorDirections[i])
            {
                motors[i].setDirection(DcMotor.Direction.FORWARD);
            }
            else
            {
                motors[i].setDirection(DcMotor.Direction.REVERSE);
            }
        }
        */

        //DCMotorSetup(motors, motorNames, motorDirections);

        fLMotor = hwMap.get(DcMotor.class, "fLMotor");
        fRMotor = hwMap.get(DcMotor.class, "fRMotor");
        bRMotor = hwMap.get(DcMotor.class, "bRMotor");
        bLMotor = hwMap.get(DcMotor.class, "bLMotor");

//        fLMotor = hwMap.dcMotor.get("fLMotor");
//        fRMotor = hwMap.dcMotor.get("fRMotor");
//        bRMotor = hwMap.dcMotor.get("bRMotor");
//        bLMotor = hwMap.dcMotor.get("bLMotor");

/*        intakeMotor = hwMap.get(DcMotor.class, "intakeMotor");
        carouselMotor = hwMap.get(DcMotor.class, "carouselMotor");

        pulleyMotorL = hwMap.get(DcMotor.class, "pulleyMotorL");
        pulleyMotorR = hwMap.get(DcMotor.class, "pulleyMotorR");
*/
        //motors[0] = fLMotor;

        fLMotor.setPower(0);
        bLMotor.setPower(0);
        fRMotor.setPower(0);
        bRMotor.setPower(0);
/*
        intakeMotor.setPower(0);
        carouselMotor.setPower(0);

        pulleyMotorL.setPower(0);
        pulleyMotorR.setPower(0);
*/
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
//        fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        bLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        bRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
/*
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        carouselMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pulleyMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pulleyMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
*/
        fLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
/*
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pulleyMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pulleyMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
*/
        fLMotor.setDirection(DcMotor.Direction.REVERSE);
        fRMotor.setDirection(DcMotor.Direction.FORWARD);
        bLMotor.setDirection(DcMotor.Direction.REVERSE);
        bRMotor.setDirection(DcMotor.Direction.FORWARD);
/*
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        carouselMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        pulleyMotorL.setDirection(DcMotorSimple.Direction.FORWARD);
        pulleyMotorR.setDirection(DcMotorSimple.Direction.FORWARD);
*/

        // Set up Servos
  //      bucketServo = hwMap.get(Servo.class, "bucketServo");

    }
    public void DCMotorSetup(DcMotor[] motors, String[] motorNames, boolean[] motorDirections) {
        // Define Motors
        for(int i = 0; i < motors.length; i++)
        {
            motors[i] = hwMap.get(DcMotor.class, motorNames[i]);
            motors[i].setPower(0);
            motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if(motorDirections[i])
            {
                motors[i].setDirection(DcMotor.Direction.FORWARD);
            }
            else
            {
                motors[i].setDirection(DcMotor.Direction.REVERSE);
            }
        }
    }
}
