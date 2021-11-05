package org.firstinspires.ftc.teamcode.Hardware;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import java.util.Iterator;

//public class package org.firstinspires.ftc.teamcode.src;

//import com.qualcomm.robotcore.hardware.DistanceSensor;


public class Hardware
{
    public DcMotor fLMotor;
    public DcMotor fRMotor;
    public DcMotor bLMotor;
    public DcMotor bRMotor;

    public DcMotor intakeMotor;
    public DcMotor carouselMotor;

    public DcMotor pulleyMotorL;
    public DcMotor pulleyMotorR;

    public Servo bucketServo;

    // motors whose directions need to be reversed
    private int flMotorPort = 1; // not true just haven't checked
    private int blMotorPort = 3; // this one too

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
        DCMotorSetup();

        // Set up Servos
        bucketServo = hwMap.get(Servo.class, "bucketServo");
    }
    public void DCMotorSetup() {
        // Define Motors
        Iterator<HardwareDevice> hardwareDevices = hwMap.iterator();
        while (hardwareDevices.hasNext()) {
            DcMotor currentMotor = (DcMotor) hardwareDevices.next();
            currentMotor.setPower(0);
            currentMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            currentMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            int portNumber = currentMotor.getPortNumber();
            if(portNumber == flMotorPort || portNumber == blMotorPort) //
            {
                currentMotor.setDirection(DcMotor.Direction.REVERSE);
            }
            else
            {
                currentMotor.setDirection(DcMotor.Direction.FORWARD);
            }
        }
//        for(int i = 0; i < motors.length; i++)
//        {
//            motors[i] = hwMap.get(DcMotor.class, motorNames[i]);
//            motors[i].setPower(0);
//            motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            if(i == 0 || i == 2) //
//            {
//                motors[i].setDirection(DcMotor.Direction.REVERSE);
//            }
//            else
//            {
//                motors[i].setDirection(DcMotor.Direction.FORWARD);
//            }
//        }
    }
}
