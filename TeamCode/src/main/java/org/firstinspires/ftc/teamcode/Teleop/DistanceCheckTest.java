package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Sensor: REV2mDistance", group = "Sensor")
public class DistanceCheckTest extends LinearOpMode {

    private DistanceSensor DistanceSensorLeft, DistanceSensorRight;

    @Override
    public void runOpMode() {

        DistanceSensorLeft = hardwareMap.get(DistanceSensor.class, "DistanceSensorLeft");
        DistanceSensorRight = hardwareMap.get(DistanceSensor.class, "DistanceSensorRight");

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("range", String.format("%.01f in", DistanceSensorLeft.getDistance(DistanceUnit.INCH)));
            telemetry.addData("range", String.format("%.01f in", DistanceSensorRight.getDistance(DistanceUnit.INCH)));

            if(DistanceSensorLeft.getDistance(DistanceUnit.INCH) < 12 && DistanceSensorRight.getDistance(DistanceUnit.INCH) >= 12){
                telemetry.addData("Duck Position", "Left");
            }
            else if(DistanceSensorLeft.getDistance(DistanceUnit.INCH) >= 12 && DistanceSensorRight.getDistance(DistanceUnit.INCH) < 12){
                telemetry.addData("Duck Position", "Middle");
            }
            else if(DistanceSensorLeft.getDistance(DistanceUnit.INCH) < 12 && DistanceSensorRight.getDistance(DistanceUnit.INCH) < 12){
                telemetry.addData("Duck Position", "Does not Compute");
            }
            else{ // Neither sensor sees a duck
                telemetry.addData("Duck Position", "Right");
            }

            telemetry.update();
        }
    }
}
