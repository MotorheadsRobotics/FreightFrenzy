package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

//import org.firstinspires.ftc.teamcode.src.main.java.org.firstinspires.ftc.teamcode.DriveOnlyHardware;


@TeleOp(name="MotorPortTest", group="Teleop")

//@Disabled

public class MotorPortTest extends OpMode {

    Hardware robot = new Hardware();

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init()
    {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("flMotorPort", robot.fLMotor.getPortNumber());
        telemetry.addData("frMotorPort", robot.fRMotor.getPortNumber());
        telemetry.addData("blMotorPort", robot.bLMotor.getPortNumber());
        telemetry.addData("brMotorPort", robot.bRMotor.getPortNumber());
        telemetry.addData("pmLeft", robot.pulleyMotorL.getPortNumber());
        telemetry.addData("pmRight", robot.pulleyMotorR.getPortNumber());
        telemetry.addData("Carousel", robot.carouselMotor.getPortNumber());
        telemetry.addData("Intake", robot.intakeMotor.getPortNumber());
        telemetry.update();
    }
}