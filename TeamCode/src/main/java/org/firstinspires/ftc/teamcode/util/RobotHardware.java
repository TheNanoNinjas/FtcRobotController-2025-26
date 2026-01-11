package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.autonomous.GoBildaPinpointDriver;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import java.util.ArrayList;
import java.util.List;

public class RobotHardware {
    // Drive Motors
    public DcMotor fl_motor;
    public DcMotor fr_motor;
    public DcMotor bl_motor;
    public DcMotor br_motor;

    // Mechanism Motors
    public DcMotor leftShooter;
    public DcMotor rightShooter;
    public DcMotor intakeMotor;
    public DcMotor wheelMotor;

    //Sensors
    private GoBildaPinpointDriver odo;
    public Rev2mDistanceSensor distance_sensor;
    public IMU imu;

    public void init(HardwareMap hardwareMap) {
        // Drive Motors
        fl_motor = hardwareMap.get(DcMotor.class, "fl_motor");
        fr_motor = hardwareMap.get(DcMotor.class, "fr_motor");
        bl_motor = hardwareMap.get(DcMotor.class, "bl_motor");
        br_motor = hardwareMap.get(DcMotor.class, "br_motor");

        // Set drive motor directions (left motors reversed)
        fl_motor.setDirection(DcMotor.Direction.REVERSE);
        bl_motor.setDirection(DcMotor.Direction.REVERSE);


        // Mechanism Motors
        leftShooter = hardwareMap.get(DcMotor.class, "leftShooter");
        rightShooter = hardwareMap.get(DcMotor.class, "rightShooter");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        wheelMotor = hardwareMap.get(DcMotor.class, "wheelMotor");
        // Set shooter direction
        leftShooter.setDirection(DcMotor.Direction.REVERSE);

        wheelMotor.setDirection(DcMotor.Direction.REVERSE);

        // IMU
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));


        //Sensors
        distance_sensor = hardwareMap.get(Rev2mDistanceSensor.class, "distance_sensor");
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-88, 0.0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
        odo.setPosition(new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0));
    }

    public void updateOdo(){
        odo.update();
    }

    public double getOdoPositionY(DistanceUnit unit){
        odo.update();
        Pose2D position = odo.getPosition();

        return position.getY(unit);
    }

    public double getOdoPositionX(DistanceUnit unit){
        odo.update();
        Pose2D position = odo.getPosition();

        return position.getX(unit);
    }

    public double getOdoHeading(AngleUnit angleUnit){
        odo.update();
        return odo.getPosition().getHeading(angleUnit);
    }

    public Pose2D getOdoPosition(){
        odo.update();
       return odo.getPosition();
    }

    public void setDrivePower(double fl, double fr, double bl, double br) {
        fl_motor.setPower(fl);
        fr_motor.setPower(fr);
        bl_motor.setPower(bl);
        br_motor.setPower(br);
    }

    public void stopAllMotors() {
        setDrivePower(0, 0, 0, 0);
        leftShooter.setPower(0);
        rightShooter.setPower(0);
        intakeMotor.setPower(0);
        wheelMotor.setPower(0);
    }

    public void logHardwareStatus(Telemetry telemetry) {
        telemetry.addData("FL Motor", fl_motor != null ? "OK" : "FAIL");
        telemetry.addData("FR Motor", fr_motor != null ? "OK" : "FAIL");
        telemetry.addData("BL Motor", bl_motor != null ? "OK" : "FAIL");
        telemetry.addData("BR Motor", br_motor != null ? "OK" : "FAIL");
        telemetry.addData("Left Shooter", leftShooter != null ? "OK" : "FAIL");
        telemetry.addData("Right Shooter", rightShooter != null ? "OK" : "FAIL");
        telemetry.addData("Intake Motor", intakeMotor != null ? "OK" : "FAIL");
        telemetry.addData("Wheel Motor", wheelMotor != null ? "OK" : "FAIL");
        telemetry.addData("Odometry Wheels", odo != null ? "OK" : "FAIL");
        telemetry.addData("Distance Sensor", distance_sensor != null ? "OK" : "FAIL");
        telemetry.addData("Hardware", "Initialized");
    }
    
    public void displayPortMapping(Telemetry telemetry) {
        telemetry.addLine("=== PORT MAPPING ===");
        if (fl_motor != null) telemetry.addData("FL Motor", "Port " + fl_motor.getPortNumber());
        if (fr_motor != null) telemetry.addData("FR Motor", "Port " + fr_motor.getPortNumber());
        if (bl_motor != null) telemetry.addData("BL Motor", "Port " + bl_motor.getPortNumber());
        if (br_motor != null) telemetry.addData("BR Motor", "Port " + br_motor.getPortNumber());
        if (leftShooter != null) telemetry.addData("Left Shooter", "Port " + leftShooter.getPortNumber());
        if (rightShooter != null) telemetry.addData("Right Shooter", "Port " + rightShooter.getPortNumber());
        if (intakeMotor != null) telemetry.addData("Intake Motor", "Port " + intakeMotor.getPortNumber());
        if (wheelMotor != null) telemetry.addData("Wheel Motor", "Port " + wheelMotor.getPortNumber());
        telemetry.addLine("===================");
    }

    public void resetOdo() {


    }
}