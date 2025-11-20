package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class RobotHardware {
    // Drive Motors
    public DcMotor fl_motor;
    public DcMotor fr_motor;
    public DcMotor bl_motor;
    public DcMotor br_motor;

    // Mechanism Motors
    public DcMotor leftShooter;
    public DcMotor rightShooter;
    public DcMotor pushMotor;

    // Servos
    public Servo gateServo;

    // Sensors
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
        pushMotor = hardwareMap.get(DcMotor.class, "pushMotor");

        // Set shooter direction
        rightShooter.setDirection(DcMotor.Direction.REVERSE);

        // Servos
        gateServo = hardwareMap.get(Servo.class, "gateServo");

        // IMU
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
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
        pushMotor.setPower(0);
    }

    public void logHardwareStatus(Telemetry telemetry) {
        telemetry.addData("FL Motor", fl_motor != null ? "OK" : "FAIL");
        telemetry.addData("FR Motor", fr_motor != null ? "OK" : "FAIL");
        telemetry.addData("BL Motor", bl_motor != null ? "OK" : "FAIL");
        telemetry.addData("BR Motor", br_motor != null ? "OK" : "FAIL");
        telemetry.addData("Left Shooter", leftShooter != null ? "OK" : "FAIL");
        telemetry.addData("Right Shooter", rightShooter != null ? "OK" : "FAIL");
        telemetry.addData("Push Motor", pushMotor != null ? "OK" : "FAIL");
        telemetry.addData("Gate Servo", gateServo != null ? "OK" : "FAIL");
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
        if (pushMotor != null) telemetry.addData("Push Motor", "Port " + pushMotor.getPortNumber());
        if (gateServo != null) telemetry.addData("Gate Servo", "Port " + gateServo.getPortNumber());
        telemetry.addLine("===================");
    }
}