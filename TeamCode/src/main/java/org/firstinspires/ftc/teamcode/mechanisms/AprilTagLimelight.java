package org.firstinspires.ftc.teamcode.mechanisms;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class AprilTagLimelight {

    private RobotHardware robot;
    private Shooter shooter;

    private Limelight3A limelight;
    private IMU imu;

    // Heights in meters
    private final double cameraHeight = 14 / 39.37;
    private final double tagHeight = 30.0 / 39.37;

    public AprilTagLimelight(HardwareMap hardwareMap,
                             RobotHardware robot,
                             Shooter shooter) {

        this.robot = robot;
        this.shooter = shooter;

        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(8);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(parameters);
    }

    /** Call once after init */
    public void start() {
        limelight.start();
    }

    /** Call every loop */
    public void update() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(
                orientation.getYaw(AngleUnit.DEGREES)
        );
    }

    public boolean hasValidTarget() {
        LLResult result = limelight.getLatestResult();
        return result != null && result.isValid();
    }

    public double getDistanceMeters() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return -1;

        double ty = result.getTy();
        if (Math.abs(ty) < 0.5) return -1;

        double angleRad = Math.toRadians(ty);
        return (tagHeight - cameraHeight) / Math.tan(angleRad);
    }

    public double getDistanceInches() {
        double meters = getDistanceMeters();
        return meters < 0 ? -1 : meters * 39.37;
    }

    public double getTx() {
        LLResult result = limelight.getLatestResult();
        return result != null ? result.getTx() : 0;
    }
}
