package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class AprilTagLimelight {

    private final Limelight3A limelight;
    private final IMU imu;

    private final double cameraHeight = 17 / 39.37;
    private final double tagHeight = 30.0 / 39.37;

    public AprilTagLimelight(HardwareMap hardwareMap) {

        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(8);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));
    }

    public void start() {
        limelight.start();
    }

    public void update() {
        double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        limelight.updateRobotOrientation(yaw);
    }

    public boolean hasValidTarget() {
        LLResult result = limelight.getLatestResult();
        return result != null && result.isValid();
    }

    public double getDistanceInches() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return -1;

        double ty = result.getTy();
        if (Math.abs(ty) < 0.5) return -1;

        return ((tagHeight - cameraHeight) / Math.tan(Math.toRadians(ty))) * 39.37;
    }

    public double getTx() {
        LLResult result = limelight.getLatestResult();
        return result != null ? result.getTx() : 0;
    }
}
