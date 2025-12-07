package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class MecanumDrive {
    private RobotHardware robot;

    public MecanumDrive(RobotHardware robot) {
        this.robot = robot;
    }

    public void mecanumDrive(double drive, double strafe, double turn) {
        double powerScale = 0.45;

        double leftFrontPower = (drive + strafe + turn) * powerScale;
        double rightFrontPower = (drive - strafe - turn) * powerScale;
        double leftRearPower = (drive - strafe + turn) * powerScale;
        double rightRearPower = (drive + strafe - turn) * powerScale;

        // Normalize powers
        double maxPower = Math.max(Math.abs(leftFrontPower),
                Math.max(Math.abs(rightFrontPower),
                        Math.max(Math.abs(leftRearPower), Math.abs(rightRearPower))));
        if (maxPower > 1) {
            leftFrontPower /= maxPower;
            rightFrontPower /= maxPower;
            leftRearPower /= maxPower;
            rightRearPower /= maxPower;
        }

        robot.fl_motor.setPower(leftFrontPower);
        robot.fr_motor.setPower(rightFrontPower);
        robot.bl_motor.setPower(leftRearPower);
        robot.br_motor.setPower(rightRearPower);
    }

    public void driveFieldRelative(double forward, double right, double rotate, IMU imu) {
        // Convert to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Rotate by robot's current heading
        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Use regular mecanum drive with adjusted values
        mecanumDrive(newForward, newRight, rotate);
    }

    public void stop() {
        robot.setDrivePower(0, 0, 0, 0);
    }
}