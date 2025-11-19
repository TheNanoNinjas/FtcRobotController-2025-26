package org.firstinspires.ftc.teamcode.mechanisms;

import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class Shooter {
    private RobotHardware robot;

    public Shooter(RobotHardware robot) {
        this.robot = robot;
    }

    public void startShooting() {
        robot.leftShooter.setPower(1.0);
        robot.rightShooter.setPower(1.0);
    }

    public void stopShooting() {
        robot.leftShooter.setPower(0.0);
        robot.rightShooter.setPower(0.0);
    }

    public void setShooterPower(double power) {
        robot.leftShooter.setPower(power);
        robot.rightShooter.setPower(power);
    }

    public boolean isRunning() {
        return robot.leftShooter.getPower() > 0 || robot.rightShooter.getPower() > 0;
    }
}