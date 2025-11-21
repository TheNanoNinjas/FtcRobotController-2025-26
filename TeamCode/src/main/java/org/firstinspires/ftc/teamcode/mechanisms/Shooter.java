package org.firstinspires.ftc.teamcode.mechanisms;

import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class Shooter {
    private RobotHardware robot;

    public Shooter(RobotHardware robot) {
        this.robot = robot;
    }



    public void startShootingFar() {
        robot.leftShooter.setPower(0.8);
        robot.rightShooter.setPower(0.8);
    }


    public void manualIntakeShooter() {
        robot.leftShooter.setPower(-0.5);
        robot.rightShooter.setPower(-0.5);
    }

    public void startShootingClose() {
        robot.leftShooter.setPower(0.7);
        robot.rightShooter.setPower(0.7);
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