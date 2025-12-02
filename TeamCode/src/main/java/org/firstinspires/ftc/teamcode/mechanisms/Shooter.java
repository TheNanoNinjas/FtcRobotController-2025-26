package org.firstinspires.ftc.teamcode.mechanisms;

import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class Shooter {
    private RobotHardware robot;

    //Autonomous long rangshooting working well with 0.58 power.
    private static double LONG_RANGE_POWER = 0.645;
    private static double SHORT_RANGE_POWER = 0.585;

    private static double MANUAL_INTAKE_POWER = -0.5;


    public Shooter(RobotHardware robot) {
        this.robot = robot;
    }



    public void startShootingFar() {
        robot.leftShooter.setPower(LONG_RANGE_POWER);
        robot.rightShooter.setPower(LONG_RANGE_POWER);
    }


    public void manualIntakeShooter() {
        robot.leftShooter.setPower(MANUAL_INTAKE_POWER);
        robot.rightShooter.setPower(MANUAL_INTAKE_POWER);
    }

    public void startShootingClose() {
        robot.leftShooter.setPower(SHORT_RANGE_POWER);
        robot.rightShooter.setPower(SHORT_RANGE_POWER);
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