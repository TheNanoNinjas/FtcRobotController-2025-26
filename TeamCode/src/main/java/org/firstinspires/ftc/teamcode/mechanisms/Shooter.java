package org.firstinspires.ftc.teamcode.mechanisms;

import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class Shooter {
    private RobotHardware robot;

    //long rangshooting working well with 0.61 power.
    private static double LONG_RANGE_POWER = 0.58;
    //0.53
    private static double SHORT_RANGE_POWER = 0.51;
    //535
    private static double AUTO_SHORT_POWER = 0.53;
    private static double AUTO_LONG_POWER = 0.71;

    private static double MANUAL_INTAKE_POWER = -0.5;


    public Shooter(RobotHardware robot) {
        this.robot = robot;
    }

    public void startShootingFar() {
        robot.leftShooter.setPower(LONG_RANGE_POWER);
        robot.rightShooter.setPower(LONG_RANGE_POWER);
    }

    public void startShootingAutoClose(){
        robot.leftShooter.setPower(AUTO_SHORT_POWER);
        robot.rightShooter.setPower((AUTO_SHORT_POWER));
    }
    public void startShootingAutoFar(){
        robot.leftShooter.setPower(AUTO_LONG_POWER);
        robot.rightShooter.setPower(AUTO_LONG_POWER);

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