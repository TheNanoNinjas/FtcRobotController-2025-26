package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class Shooter {


    private final DcMotorEx leftShooter;
    private final DcMotorEx rightShooter;
//1577
    private static final double LONG_RANGE_VELOCITY = 1570  ;
    //1510
    private static final double SHORT_RANGE_VELOCITY = 1350;
    private static final double AUTO_LONG_VELOCITY = 1600;
    private static final double AUTO_SHORT_VELOCITY = 1300;


    private static final double MANUAL_INTAKE_POWER = -0.5;

    public Shooter(RobotHardware robot) {


        leftShooter = (DcMotorEx) robot.leftShooter;
        rightShooter = (DcMotorEx) robot.rightShooter;


        leftShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }



    public void startShootingFar() {
        leftShooter.setVelocity(LONG_RANGE_VELOCITY);
        rightShooter.setVelocity(LONG_RANGE_VELOCITY);
    }

    public void startShootingClose() {
        leftShooter.setVelocity(SHORT_RANGE_VELOCITY);
        rightShooter.setVelocity(SHORT_RANGE_VELOCITY);
    }

    public void startShootingAutoFar() {
        leftShooter.setVelocity(AUTO_LONG_VELOCITY);
        rightShooter.setVelocity(AUTO_LONG_VELOCITY);
    }

    public void startShootingAutoClose() {
        leftShooter.setVelocity(AUTO_SHORT_VELOCITY);
        rightShooter.setVelocity(AUTO_SHORT_VELOCITY);
    }

    public void manualIntakeShooter() {
        leftShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftShooter.setPower(MANUAL_INTAKE_POWER);
        rightShooter.setPower(MANUAL_INTAKE_POWER);
    }

    public void stopShooting() {
        leftShooter.setVelocity(0);
        rightShooter.setVelocity(0);
    }


    public boolean isAtVelocity(double targetVelocity) {
        double leftError = Math.abs(leftShooter.getVelocity() - targetVelocity);
        double rightError = Math.abs(rightShooter.getVelocity() - targetVelocity);
        return leftError < 50 && rightError < 50;
    }

    public boolean isFarShotReady() {
        return isAtVelocity(LONG_RANGE_VELOCITY);
    }

    public boolean isCloseShotReady() {
        return isAtVelocity(SHORT_RANGE_VELOCITY);
    }

    public double getLeftVelocity() {
        return leftShooter.getVelocity();
    }

    public double getRightVelocity() {
        return rightShooter.getVelocity();
    }
}
