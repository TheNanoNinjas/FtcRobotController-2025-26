package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.mechanisms.AprilTagLimelight;


import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class Shooter {

    private RobotHardware robot;
    private AprilTagLimelight tagLimelight;

    private final DcMotorEx leftShooter;
    private final DcMotorEx rightShooter;
//1570
    private static final double LONG_RANGE_VELOCITY = 1580  ;
    //1510
   private static final double SHORT_RANGE_VELOCITY = 1400;
    private static final double AUTO_LONG_VELOCITY = 1600;
    private static final double AUTO_SHORT_VELOCITY = 1300;



   // private double targetVelocity = 0;
    //private static final double VELOCITY_STEP = 75;
    //private static final double MAX_VELOCITY = 2500; // safety cap


private final double TagVelocityScaleShort = (SHORT_RANGE_VELOCITY / 75);

private final double TagVelocityScaleFar = (LONG_RANGE_VELOCITY / 121);


private double VelocityTag;
private double VelocityTagFar;

    private static final double MANUAL_INTAKE_POWER = -0.5;

    public Shooter(RobotHardware robot, AprilTagLimelight tagLimelight) {
        this.tagLimelight = tagLimelight;


        leftShooter = robot.leftShooter;
        rightShooter = robot.rightShooter;


        leftShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
  /*  public void increaseVelocity() {
        targetVelocity += VELOCITY_STEP;
        targetVelocity = Math.min(targetVelocity, MAX_VELOCITY);

        leftShooter.setVelocity(targetVelocity);
        rightShooter.setVelocity(targetVelocity);
    }
*/


    public void startShootingFar() {
        leftShooter.setVelocity(LONG_RANGE_VELOCITY);
        rightShooter.setVelocity(LONG_RANGE_VELOCITY);
    }

    public void startShootingClose() {
        leftShooter.setVelocity(SHORT_RANGE_VELOCITY);
        rightShooter.setVelocity(SHORT_RANGE_VELOCITY);
    }
    public void shootTagsClose() {
        double distanceIn = tagLimelight.getDistanceInches();
        if (distanceIn <= 0) return;

        VelocityTag = TagVelocityScaleShort * distanceIn;

        leftShooter.setVelocity(VelocityTag);
        rightShooter.setVelocity(VelocityTag);
    }

    public void shootTagsFar() {
        double distanceIn = tagLimelight.getDistanceInches();
        if (distanceIn <= 0) return;

        VelocityTagFar = TagVelocityScaleFar * distanceIn;

        leftShooter.setVelocity(VelocityTagFar);
        rightShooter.setVelocity(VelocityTagFar);
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
        return leftError < 200 && rightError < 200;
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
