package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class ShooterAI {

    private DcMotorEx leftShooter;
    private DcMotorEx rightShooter;

    private static final double LONG_RANGE_VELOCITY = 1900;
    private static final double SHORT_RANGE_VELOCITY = 1400;
    private static final double AUTO_LONG_VELOCITY = 1750;
    private static final double AUTO_SHORT_VELOCITY = 1500;

    private static final double MANUAL_INTAKE_POWER = -0.5;

    private static final double READY_TOLERANCE_PCT = 0.02;
    private static final long READY_HOLD_MS = 100;
    private static final long RECOVERY_HOLD_MS = 80;
    private static final double EMA_ALPHA = 0.25;

    private static final double KF = 0.0009;
    private static final double KP = 0.0008;
    private static final double KI = 0.0000;
    private static final double KD = 0.0001;

    private final ElapsedTime timer = new ElapsedTime();
    private long readySinceMs = -1L;
    private long recoveryArmedAtMs = -1L;
    private double smoothedLeft = 0.0;
    private double smoothedRight = 0.0;
    private double lastTarget = 0.0;

    public ShooterAI(RobotHardware robot) {

        leftShooter = (DcMotorEx) robot.leftShooter;
        rightShooter = (DcMotorEx) robot.rightShooter;

        // REQUIRED: explicit directions (adjust if needed)
        leftShooter.setDirection(DcMotor.Direction.REVERSE);
        rightShooter.setDirection(DcMotor.Direction.FORWARD);

        leftShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftShooter.setVelocityPIDFCoefficients(KP, KI, KD, KF);
        rightShooter.setVelocityPIDFCoefficients(KP, KI, KD, KF);

        timer.reset();
    }

    private void spinUpTo(double targetTps) {

        // CRITICAL FIX: clear leftover power from manual mode
        leftShooter.setPower(0);
        rightShooter.setPower(0);

        if (leftShooter.getMode() != DcMotor.RunMode.RUN_USING_ENCODER)
            leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (rightShooter.getMode() != DcMotor.RunMode.RUN_USING_ENCODER)
            rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lastTarget = targetTps;

        leftShooter.setVelocity(targetTps);
        rightShooter.setVelocity(targetTps);
    }

    public void startShootingFar() {
        spinUpTo(LONG_RANGE_VELOCITY);
    }

    public void startShootingClose() {
        spinUpTo(SHORT_RANGE_VELOCITY);
    }

    public void startShootingAutoFar() {
        spinUpTo(AUTO_LONG_VELOCITY);
    }

    public void startShootingAutoClose() {
        spinUpTo(AUTO_SHORT_VELOCITY);
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

        lastTarget = 0.0;
        readySinceMs = -1L;
        recoveryArmedAtMs = -1L;
    }

    public double getLeftVelocity() {
        return leftShooter.getVelocity();
    }

    public double getRightVelocity() {
        return rightShooter.getVelocity();
    }

    public void update() {
        double l = leftShooter.getVelocity();
        double r = rightShooter.getVelocity();

        smoothedLeft = EMA_ALPHA * l + (1.0 - EMA_ALPHA) * smoothedLeft;
        smoothedRight = EMA_ALPHA * r + (1.0 - EMA_ALPHA) * smoothedRight;

        if (lastTarget > 0 && withinTolerance(lastTarget)) {
            if (readySinceMs < 0) readySinceMs = (long) timer.milliseconds();
        } else {
            readySinceMs = -1L;
        }
    }

    public void markShotFired() {
        recoveryArmedAtMs = (long) timer.milliseconds();
    }

    public boolean shouldGateFeed() {
        if (lastTarget <= 0) return true;

        boolean readyNow = isReadyToFire();

        if (recoveryArmedAtMs >= 0) {
            long t = (long) timer.milliseconds() - recoveryArmedAtMs;
            if (t < RECOVERY_HOLD_MS) return true;
            if (readyNow) recoveryArmedAtMs = -1L;
        }
        return !readyNow;
    }

    public boolean isReadyToFire() {
        if (lastTarget <= 0) return false;
        if (!withinTolerance(lastTarget)) return false;
        if (readySinceMs < 0) return false;

        return (long) timer.milliseconds() - readySinceMs >= READY_HOLD_MS;
    }

    private boolean withinTolerance(double targetTps) {
        double tol = Math.abs(targetTps) * READY_TOLERANCE_PCT;
        return Math.abs(smoothedLeft - targetTps) <= tol
                && Math.abs(smoothedRight - targetTps) <= tol;
    }
}
