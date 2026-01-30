package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class HogbackAI {

    private final RobotHardware robot;
    private final ShooterAI shooter;

    private static final double PUSH_POWER = 1.0;

    private final ElapsedTime feedTimer = new ElapsedTime();
    private boolean feedInProgress = false;

    private static final double START_POWER = 0.20;
    private static final double SLEW_PER_MS = 0.005;

    private static final long FEED_PULSE_MS = 220;

    private boolean lastShootRequest = false;

    private double commandedPower = 0.0;
    private double lastSetPower = 0.0;
    private long lastPowerUpdateMs = 0;

    public HogbackAI(RobotHardware robot, ShooterAI shooter) {
        this.robot = robot;
        this.shooter = shooter;
        feedTimer.reset();
        lastPowerUpdateMs = (long) feedTimer.milliseconds();
    }

    public void startArtifactPushing() {
        robot.wheelMotor.setPower(1);
    }

    public void reverseArtifacts() {
        robot.wheelMotor.setPower(-0.8);
    }

    public void startWheelIntake() {
        robot.wheelMotor.setPower(0.75);
    }

    // ✅ FIX: unified shooter readiness
    public void driverPushFar(boolean driverWantsToShoot) {
        if (driverWantsToShoot && shooter.isReadyToFire()) {
            robot.wheelMotor.setPower(PUSH_POWER);
        } else {
            robot.wheelMotor.setPower(0);
        }
    }


    public void driverPushClose(boolean driverWantsToShoot) {
        if (driverWantsToShoot && shooter.isReadyToFire()) {
            robot.wheelMotor.setPower(PUSH_POWER);
        } else {
            robot.wheelMotor.setPower(0);
        }
    }

    public void stopPushing() {
        robot.wheelMotor.setPower(0);
        feedInProgress = false;
        commandedPower = 0.0;
        lastSetPower = 0.0;
    }

    private void applySlew(double targetPower) {
        long now = (long) feedTimer.milliseconds();
        long dt = Math.max(1, now - lastPowerUpdateMs);
        double maxStep = SLEW_PER_MS * dt;

        double delta = targetPower - lastSetPower;
        double step = Math.copySign(Math.min(Math.abs(delta), maxStep), delta);

        lastSetPower += step;
        robot.wheelMotor.setPower(lastSetPower);
        lastPowerUpdateMs = now;
    }

    public void feedOneWhenReady(boolean shootRequest) {
        boolean rising = shootRequest && !lastShootRequest;
        lastShootRequest = shootRequest;

        if (feedInProgress) {
            if (feedTimer.milliseconds() >= FEED_PULSE_MS) {
                feedInProgress = false;
                commandedPower = 0.0;
                applySlew(commandedPower);
                feedTimer.reset();
                return;
            }

            commandedPower = 1.0;
            applySlew(commandedPower);
            return;
        }

        commandedPower = 0.0;
        applySlew(commandedPower);

        if (rising && !shooter.shouldGateFeed()) {
            shooter.markShotFired();

            commandedPower = START_POWER;
            applySlew(commandedPower);

            feedInProgress = true;
            feedTimer.reset();
        }
    }
}
