package org.firstinspires.ftc.teamcode.mechanisms;

import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class ArtifactPusher {

    private final RobotHardware robot;
    private final Shooter shooter;
//.8
    private static final double PUSH_POWER = 0.7;

    public ArtifactPusher(RobotHardware robot, Shooter shooter) {
        this.robot = robot;
        this.shooter = shooter;
    }

    public void startArtifactPushing() {
        robot.wheelMotor.setPower(0.8);
    }
//.8
    public void reverseArtifacts() {
        robot.wheelMotor.setPower(-0.9);
    }

    public void startWheelIntake() {
        robot.wheelMotor.setPower(0.7);
    }

    public void driverPushFar(boolean driverWantsToShoot) {
        if (driverWantsToShoot && shooter.isFarShotReady()) {
            robot.wheelMotor.setPower(PUSH_POWER);
        } else {
            robot.wheelMotor.setPower(0);
        }
    }



    public void driverPushClose(boolean driverWantsToShoot) {
        if (driverWantsToShoot && shooter.isCloseShotReady()) {
            robot.wheelMotor.setPower(PUSH_POWER);
        } else {
            robot.wheelMotor.setPower(0);
        }
    }

    public void stopPushing() {
        robot.wheelMotor.setPower(0);
    }
}
