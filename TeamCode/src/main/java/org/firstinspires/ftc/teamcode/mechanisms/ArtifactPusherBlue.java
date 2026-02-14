package org.firstinspires.ftc.teamcode.mechanisms;

import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class ArtifactPusherBlue {

    private final RobotHardware robot;
    private final ShooterBlue shooterBlue;
    //.8
    private static final double PUSH_POWER = 0.85;

    public ArtifactPusherBlue(RobotHardware robot, ShooterBlue shooterBlue) {
        this.robot = robot;
        this.shooterBlue = shooterBlue;
    }

    public void startArtifactPushing() {
        robot.wheelMotor.setPower(0.76);
    }
    //.8
    public void reverseArtifacts() {
        robot.wheelMotor.setPower(-0.9);
    }

    public void startWheelIntake() {
        robot.wheelMotor.setPower(0.7);
    }

    public void driverPushFar(boolean driverWantsToShoot) {
        if (driverWantsToShoot && shooterBlue.isFarShotReady()) {
            robot.wheelMotor.setPower(PUSH_POWER);
        } else {
            robot.wheelMotor.setPower(0);
        }
    }



    public void driverPushClose(boolean driverWantsToShoot) {
        if (driverWantsToShoot && shooterBlue.isCloseShotReady()) {
            robot.wheelMotor.setPower(PUSH_POWER);
        } else {
            robot.wheelMotor.setPower(0);
        }
    }

    public void stopPushing() {
        robot.wheelMotor.setPower(0);
    }
}
