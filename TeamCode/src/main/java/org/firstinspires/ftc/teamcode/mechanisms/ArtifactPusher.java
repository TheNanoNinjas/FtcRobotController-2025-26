package org.firstinspires.ftc.teamcode.mechanisms;

import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class ArtifactPusher {

    private RobotHardware robot;

    public ArtifactPusher(RobotHardware robot) {
        this.robot = robot;
    }

    public void startWheel() {
        robot.wheelMotor.setPower(.75);
    }

    public void startWheelIntaker(){robot.wheelMotor.setPower(0.5);}

    public void stopPushing() {
        robot.wheelMotor.setPower(0.0);
    }

    public void reversePush() {
        robot.wheelMotor.setPower(-.5);
    }

    public void setPushPower(double power) {
        robot.wheelMotor.setPower(power);
    }
}
