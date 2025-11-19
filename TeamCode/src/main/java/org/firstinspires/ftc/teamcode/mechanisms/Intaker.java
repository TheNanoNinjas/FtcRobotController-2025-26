package org.firstinspires.ftc.teamcode.mechanisms;

import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class Intaker {
    private RobotHardware robot;

    public Intaker(RobotHardware robot) {
        this.robot = robot;
    }

    public void startPushing() {
        robot.pushMotor.setPower(1.0);
    }

    public void stopPushing() {
        robot.pushMotor.setPower(0.0);
    }

    public void reversePush() {
        robot.pushMotor.setPower(-1.0);
    }

    public void setPushPower(double power) {
        robot.pushMotor.setPower(power);
    }
}