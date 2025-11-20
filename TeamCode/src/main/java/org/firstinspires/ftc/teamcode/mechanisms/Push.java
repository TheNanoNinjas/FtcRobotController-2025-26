package org.firstinspires.ftc.teamcode.mechanisms;

import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class Push {

    private RobotHardware robot;

    public Push(RobotHardware robot) {
        this.robot = robot;
    }

    public void startWheel() {
        robot.wheelMotor.setPower(1.0);
    }

    public void stopPushing() {
        robot.wheelMotor.setPower(0.0);
    }

    public void reversePush() {
        robot.wheelMotor.setPower(-1.0);
    }

    public void setPushPower(double power) {
        robot.wheelMotor.setPower(power);
    }
}
