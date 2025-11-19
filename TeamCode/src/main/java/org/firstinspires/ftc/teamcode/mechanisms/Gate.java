package org.firstinspires.ftc.teamcode.mechanisms;

import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class Gate {
    private RobotHardware robot;
    private static final double OPEN_POSITION = 0.5;
    private static final double CLOSED_POSITION = -1.0;

    public Gate(RobotHardware robot) {
        this.robot = robot;
    }

    public void openGate() {
        robot.gateServo.setPosition(OPEN_POSITION);
    }

    public void closeGate() {
        robot.gateServo.setPosition(CLOSED_POSITION);
    }

    public void setGatePosition(double position) {
        robot.gateServo.setPosition(position);
    }

    public double getGatePosition() {
        return robot.gateServo.getPosition();
    }

    public boolean isOpen() {
        return Math.abs(robot.gateServo.getPosition() - OPEN_POSITION) < 0.1;
    }
}