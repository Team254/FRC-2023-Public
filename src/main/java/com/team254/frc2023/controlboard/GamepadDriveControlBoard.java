package com.team254.frc2023.controlboard;

import com.team254.frc2023.Constants;

public class GamepadDriveControlBoard implements IDriveControlBoard {
    private static GamepadDriveControlBoard mInstance = null;

    public static GamepadDriveControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new GamepadDriveControlBoard();
        }

        return mInstance;
    }

    private final XboxController mController;

    private GamepadDriveControlBoard() {
        mController = new XboxController(Constants.kDriveGamepadPort);
    }

    @Override
    public double getThrottle() {
        return mController.getJoystick(XboxController.Side.LEFT, XboxController.Axis.Y);
    }

    @Override
    public double getStrafe() {
        return -mController.getJoystick(XboxController.Side.LEFT, XboxController.Axis.X);
    }

    @Override
    public double getRotation() {
        return -mController.getJoystick(XboxController.Side.RIGHT, XboxController.Axis.X);
    }

    @Override
    public boolean resetGyro() {
        return mController.getButton(XboxController.Button.BACK);
    }

    @Override
    public boolean getSwitchDriveMode() { return mController.getButton(XboxController.Button.LB); }

    @Override
    public boolean getScore() {
        return mController.getTrigger(XboxController.Side.RIGHT);
    }

    @Override
    public boolean getAutoAlign() {
        return mController.getTrigger(XboxController.Side.LEFT);
    }
}