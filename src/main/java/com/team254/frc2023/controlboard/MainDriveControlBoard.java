package com.team254.frc2023.controlboard;

import com.team254.frc2023.Constants;
import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj.Joystick;

public class MainDriveControlBoard implements IDriveControlBoard {
    private static MainDriveControlBoard mInstance = null;

    public static MainDriveControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new MainDriveControlBoard();
        }

        return mInstance;
    }

    private final Joystick mTranslateStick;
    private final Joystick mRotateStick;

    private MainDriveControlBoard() {
        mTranslateStick = new Joystick(Constants.kMainThrottleJoystickPort);
        mRotateStick = new Joystick(Constants.kMainTurnJoystickPort);
    }

    @Override
    public double getThrottle() {
        return Util.handleDeadband(-mTranslateStick.getRawAxis(1), Constants.kDriveJoystickThreshold);
    }

    @Override
    public double getStrafe() {
        return Util.handleDeadband(-mTranslateStick.getRawAxis(0), Constants.kDriveJoystickThreshold);
    }

    @Override
    public double getRotation() {
        return Util.handleDeadband(mRotateStick.getRawAxis(0), Constants.kJoystickThreshold);
    }

    @Override
    public boolean resetGyro() {
        return mRotateStick.getRawButton(2);
    }

    @Override
    public boolean getSwitchDriveMode() {
        return mTranslateStick.getRawButton(2);
    }

    @Override
    public boolean getScore() {
        return mTranslateStick.getRawButton(1);
    }

    @Override
    public boolean getAutoAlign() {
        return mRotateStick.getRawButton(1);
    }
}