package com.team254.frc2023.controlboard;

import com.team254.frc2023.Constants;

import edu.wpi.first.wpilibj.DriverStation;

public class ControlBoard implements IDriveControlBoard, IButtonControlBoard {
    private static ControlBoard mInstance = null;

    public static ControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new ControlBoard();
        }

        return mInstance;
    }

    private final IDriveControlBoard mDriveControlBoard;
    private final IButtonControlBoard mButtonControlBoard;

    private ControlBoard() {
        boolean useDriveGamepad = Constants.kForceDriveGamepad ||
                DriverStation.getJoystickIsXbox(Constants.kDriveGamepadPort);
        mDriveControlBoard = useDriveGamepad ? GamepadDriveControlBoard.getInstance()
                : MainDriveControlBoard.getInstance();
        mButtonControlBoard = GamepadButtonControlBoard.getInstance();
    }

    @Override
    public double getThrottle() {
        return mDriveControlBoard.getThrottle();
    }

    @Override
    public double getStrafe() {
        return mDriveControlBoard.getStrafe();
    }

    @Override
    public double getRotation() {
        return mDriveControlBoard.getRotation();
    }

    @Override
    public boolean resetGyro() {
        return mDriveControlBoard.resetGyro();
    }

    @Override
    public boolean getTestButton() {
        return mButtonControlBoard.getTestButton();
    }

    @Override
    public boolean getSwitchDriveMode() {
        return mDriveControlBoard.getSwitchDriveMode();
    }

    @Override
    public boolean getScoreHigh() {
        return mButtonControlBoard.getScoreHigh();
    }

    @Override
    public boolean getScoreMedium() {
        return mButtonControlBoard.getScoreMedium();
    }

    @Override
    public boolean getScoreLow() {
        return mButtonControlBoard.getScoreLow();
    }

    @Override
    public boolean primeLowFling() {
        return mButtonControlBoard.primeLowFling();
    }

    @Override
    public boolean primeHighFling() {
        return mButtonControlBoard.primeHighFling();
    }

    @Override
    public boolean getScoreMid() {
        return mButtonControlBoard.getScoreMid();
    }

    @Override
    public boolean getPickupFeederCone() {
        return mButtonControlBoard.getPickupFeederCone();
    }

    @Override
    public boolean getPickupGroundCone() {
        return mButtonControlBoard.getPickupGroundCone();
    }

    @Override
    public boolean getPickupFeederCube() {
        return mButtonControlBoard.getPickupFeederCube();
    }

    @Override
    public boolean getPickupGroundCube() {
        return mButtonControlBoard.getPickupGroundCube();
    }

    @Override
    public boolean getScore() {
        return mDriveControlBoard.getScore();
    }

    @Override
    public boolean getStow() {
        return mButtonControlBoard.getStow();
    }

    @Override
    public boolean getOpenClaw() {
        return mButtonControlBoard.getOpenClaw();
    }

    @Override
    public boolean getCloseClaw() {
        return mButtonControlBoard.getCloseClaw();
    }

    @Override
    public boolean getAutoAlign() {
        return mDriveControlBoard.getAutoAlign();
    }

    @Override
    public boolean getToggleClimbMode() {
        return mButtonControlBoard.getToggleClimbMode();
    }

    @Override
    public double getManualForksJog() {
        return mButtonControlBoard.getManualForksJog();
    }

    @Override
    public boolean getForksUp() {
        return mButtonControlBoard.getForksUp();
    }

    @Override
    public boolean getForksDown() {
        return mButtonControlBoard.getForksDown();
    }

    @Override
    public boolean getFling() {
        return mButtonControlBoard.getFling();
    }

    @Override
    public boolean getGroundIntakePush() { return mButtonControlBoard.getGroundIntakePush(); }
}