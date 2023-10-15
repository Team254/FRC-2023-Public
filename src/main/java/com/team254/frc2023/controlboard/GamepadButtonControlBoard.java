package com.team254.frc2023.controlboard;

import com.team254.frc2023.Constants;
import com.team254.frc2023.controlboard.XboxController.Button;
import com.team254.lib.util.Util;

public class GamepadButtonControlBoard implements IButtonControlBoard {
    private static GamepadButtonControlBoard mInstance = null;
    public static GamepadButtonControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new GamepadButtonControlBoard();
        }

        return mInstance;
    }

    private final XboxController mController;

    private GamepadButtonControlBoard() {
        mController = new XboxController(Constants.kOperatorControllerPort);
    }

    @Override
    public boolean getScoreHigh() {
        return mController.getButton(XboxController.Button.Y);
    }

    @Override
    public boolean getScoreMedium() {
        return mController.getButton(XboxController.Button.B);
    }

    @Override
    public boolean getScoreLow() {
        return mController.getButton(XboxController.Button.A);
    }

    @Override
    public boolean primeLowFling() {
        return Util.inRange(mController.getDPad() - 90, 45);
    }

    @Override
    public boolean primeHighFling() {
        return Util.inRange(mController.getDPad() - 270, 45);
    }

    @Override
    public boolean getScoreMid() {
        return mController.getDPad() == 0;
    }

    @Override
    public boolean getPickupFeederCone() {
        return mController.getButton(XboxController.Button.RB);
    }

    @Override
    public boolean getPickupGroundCone() {
        return mController.getTrigger(XboxController.Side.RIGHT);
    }

    @Override
    public boolean getPickupFeederCube() {
        return mController.getButton(XboxController.Button.LB);
    }

    @Override
    public boolean getPickupGroundCube() {
        return mController.getTrigger(XboxController.Side.LEFT);
    }

    @Override
    public boolean getTestButton() { return mController.getButton(Button.START); }

    @Override
    public boolean getStow() {
        return mController.getButton(XboxController.Button.X);
    }

    @Override
    public boolean getOpenClaw() {
        return mController.getButton(XboxController.Button.BACK) && !mController.getButton(Button.START);
    }

    @Override
    public boolean getCloseClaw() {
        return mController.getButton(XboxController.Button.START) && !mController.getButton(Button.BACK);
    }

    @Override
    public boolean getToggleClimbMode() {
        return mController.getButton(XboxController.Button.START) && mController.getButton(Button.BACK);
    }

    @Override
    public double getManualForksJog() {
        return Util.handleDeadband(mController.getJoystick(XboxController.Side.RIGHT, XboxController.Axis.Y), 0.1);
    }

    @Override
    public boolean getForksUp() {
        return Util.epsilonEquals(mController.getDPad(), 0);
    }


    @Override
    public boolean getFling() {
        return Util.epsilonEquals(mController.getDPad(), 0);
    }

    @Override
    public boolean getForksDown() {
        return Util.epsilonEquals(mController.getDPad(), 180);
    }

    @Override
    public boolean getGroundIntakePush() { return mController.getButton(Button.L_JOYSTICK); }
}
