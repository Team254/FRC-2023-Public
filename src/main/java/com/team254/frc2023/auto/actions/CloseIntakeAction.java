package com.team254.frc2023.auto.actions;

public class CloseIntakeAction implements Action {
//    protected final PinchyClaw mClaw = PinchyClaw.getInstance();

    @Override
    public void start() {
        //mClaw.wantClose();
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
//        return mClaw.atGoal();
        return false;
    }

    @Override
    public void done() {}
}