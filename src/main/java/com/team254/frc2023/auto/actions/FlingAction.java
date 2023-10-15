package com.team254.frc2023.auto.actions;

import com.team254.frc2023.subsystems.RollerClaw;
import com.team254.frc2023.subsystems.Superstructure;

public class FlingAction implements Action {
    protected final Superstructure mSuperstructure = Superstructure.getInstance();
    protected final RollerClaw mRollerClaw = RollerClaw.getInstance();
    protected final boolean wantHighFling;


    public FlingAction(boolean wantHighFling) {
        this.wantHighFling = wantHighFling;
    }

    @Override
    public void start() {
        if(wantHighFling) {
            mSuperstructure.setGoalState(Superstructure.GoalState.FAR_FlING);
        } else {
            mSuperstructure.setGoalState(Superstructure.GoalState.CLOSE_FLING);
        }
    }

    @Override
    public void update() { }

    @Override
    public boolean isFinished() {
        return !mRollerClaw.hasGamePiece() && mSuperstructure.isAtGoal();
    }

    @Override
    public void done() {}
}
