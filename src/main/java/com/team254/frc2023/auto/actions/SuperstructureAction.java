package com.team254.frc2023.auto.actions;

import com.team254.frc2023.subsystems.RollerClaw;
import com.team254.frc2023.subsystems.Superstructure;

import edu.wpi.first.wpilibj.Timer;

public class SuperstructureAction implements Action {
    protected final Superstructure mSuperstructure = Superstructure.getInstance();
    protected final RollerClaw mRollerClaw = RollerClaw.getInstance();
    protected Superstructure.GoalState mDesiredState;

    protected double mTimeout = -1.0;
    private double mStartTime = 0.0;

    public SuperstructureAction(Superstructure.GoalState goal) {
        mDesiredState = goal;
    }

    public SuperstructureAction(Superstructure.GoalState goal, double timeout) {
        mDesiredState = goal;
        this.mTimeout = timeout;
    }

    @Override
    public void start() {
        this.mStartTime = Timer.getFPGATimestamp();
        mSuperstructure.setGoalState(mDesiredState);
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        if (mTimeout >= 0 && Timer.getFPGATimestamp() - mStartTime >= mTimeout) return true;
        return (mDesiredState == Superstructure.GoalState.SCORE
                && mSuperstructure.getGameObjectType() == Superstructure.GameObjectType.CONE
                && !mRollerClaw.hasGamePiece())
                || mSuperstructure.isAtGoal();
    }

    @Override
    public void done() {}
}