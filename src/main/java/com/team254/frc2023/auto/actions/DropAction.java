package com.team254.frc2023.auto.actions;

import com.team254.frc2023.subsystems.RollerClaw;
import com.team254.frc2023.subsystems.Superstructure;
import edu.wpi.first.wpilibj.Timer;

public class DropAction implements Action {
    protected final Superstructure mSuperstructure = Superstructure.getInstance();
    protected final RollerClaw mRollerClaw = RollerClaw.getInstance();
    double start_ts;

    @Override
    public void start() {
        start_ts = Timer.getFPGATimestamp();
        mSuperstructure.setGoalState(Superstructure.GoalState.DROP);
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - start_ts > 0.65;
    }

    @Override
    public void done() {
        mSuperstructure.setGoalState(Superstructure.GoalState.STOW);
    }
}
