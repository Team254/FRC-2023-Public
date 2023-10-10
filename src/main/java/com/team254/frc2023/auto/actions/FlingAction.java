package com.team254.frc2023.auto.actions;

import com.team254.frc2023.RobotState;
import com.team254.frc2023.planners.AutoAlignPointSelector;
import com.team254.frc2023.subsystems.Drive;
import com.team254.frc2023.subsystems.RollerClaw;
import com.team254.frc2023.subsystems.Superstructure;
import com.team254.lib.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;

import java.util.Optional;

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
