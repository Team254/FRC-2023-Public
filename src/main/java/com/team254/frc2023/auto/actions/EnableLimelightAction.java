package com.team254.frc2023.auto.actions;

import com.team254.frc2023.subsystems.Limelight;

public class EnableLimelightAction implements Action {

    private Limelight mLimelight = Limelight.getInstance();

    public EnableLimelightAction() {
    }
    @Override
    public void start() {
        mLimelight.setDisableProcessing(false);
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void done() {
    }
}
