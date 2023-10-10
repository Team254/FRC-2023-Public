package com.team254.frc2023.auto.actions;

import com.team254.frc2023.subsystems.Limelight;

public class DisableLimelightAction implements Action {

    private Limelight mLimelight = Limelight.getInstance();

    public DisableLimelightAction() {
    }

    @Override
    public void start() {
        mLimelight.setDisableProcessing(true);
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
