package com.team254.frc2023.auto.actions;


import com.team254.frc2023.RobotState;
import edu.wpi.first.wpilibj.Timer;

public class WaitUntilCrossXBoundaryCommand implements Action {
    private double mXBoundary = 0;

    public WaitUntilCrossXBoundaryCommand(double x) {
        mXBoundary = x;
    }

    @Override
    public boolean isFinished() {
        return RobotState.getInstance().getOdomToVehicle(Timer.getFPGATimestamp()).getTranslation().x() > mXBoundary;
    }

    @Override
    public void update() {

    }

    @Override
    public void done() {

    }

    @Override
    public void start() {

    }
}
