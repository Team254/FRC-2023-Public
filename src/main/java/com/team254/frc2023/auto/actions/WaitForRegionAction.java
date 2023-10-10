package com.team254.frc2023.auto.actions;

import com.team254.frc2023.RobotState;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

/**
 * Action to wait for a given amount of time To use this Action, call runAction(new WaitAction(your_time))
 */
public class WaitForRegionAction implements Action {
    private Translation2d mBottomLeft;
    private Translation2d mTopRight;

    public WaitForRegionAction(Translation2d bottomLeft, Translation2d topRight) {
        mBottomLeft = bottomLeft;
        mTopRight = topRight;
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        Pose2d robotState = RobotState.getInstance().getFieldToVehicle(Timer.getFPGATimestamp());
        return robotState.getTranslation().x() < mTopRight.x() && robotState.getTranslation().x() > mBottomLeft.x()
                && robotState.getTranslation().y() > mBottomLeft.y() && robotState.getTranslation().y() < mTopRight.y();
    }

    @Override
    public void done() {}
}
