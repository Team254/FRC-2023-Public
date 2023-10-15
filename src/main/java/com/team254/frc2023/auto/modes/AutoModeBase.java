package com.team254.frc2023.auto.modes;

import java.util.ArrayList;
import java.util.List;

import com.team254.frc2023.auto.AutoModeEndedException;
import com.team254.frc2023.auto.actions.Action;
import com.team254.frc2023.subsystems.Drive;
import com.team254.frc2023.subsystems.RobotStateEstimator;
import com.team254.lib.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * An abstract class that is the basis of the robot's autonomous routines. This is implemented in auto modes (which are
 * routines that do actions).
 */
public abstract class AutoModeBase {
    protected final double mUpdateRate = 1.0 / 50.0;
    protected boolean mActive = false;
    protected boolean mIsInterrupted = false;
    public Pose2d mStartPose = Pose2d.identity();
    List<Action> mActions = new ArrayList<>();

    protected void addAction(Action action) {
        mActions.add(action);
    }

    private void routine() throws AutoModeEndedException {
        for (var action : mActions) {
            runAction(action);
        }
    }

    public void setStartPose() {
        RobotStateEstimator.getInstance().resetOdometry(mStartPose);
        Drive.getInstance().setHeading(mStartPose.getRotation());
    }

    public void run() {
        mActive = true;

        try {
            routine();
        } catch (AutoModeEndedException e) {
            DriverStation.reportError("AUTO MODE DONE!!!! ENDED EARLY!!!!", false);
            return;
        }

        done();
    }

    public void done() {
        System.out.println("Auto mode done");
    }

    public void stop() {
        mActive = false;
    }

    public boolean isActive() {
        return mActive;
    }

    public boolean isActiveWithThrow() throws AutoModeEndedException {
        if (!isActive()) {
            throw new AutoModeEndedException();
        }

        return isActive();
    }

    public void interrupt() {
        System.out.println("** Auto mode interrrupted!");
        mIsInterrupted = true;
    }

    public void resume() {
        System.out.println("** Auto mode resumed!");
        mIsInterrupted = false;
    }

    public void runAction(Action action) throws AutoModeEndedException {
        isActiveWithThrow();
        long waitTime = (long) (mUpdateRate * 1000.0);

        // WaitForNumBannerSensorsAction for interrupt state to clear
        while (isActiveWithThrow() && mIsInterrupted) {
            try {
                Thread.sleep(waitTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        action.start();

        // Run action, stop action on interrupt, non active mode, or done
        while (isActiveWithThrow() && !action.isFinished() && !mIsInterrupted) {
            action.update();

            try {
                Thread.sleep(waitTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        action.done();

    }

    public boolean getIsInterrupted() {
        return mIsInterrupted;
    }
}
