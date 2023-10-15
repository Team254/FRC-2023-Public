package com.team254.frc2023.auto.actions;

import com.team254.frc2023.RobotState;
import com.team254.frc2023.subsystems.Drive;
import com.team254.frc2023.subsystems.RobotStateEstimator;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithMotion;
import com.team254.lib.trajectory.TimedView;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;
import edu.wpi.first.wpilibj.Timer;

public class DriveTrajectoryAction implements Action {
    private Drive mDrive = null;

    private final TrajectoryIterator<TimedState<Pose2dWithMotion>> mTrajectory;
    private final boolean mResetPose;
    private final boolean mResetGyro;

    public DriveTrajectoryAction(Trajectory<TimedState<Pose2dWithMotion>> trajectory) {
        this(trajectory, false);
    }

    public DriveTrajectoryAction(Trajectory<TimedState<Pose2dWithMotion>> trajectory, boolean resetPose) {
        mTrajectory = new TrajectoryIterator<>(new TimedView<>(trajectory));
        mResetPose = resetPose;
        mDrive = Drive.getInstance();
        mResetGyro = true;
    }

    public DriveTrajectoryAction(Trajectory<TimedState<Pose2dWithMotion>> trajectory, boolean resetPose, boolean resetGyro) {
        mTrajectory = new TrajectoryIterator<>(new TimedView<>(trajectory));
        mResetPose = resetPose;
        mDrive = Drive.getInstance();
        mResetGyro = resetGyro;
    }

    @Override
    public void start() {
        System.out.println("Starting trajectory! (length=" + mTrajectory.getRemainingProgress() + ")");
        if (mResetPose) {
            if (mResetGyro) {
                RobotStateEstimator.getInstance().resetOdometry(new Pose2d(mTrajectory.getState().state().getTranslation(), mTrajectory.getState().state().getRotation()));   // inches to meters
                mDrive.setHeading(mTrajectory.getState().state().getRotation());
            } else {
                RobotStateEstimator.getInstance().resetOdometry(new Pose2d(mTrajectory.getState().state().getTranslation(), mDrive.getFieldRelativeGyroscopeRotation()));   // inches to meters
            }
        }
        mDrive.setTrajectory(mTrajectory);
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        if (mDrive.isDoneWithTrajectory()) {
            double t = Timer.getFPGATimestamp();
            System.out.println("Trajectory finished");
            System.out.println("Pose: " + RobotState.getInstance().getFieldToVehicle(t));
            System.out.println("Setpoint: " + mTrajectory.getState().state().toString());
            System.out.println("Error: " + mTrajectory.getState().state().transformBy(RobotState.getInstance().getFieldToVehicle(t).inverse()));
            return true;
        }
        return false;
    }

    @Override
    public void done() {}
}

