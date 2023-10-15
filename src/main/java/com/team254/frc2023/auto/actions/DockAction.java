package com.team254.frc2023.auto.actions;

import com.team254.frc2023.subsystems.Drive;
import com.team254.lib.control.SwerveHeadingController;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.swerve.ChassisSpeeds;
import com.team254.lib.util.SynchronousPIDF;
import com.team254.lib.util.TimeDelayedBoolean;
import edu.wpi.first.wpilibj.Timer;

import java.util.List;

public class DockAction implements Action {
    private static final Drive mDrive = Drive.getInstance();
    private static final SwerveHeadingController mSwerveHeadingController = SwerveHeadingController.getInstance();
    private static final SynchronousPIDF mRollController = new SynchronousPIDF();
    private TimeDelayedBoolean mBalanced = new TimeDelayedBoolean();
    private TimeDelayedBoolean pitchedUpTime = new TimeDelayedBoolean();
    private static final double gyroInclinationToSwitchToClosedLoop = 8.0; // degrees
    private final double mOpenLoopVel = 1.0; // m/s
    private double mStartTime = 0.0;
    private final double mTimeout;

    private Rotation2d mSteeringDirection; // field relative
    private boolean mBalanceMode = false;

    public DockAction(Rotation2d steeringDirection, double timeout) {
        mSteeringDirection = steeringDirection;
        mTimeout = timeout;
    }

    @Override
    public void start() {
        mStartTime = Timer.getFPGATimestamp();
        mSwerveHeadingController.setHeadingControllerState(SwerveHeadingController.HeadingControllerState.MAINTAIN);
        mSwerveHeadingController.setGoal(mDrive.getFieldRelativeGyroscopeRotation().getDegrees());
        mRollController.setPIDF(0.03, 0, 0, 0);
        mRollController.setSetpoint(0);
        mDrive.setVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                mOpenLoopVel * mSteeringDirection.cos(),
                0,
                mSwerveHeadingController.update(mDrive.getFieldRelativeGyroscopeRotation().getDegrees()),
                mDrive.getFieldRelativeGyroscopeRotation()));
    }

    @Override
    public void update() {
        double t = Timer.getFPGATimestamp();
        if(!mBalanceMode) {
            if(t - mStartTime > mTimeout) {
                System.out.println("Unable to dock");
            }
            if(pitchedUpTime.update(Math.abs(mDrive.getPitch().getDegrees()) > gyroInclinationToSwitchToClosedLoop, 1.4)) {
                mBalanceMode = true;
            }
        } else {
            mDrive.setVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                    -mRollController.calculate(mDrive.getPitch().rotateBy(mSteeringDirection).getDegrees()) * mSteeringDirection.cos(),
                    0,
                    mSwerveHeadingController.update(mDrive.getFieldRelativeGyroscopeRotation().getDegrees()),
                    mDrive.getFieldRelativeGyroscopeRotation()));
        }
    }

    @Override
    public boolean isFinished() {
        return (mBalanceMode && mBalanced.update(mRollController.onTarget(1.0), 0.5))
                || (!mBalanceMode && Timer.getFPGATimestamp() - mStartTime > mTimeout);
    }

    @Override
    public void done() {
        mDrive.setVelocity(new ChassisSpeeds());
        mDrive.orientModules(List.of(
                Rotation2d.fromDegrees(45),
                Rotation2d.fromDegrees(-45),
                Rotation2d.fromDegrees(-45),
                Rotation2d.fromDegrees(45)
        ));
    }
}