package com.team254.frc2023.auto.actions;

import com.team254.frc2023.Robot;
import com.team254.frc2023.RobotState;
import com.team254.frc2023.subsystems.Drive;
import com.team254.lib.control.SwerveHeadingController;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.swerve.ChassisSpeeds;
import com.team254.lib.util.TimeDelayedBoolean;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;

import java.util.List;
import java.util.Optional;

public class DockActionNew implements Action {

    enum DockingState {
        GROUND_TO_CLIMB,
        LEVEL_FINE_ADJUST,
        CLIMB_DRIVE_CENTER,
        LEVEL
    }

    public enum StartingSide {
        NEAR,
        FAR
    }

    public enum ApproachOrientation {
        FRONT_FIRST,
        BACK_FIRST
    }

    private static final Drive mDrive = Drive.getInstance();
    private static final SwerveHeadingController mSwerveHeadingController = SwerveHeadingController.getInstance();

    private DockingState mDockingState = DockingState.GROUND_TO_CLIMB;

    private static final double gyroInclinationToSwitchToClimbing = 8.0; // degrees
    private final double mGroundVel = 1.0; // m/s
    private final double mCenteringVel = 1.0; // m/s
    private double mUntipVel = 0.20; // m/s
    private double mStartTime;
    private final double mTimeout;

    private Optional<Double> mLevelStartTime = Optional.empty();

    private static final double gyroIsLevelAngle = 6.0;
    public static final double driveDistanceStaticMetersBackFirst = 1.1;
    public static final double driveDistanceStaticMetersFrontFirst = 1.35;
    private static final double levelingTime = 0.5;

    TimeDelayedBoolean unleveled = new TimeDelayedBoolean();

    private Rotation2d mSteeringDirection; // field relative
    private Optional<Pose2d> mInitialTipPose = Optional.empty();
    private boolean mBalanceMode = false;
    private StartingSide mApproachSide;
    private ApproachOrientation mOrientation;
    private double mLevelingDistance;

    public DockActionNew(StartingSide approachSide, ApproachOrientation orientation, double leveling_distance, double timeout) {
        mOrientation = orientation;
        mApproachSide = approachSide;
        mSteeringDirection = approachSide == StartingSide.FAR ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0);
        mTimeout = timeout;
        mDockingState = DockingState.GROUND_TO_CLIMB;
        mLevelingDistance = leveling_distance;
    }

    public DockActionNew(StartingSide approachSide, ApproachOrientation orientation, double leveling_distance, double timeout, double untipVel) {
        mOrientation = orientation;
        mApproachSide = approachSide;
        mSteeringDirection = approachSide == StartingSide.FAR ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0);
        mTimeout = timeout;
        mDockingState = DockingState.GROUND_TO_CLIMB;
        mLevelingDistance = leveling_distance;
        mUntipVel = untipVel;
    }

    private DockingState handleStateTransitions(double timestamp) {
        switch (mDockingState) {
            case GROUND_TO_CLIMB:
                // transition: gyro over X degrees -> CLIMB_DRIVE_CENTER
                if (Math.abs(getInclination()) >= gyroInclinationToSwitchToClimbing) {
                    mInitialTipPose = Optional.of(RobotState.getInstance().getOdomToVehicle(timestamp));
                    System.out.println("T: g climb -> d center");
                    return DockingState.CLIMB_DRIVE_CENTER;
                }
                break;
            case CLIMB_DRIVE_CENTER:
                // transition: drives Y inches -> CLIMB_TO_LEVEL
                Pose2d currentPose = RobotState.getInstance().getOdomToVehicle(timestamp);
                if (mInitialTipPose.isPresent() && mInitialTipPose.get().distance(currentPose) >= mLevelingDistance) {
                    System.out.println("T: d center -> level fine (dist trig)");
                    return DockingState.LEVEL_FINE_ADJUST;
                }
                // alt transition: if we hit the level range, just immediately go to fine adjustment mode,
                // means we probably overshot and this does damage control
                if (Math.abs(getInclination()) <= gyroIsLevelAngle) {
                    System.out.println("T: d center -> level fine (gyro trig)");
                    return DockingState.LEVEL_FINE_ADJUST;
                }
                break;
            case LEVEL_FINE_ADJUST:
                // transition: gyro at 0ish for 0.5s -> LEVEL
                if (Math.abs(getInclination()) <= gyroIsLevelAngle) {
                    if (mLevelStartTime.isPresent() && timestamp - mLevelStartTime.get() >= levelingTime) {
                        System.out.println("T: level fine -> level");
                        return DockingState.LEVEL;
                    } else if (mLevelStartTime.isEmpty()) {
                        mLevelStartTime = Optional.of(timestamp);
                    }
                } else {
                    // we aren't level, try again
                    mLevelStartTime = Optional.empty();
                }
                break;
            case LEVEL:
                break;
        }
        // by default do not transition
        return mDockingState;
    }

    private void processStates() {
        switch (mDockingState) {
            case GROUND_TO_CLIMB:
                handleClimbing();
                break;
            case CLIMB_DRIVE_CENTER:
                handleCenterMove();
                break;
            case LEVEL_FINE_ADJUST:
                handleLeveling();
                break;
            case LEVEL:
                handleLevel();
                break;
        }
    }
    private double getInclination() {

        Transform3d gyro = new Transform3d(new Translation3d(), new Rotation3d(mDrive.getRoll().getRadians(), mDrive.getPitch().getRadians(), mDrive.getFieldRelativeGyroscopeRotation().getRadians()));
        Transform3d rot = new Transform3d(new Translation3d(), new Rotation3d(0, 0, -mDrive.getFieldRelativeGyroscopeRotation().getRadians()));

        Rotation3d out = gyro.plus(rot).getRotation();

        // negative because returned value is negative is near side pitch when robot is backwards, and we want opposite convention
        return -Math.toDegrees(out.getY());
    }

    public boolean getInclinationAboveThresholdForDock() {
        return Math.abs(getInclination()) >= gyroInclinationToSwitchToClimbing;
    }


    private void handleClimbing() {
        mDrive.setVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                mGroundVel * mSteeringDirection.cos(),
                0,
                mSwerveHeadingController.update(mDrive.getFieldRelativeGyroscopeRotation().getDegrees()),
                mDrive.getFieldRelativeGyroscopeRotation()));
    }

    private void handleCenterMove() {
        mDrive.setVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                mCenteringVel * mSteeringDirection.cos(),
                0,
                mSwerveHeadingController.update(mDrive.getFieldRelativeGyroscopeRotation().getDegrees()),
                mDrive.getFieldRelativeGyroscopeRotation()));
    }

    private void handleLeveling() {

        // positive inclination means its on the near side of bridge, negative inclination means its on the far side of bridge
        double incl = -getInclination();
        if (incl < gyroIsLevelAngle && incl > -gyroIsLevelAngle) {
            mDrive.setVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                    0,
                    0,
                    mSwerveHeadingController.update(mDrive.getFieldRelativeGyroscopeRotation().getDegrees()),
                    mDrive.getFieldRelativeGyroscopeRotation()));
        } else if (incl > gyroIsLevelAngle) {
            // near side
            mDrive.setVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                    mUntipVel * (mApproachSide == StartingSide.NEAR ? mSteeringDirection.cos() : -mSteeringDirection.cos()),
                    0,
                    mSwerveHeadingController.update(mDrive.getFieldRelativeGyroscopeRotation().getDegrees()),
                    mDrive.getFieldRelativeGyroscopeRotation()));
        } else if (incl < -gyroIsLevelAngle) {
            // far side
            mDrive.setVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                    mUntipVel * (mApproachSide == StartingSide.FAR ? mSteeringDirection.cos() : -mSteeringDirection.cos()),
                    0,
                    mSwerveHeadingController.update(mDrive.getFieldRelativeGyroscopeRotation().getDegrees()),
                    mDrive.getFieldRelativeGyroscopeRotation()));
        }
    }

    private void handleLevel() {
        mDrive.setVelocity(new ChassisSpeeds());
        mDrive.orientModules(List.of(
                Rotation2d.fromDegrees(45),
                Rotation2d.fromDegrees(-45),
                Rotation2d.fromDegrees(-45),
                Rotation2d.fromDegrees(45)
        ));
    }

    @Override
    public void start() {
        mStartTime = Timer.getFPGATimestamp();
        mSwerveHeadingController.setHeadingControllerState(SwerveHeadingController.HeadingControllerState.MAINTAIN);
        mSwerveHeadingController.setGoal(mDrive.getFieldRelativeGyroscopeRotation().getDegrees());
    }

    @Override
    public void update() {
        double t = Timer.getFPGATimestamp();
        mDockingState = handleStateTransitions(t);
        processStates();
    }

    @Override
    public boolean isFinished() {
        return mDockingState == DockingState.LEVEL || (Timer.getFPGATimestamp() - mStartTime) > mTimeout || (Timer.getFPGATimestamp() - Robot.getAutoInitStartTime() > 14.9);
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