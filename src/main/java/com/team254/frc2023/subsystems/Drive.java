// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team254.frc2023.subsystems;

import com.ctre.phoenixpro.hardware.Pigeon2;
import com.ctre.phoenixpro.StatusSignalValue;
import com.ctre.phoenixpro.configs.Pigeon2Configuration;
import com.team254.frc2023.Constants;
import com.team254.frc2023.RobotState;
import com.team254.frc2023.paths.TrajectoryGenerator;
import com.team254.frc2023.planners.AutoAlignMotionPlanner;
import com.team254.frc2023.planners.DriveMotionPlanner;
import com.team254.frc2023.planners.HeadingControlPlanner;
import com.team254.lib.drivers.PhoenixProUtil;
import com.team254.lib.drivers.Subsystem;
import com.team254.lib.drivers.SwerveModule;
import com.team254.lib.geometry.*;
import com.team254.lib.loops.ILooper;
import com.team254.lib.loops.Loop;
import com.team254.lib.motion.*;
import com.team254.lib.swerve.ChassisSpeeds;
import com.team254.lib.swerve.SwerveKinematicLimits;
import com.team254.lib.swerve.SwerveModuleState;
import com.team254.lib.swerve.SwerveSetpoint;
import com.team254.lib.swerve.SwerveSetpointGenerator;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.util.Units;
import com.team254.lib.util.Util;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Drive extends Subsystem {

    private final Pigeon2 mPigeon = new Pigeon2(Constants.kPigeonIMUId, Constants.kCANivoreCANBusName);
    /*
     Comp Gyro Orientation:
     Pitch: Inverted
     Roll: Inverted
     Yaw: Correct
     */

    private final SwerveModule[] mModules;
    public static final int kFrontLeftModuleIdx = 0; //3
    public static final int kFrontRightModuleIdx = 1; //2
    public static final int kBackLeftModuleIdx = 2; //1
    public static final int kBackRightModuleIdx = 3; //0

    private SwerveSetpointGenerator mSetpointGenerator;

    private TrajectoryGenerator mTrajectoryGenerator;

    private double mYawOffset;
    private double mRollOffset;
    private double mPitchOffset;

    private final DriveMotionPlanner mDriveMotionPlanner;

    private final AutoAlignMotionPlanner mAutoAlignPlanner;

    private final HeadingControlPlanner mHeadingControllerPlanner;

    private boolean mUseHeadingController = false;

    private boolean mOverrideTrajectory = false;
    private DriveControlState mDriveControlState = DriveControlState.VELOCITY_CONTROL;
    private SwerveKinematicLimits mKinematicLimits = Constants.kUncappedKinematicLimits;
    private Rotation2d mHeadingOffset = Rotation2d.identity();

    private static Drive mInstance = null;

    private ArrayList<StatusSignalValue<?>> mAllSwerveSignalValues;
    private final StatusSignalValue<Double> mPigeonYawStatusSignal;
    private final StatusSignalValue<Double> mPigeonRollStatusSignal;
    private final StatusSignalValue<Double> mPigeonPitchStatusSignal;
    private Pose2d mTargetPoint;

    private ProfileFollower mXController = new ProfileFollower(2.54, 0, 0, 0, 0, 0);
    private ProfileFollower mYController = new ProfileFollower(2.54, 0, 0, 0, 0, 0);
    private ProfileFollower mThetaController = new ProfileFollower(2.54, 0, 0, 0, 0, 0);

    private boolean mSkipConfigChecks = false;

    public static Drive getInstance() {
        if (mInstance == null) {
            mInstance = new Drive();
        }
        return mInstance;
    }

    private Drive() {
        if (Constants.kUseMagEncoders) {
            mModules = new FalconMagSwerveModule[4];
            mModules[kFrontLeftModuleIdx] = new FalconMagSwerveModule(
                Constants.kFrontLeftDriveTalonId,
                Constants.kFrontLeftAziTalonId,
                Constants.kDIOFrontLeftEncoderPortId,
                Constants.kFrontLeftAziEncoderOffset
            );

            mModules[kFrontRightModuleIdx] = new FalconMagSwerveModule(
                Constants.kFrontRightDriveTalonId,
                Constants.kFrontRightAziTalonId,
                Constants.kDIOFrontRightEncoderPortId,
                Constants.kFrontRightAziEncoderOffset
            );

            mModules[kBackLeftModuleIdx] = new FalconMagSwerveModule(
                Constants.kBackLeftDriveTalonId,
                Constants.kBackLeftAziTalonId,
                Constants.kDIOBackLeftEncoderPortId,
                Constants.kBackLeftAziEncoderOffset
            );

            mModules[kBackRightModuleIdx] = new FalconMagSwerveModule(
                Constants.kBackRightDriveTalonId,
                Constants.kBackRightAziTalonId,
                Constants.kDIOBackRightEncoderPortId,
                Constants.kBackRightAziEncoderOffset
            );
        } else {
            mModules = new FalconCANCoderSwerveModule[4];

            mModules[kFrontLeftModuleIdx] = new FalconCANCoderSwerveModule(
                Constants.kFrontLeftDriveTalonId,
                Constants.kFrontLeftAziTalonId,
                Cancoders.getInstance().getFrontLeft(),
                Constants.kFrontLeftAziEncoderOffset
            );

            mModules[kFrontRightModuleIdx] = new FalconCANCoderSwerveModule(
                Constants.kFrontRightDriveTalonId,
                Constants.kFrontRightAziTalonId,
                Cancoders.getInstance().getFrontRight(),
                Constants.kFrontRightAziEncoderOffset
            );

            mModules[kBackLeftModuleIdx] = new FalconCANCoderSwerveModule(
                Constants.kBackLeftDriveTalonId,
                Constants.kBackLeftAziTalonId,
                Cancoders.getInstance().getBackLeft(),
                Constants.kBackLeftAziEncoderOffset
            );

            mModules[kBackRightModuleIdx] = new FalconCANCoderSwerveModule(
                Constants.kBackRightDriveTalonId,
                Constants.kBackRightAziTalonId,
                Cancoders.getInstance().getBackRight(),
                Constants.kBackRightAziEncoderOffset
            );
        }

        Pigeon2Configuration pigeonConfig = new Pigeon2Configuration();
        pigeonConfig.GyroTrim.GyroScalarZ = Constants.kPigeonZScaleFactor;
        mPigeon.getConfigurator().apply(pigeonConfig);
        mPigeonYawStatusSignal = mPigeon.getYaw();
        mPigeonRollStatusSignal = mPigeon.getRoll();
        mPigeonPitchStatusSignal = mPigeon.getPitch();
        PhoenixProUtil.checkErrorAndRetry(() -> mPigeonRollStatusSignal.setUpdateFrequency(200));
        PhoenixProUtil.checkErrorAndRetry(() -> mPigeonPitchStatusSignal.setUpdateFrequency(200));
        PhoenixProUtil.checkErrorAndRetry(() -> mPigeonYawStatusSignal.setUpdateFrequency(200));

        mAllSwerveSignalValues = new ArrayList<>();
        for (SwerveModule module : mModules) {
            mAllSwerveSignalValues.add(module.getDrivePositionSignalValue());
            mAllSwerveSignalValues.add(module.getDriveVelocitySignalValue());
            mAllSwerveSignalValues.add(module.getSteerPositionSignalValue());
            mAllSwerveSignalValues.add(module.getSteerVelocitySignalValue());
        }
        mAllSwerveSignalValues.add(mPigeonYawStatusSignal);
        mAllSwerveSignalValues.add(mPigeonRollStatusSignal);
        mAllSwerveSignalValues.add(mPigeonPitchStatusSignal);

        mYawOffset = mPigeonYawStatusSignal.asSupplier().get();
        mRollOffset = mPigeonRollStatusSignal.asSupplier().get();
        mPitchOffset = mPigeonPitchStatusSignal.asSupplier().get();
        readGyro();
        readModules();
        setSetpointFromMeasured();



        mSetpointGenerator = new SwerveSetpointGenerator(Constants.kKinematics);

        mDriveMotionPlanner = new DriveMotionPlanner(Constants.kKinematics, Constants.kSmoothKinematicLimits);
        mTrajectoryGenerator = new TrajectoryGenerator(mDriveMotionPlanner);
        mAutoAlignPlanner = new AutoAlignMotionPlanner();
        mHeadingControllerPlanner = new HeadingControlPlanner();
        resetAzimuth();
    }

    public TrajectoryGenerator getTrajectoryGenerator() {
        return mTrajectoryGenerator;
    }

    public static class PeriodicIO {
        // input/measured
        double timestamp;
        ChassisSpeeds des_chassis_speeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        SwerveModuleState[] measured_states = new SwerveModuleState[] {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
        };

        double[] abs_azi_angles_deg = new double[] {0, 0, 0, 0};

        double auto_align_ytheta_vx_field = 0.0;

        Rotation2d heading = Rotation2d.identity();
        Rotation2d roll = Rotation2d.identity();
        Rotation2d pitch = Rotation2d.identity();

        // outputs
        SwerveSetpoint setpoint = new SwerveSetpoint(new ChassisSpeeds(), new SwerveModuleState[Constants.kKinematics.getNumModules()]);
        Translation2d translational_error = Translation2d.identity();
        Rotation2d heading_error = Rotation2d.identity();
        TimedState<Pose2dWithMotion> path_setpoint = new TimedState<Pose2dWithMotion>(Pose2dWithMotion.identity());

        boolean want_orient = false;
    }

    private final PeriodicIO mPeriodicIO = new PeriodicIO();

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
     * 'forwards' direction.
     */
    public synchronized void zeroGyroscope() {
        mYawOffset = mPigeonYawStatusSignal.asSupplier().get();
        mRollOffset = mPigeonRollStatusSignal.asSupplier().get();
        mPitchOffset = mPigeonPitchStatusSignal.asSupplier().get();
        readGyro();
    }

    public synchronized void setSwerveModuleAziTuning(Rotation2d angle) {
        mModules[0].setSteerAngleUnclamped(angle.getRadians(), false);
    }

    public void syncDriveSignals() {
        final double kMaxTimingSkew = 0.002; // Seconds
        final int kNumRetries = 0;
        for (int i = 0; i <= kNumRetries; ++i) {
            double oldest_timestamp = Double.MAX_VALUE;
            double newest_timestamp = Double.MIN_VALUE;
            for (StatusSignalValue<?> val : mAllSwerveSignalValues) {
                val.refresh();
                double ts = val.getAllTimestamps().getCANivoreTimestamp().getTime();
                if (ts < oldest_timestamp) { oldest_timestamp = ts; }
                if (ts > newest_timestamp) { newest_timestamp = ts; }
            }
            if (newest_timestamp - oldest_timestamp < kMaxTimingSkew) {
                break;
            } else if (i == kNumRetries) {
            }
        }
    }

    public Rotation2d getHeadingOffset() {
        return mHeadingOffset;
    }

    public synchronized void setHeading(Rotation2d heading) {
        mHeadingOffset = heading;
        mYawOffset += mPeriodicIO.heading.getDegrees() - heading.getDegrees();
    }

    public synchronized Rotation2d getFieldRelativeGyroscopeRotation() {
        return mPeriodicIO.heading;
    }


    public double getYawOffset() {
        return mYawOffset;
    }

    protected synchronized void readGyro() {
        // Reads are synchronized.
        mPeriodicIO.heading = Rotation2d.fromDegrees(mPigeonYawStatusSignal.getValue() - mYawOffset);
        mPeriodicIO.roll = Rotation2d.fromDegrees(mPigeonRollStatusSignal.getValue() - mRollOffset);
        mPeriodicIO.pitch = Rotation2d.fromDegrees(mPigeonPitchStatusSignal.getValue() - mPitchOffset);
    }

    // Returns unsigned tilt of robot.
    public synchronized Rotation2d getTilt() {
        // roll, pitch, yaw
        Rotation3d rot = new Rotation3d(mPeriodicIO.roll.getRadians(),
                mPeriodicIO.pitch.getRadians(),
                mPeriodicIO.heading.getRadians());
        Transform3d transform = new Transform3d(new Translation3d(0.0, 0.0, 0.0), rot);
        Pose3d robot_normal = new Pose3d(0.0, 0.0, 1.0, new Rotation3d(0.0, 0.0, 0.0))
                .transformBy(transform);

        // Normalized robot_normal to make life easier.
        robot_normal = robot_normal.times(1.0 / Math.sqrt(robot_normal.getX()*robot_normal.getX()
                + robot_normal.getY()*robot_normal.getY() + robot_normal.getZ()*robot_normal.getZ()));

        // Normal of ground is (0, 0, 1), robot_normal is normalized.
        return Rotation2d.fromRadians(Math.acos(robot_normal.getZ()));
    }

    public synchronized Rotation2d getRoll() {
        return mPeriodicIO.roll;
    }

    public synchronized Rotation2d getPitch() {
        return mPeriodicIO.pitch;
    }

    public synchronized void resetRoll() {
        mRollOffset = mPigeonRollStatusSignal.asSupplier().get();
    }

    public synchronized SwerveModuleState[] getModuleStates() {
        return mPeriodicIO.measured_states;
    }

    public synchronized ChassisSpeeds getDesiredChassisSpeeds() {
        return mPeriodicIO.des_chassis_speeds;
    }

    public SwerveModuleState[] getDesiredModuleStates () {
        return mPeriodicIO.setpoint.mModuleStates;
    }

    public synchronized SwerveSetpoint getSetpoint () {
        return mPeriodicIO.setpoint;
    }


    public synchronized void setVelocity(ChassisSpeeds chassisSpeeds) {
        if (Constants.kUseVelocityDrive) {
            mDriveControlState = DriveControlState.VELOCITY_CONTROL;
        } else {
            mDriveControlState = DriveControlState.OPEN_LOOP;
        }
        mPeriodicIO.des_chassis_speeds = chassisSpeeds;
        mPeriodicIO.want_orient = false;
    }

    public synchronized void setWantOrient(boolean wantOrient) {
        mPeriodicIO.want_orient = wantOrient;
        if (!wantOrient) {
            for (int i = 0; i < mModules.length; i++) {
                mModules[i].stop();
            }
        }
    }

    public synchronized void orientModules(List<Rotation2d> orientations) {
        setWantOrient(true);
        for (int i = 0; i < mModules.length; ++i) {
            mPeriodicIO.setpoint.mModuleStates[i] =
                    new SwerveModuleState(0.0, orientations.get(i));
        }
    }

    public synchronized void orientModules(Rotation2d orientation) {
        setWantOrient(true);
        for (int i = 0; i < mModules.length; ++i) {
            mPeriodicIO.setpoint.mModuleStates[i] = new SwerveModuleState(0.0, orientation);
        }
    }

    public synchronized MotionState getXSetpoint() {
        return mXController.getSetpoint();
    }

    public synchronized MotionState getYSetpoint() {
        return mYController.getSetpoint();
    }
    public synchronized MotionState getThetaSetpoint() {
        return mThetaController.getSetpoint();
    }


    @Override
    public synchronized void readPeriodicInputs() {
        syncDriveSignals();
        readGyro();
        readModules();
    }

    private void readModules() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();
        for (int i = 0; i < mModules.length; ++i) {
            mPeriodicIO.measured_states[i] = mModules[i].getPreviewedState();
            if (mModules[i].getSteerAngle() != null) {
                mPeriodicIO.abs_azi_angles_deg[i] = mModules[i].getSteerEncoderAngle().getDegrees();
            }
        }
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        for (int i = 0; i < mModules.length; ++i) {
            if (!mPeriodicIO.want_orient) {
                if (mDriveControlState == DriveControlState.VELOCITY_CONTROL || mDriveControlState == DriveControlState.PATH_FOLLOWING) {
                    mModules[i].setWithVelocityShortestPath(mPeriodicIO.setpoint.mModuleStates[i].speedMetersPerSecond,
                            mPeriodicIO.setpoint.mModuleStates[i].angle, false);
                } else if (mDriveControlState == DriveControlState.AUTO_ALIGN || mDriveControlState == DriveControlState.AUTO_ALIGN_Y_THETA) {
                    mModules[i].setWithVelocityShortestPath(mPeriodicIO.setpoint.mModuleStates[i].speedMetersPerSecond,
                            mPeriodicIO.setpoint.mModuleStates[i].angle, true);
                } else if (mDriveControlState == DriveControlState.OPEN_LOOP) {
                    mModules[i].setWithVoltageShortestPath(
                            mPeriodicIO.setpoint.mModuleStates[i].speedMetersPerSecond / Constants.kMaxVelocityMetersPerSecond,
                            mPeriodicIO.setpoint.mModuleStates[i].angle, false);
                }
            } else {
                mModules[i].setWithVoltageShortestPath(0.0, mPeriodicIO.setpoint.mModuleStates[i].angle, false);
            }
        }
        double timestamp = Timer.getFPGATimestamp();
    }

    public synchronized void setSetpointFromMeasured() {
        for (int i = 0; i < mModules.length; ++i) {
            mPeriodicIO.setpoint.mModuleStates[i] = mPeriodicIO.measured_states[i];
        }
        mPeriodicIO.setpoint.mChassisSpeeds = Constants.kKinematics.toChassisSpeedWheelConstraints(mPeriodicIO.setpoint.mModuleStates);
    }

    public synchronized Pose2d getAutonSetpoint() {
        return mPeriodicIO.path_setpoint.state().getPose();
    }

    // Reconfigure periodically in case an error was thrown the first time
    public void reconfigureTalons() {
        for (SwerveModule module : mModules) {
            module.configureTalons();
        }
    }

    public enum DriveControlState {
        OPEN_LOOP,
        VELOCITY_CONTROL,
        PATH_FOLLOWING,
        AUTO_ALIGN,
        AUTO_ALIGN_Y_THETA
    }

    public synchronized void setTrajectory(TrajectoryIterator<TimedState<Pose2dWithMotion>> trajectory) {
        if (mDriveMotionPlanner != null) {
            mOverrideTrajectory = false;
            mDriveMotionPlanner.reset();
            mDriveMotionPlanner.setTrajectory(trajectory);
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
        }
    }

    public synchronized boolean isDoneWithTrajectory() {
        if (mDriveMotionPlanner == null || mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            return false;
        }
        return mDriveMotionPlanner.isDone() || mOverrideTrajectory;
    }

    public synchronized void overrideTrajectory(boolean value) {
        mOverrideTrajectory = value;
    }

    public synchronized void setKinematicLimits(SwerveKinematicLimits limits) {
        if (limits != mKinematicLimits) {
            mKinematicLimits = limits;
        }
    }

    private void updatePathFollower() {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING) {
            final double now = Timer.getFPGATimestamp();
            ChassisSpeeds output =  mDriveMotionPlanner.update(now,
                    RobotState.getInstance().getFieldToVehicle(now), RobotState.getInstance().getMeasuredVelocity());
            if (output != null) {
                if (!mPeriodicIO.want_orient) {
                    mPeriodicIO.des_chassis_speeds = output;
                }
            }

            mPeriodicIO.translational_error = mDriveMotionPlanner.getTranslationalError();
            mPeriodicIO.heading_error = mDriveMotionPlanner.getHeadingError();
            mPeriodicIO.path_setpoint = mDriveMotionPlanner.getSetpoint();
        } else {
            DriverStation.reportError("Drive is not in path following state", false);
        }
    }

    public synchronized void resetAzimuth() {
        for (SwerveModule module : mModules) {
            module.rezeroSteeringMotor();
        }
        readModules();
    }

    public synchronized void setSnapToPoint(Pose2d targetPoint) {
        if (mAutoAlignPlanner != null) {
            mOverrideTrajectory = false;
            if (mDriveControlState != DriveControlState.AUTO_ALIGN) {
                mAutoAlignPlanner.reset();
                mDriveControlState = DriveControlState.AUTO_ALIGN;
            }
            mAutoAlignPlanner.setTargetPoint(targetPoint);
            mTargetPoint = targetPoint;
            RobotState.getInstance().setDisplaySetpointPose(targetPoint);
        }
    }

    /**
     * Snap in Y and Theta and allow driver control in field x direction. Uses the same auto align planner,
     * but ignores the X output of it.
     *
     * TargetPoint is in the absolute field frame.
     * @param targetPoint
     */
    public synchronized void setSnapYTheta(Pose2d targetPoint, double vxMetersPerSecond) {
        if (mAutoAlignPlanner != null) {
            mOverrideTrajectory = false;
            if (mDriveControlState != DriveControlState.AUTO_ALIGN_Y_THETA) {
                mAutoAlignPlanner.reset();
                mDriveControlState = DriveControlState.AUTO_ALIGN_Y_THETA;
            }
            mAutoAlignPlanner.setTargetPoint(targetPoint);
        }
        mPeriodicIO.auto_align_ytheta_vx_field = vxMetersPerSecond;
    }

    private void updateAutoAlign() {
        if (mDriveControlState != DriveControlState.AUTO_ALIGN_Y_THETA && mDriveControlState != DriveControlState.AUTO_ALIGN) {
            return;
        }
        final double now = Timer.getFPGATimestamp();
        var field_to_odom = RobotState.getInstance().getAbsoluteFieldToOdom(now);
        var odom_to_vehicle = RobotState.getInstance().getOdomToVehicle(now);
        ChassisSpeeds output = mAutoAlignPlanner.updateAutoAlign(now, odom_to_vehicle, Pose2d.fromTranslation(field_to_odom), RobotState.getInstance().getMeasuredVelocity());

        // Find x error in field frame.
        var x_error_field = RobotState.getInstance().getFieldToVehicleAbsolute(now).getTranslation().x() -
               mTargetPoint.getTranslation().x();
        var offset = Util.limit(Units.meters_to_inches(x_error_field) / Math.cos(Math.toRadians(15.0)), -1.0, 1.0);
        Superstructure.getInstance().setScoringOffset(offset);

        if (output != null) {
            if (!mPeriodicIO.want_orient) {
                if (mDriveControlState == DriveControlState.AUTO_ALIGN_Y_THETA) {
                    // in y-theta mode, use the cached desired speed for x direction

                    double fieldRelativeXCommand = mPeriodicIO.auto_align_ytheta_vx_field;
                    double fieldRelativeYCommand = new Translation2d(output.vxMetersPerSecond, output.vyMetersPerSecond)
                            .rotateBy(getFieldRelativeGyroscopeRotation())
                            .y();

                    mPeriodicIO.des_chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                            fieldRelativeXCommand,
                            fieldRelativeYCommand,
                            output.omegaRadiansPerSecond,
                            getFieldRelativeGyroscopeRotation());
                } else {
                    mPeriodicIO.des_chassis_speeds = output;
                }
            }
        }

    }

    private void updateHeadingController() {
        if (mUseHeadingController) {
            mPeriodicIO.des_chassis_speeds = mHeadingControllerPlanner.update(Timer.getFPGATimestamp(), mPeriodicIO.des_chassis_speeds, RobotState.getInstance().getMeasuredVelocity(), getFieldRelativeGyroscopeRotation());
        }
    }

    private void updateDesiredStates() {
        if (mPeriodicIO.want_orient) return;
        // Set the des_states to account for robot traversing arc.
        Pose2d robot_pose_vel = new Pose2d(mPeriodicIO.des_chassis_speeds.vxMetersPerSecond * Constants.kLooperDt,
                mPeriodicIO.des_chassis_speeds.vyMetersPerSecond * Constants.kLooperDt,
                Rotation2d.fromRadians(mPeriodicIO.des_chassis_speeds.omegaRadiansPerSecond * Constants.kLooperDt));
        Twist2d twist_vel = Pose2d.log(robot_pose_vel).scaled(1.0 / Constants.kLooperDt);
        ChassisSpeeds updated_chassis_speeds = new ChassisSpeeds(twist_vel.dx, twist_vel.dy, twist_vel.dtheta);
        mPeriodicIO.setpoint = mSetpointGenerator.generateSetpoint(mKinematicLimits, mPeriodicIO.setpoint, updated_chassis_speeds, Constants.kLooperDt);
    }

    @Override
    public void registerEnabledLoops(ILooper in) {
        in.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                setKinematicLimits(Constants.kUncappedKinematicLimits);
                for (int i = 0; i < mModules.length; i++) {
                    if (mSkipConfigChecks) {
                        mModules[i].setSteerBrakeModeUnchecked();
                        mModules[i].setDriveBreakModeUnchecked();
                    } else {
                        mModules[i].setSteerBrakeMode();
                        mModules[i].setDriveBrakeMode();
                    }
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Drive.this) {
                    // Reset heading controller (if needed)
                    if (mDriveControlState != DriveControlState.OPEN_LOOP &&  mDriveControlState != DriveControlState.VELOCITY_CONTROL) {
                        // Only use heading controller in open loop mode
                        mUseHeadingController = false;
                    }
                    if (!mUseHeadingController) {
                        // Reset yaw goal when this flag is off
                        mHeadingControllerPlanner.reset();
                    }


                    // Handle drive modes
                    switch (mDriveControlState) {
                        case PATH_FOLLOWING:
                            setKinematicLimits(Constants.kFastKinematicLimits);
                            updatePathFollower();
                            break;
                        case OPEN_LOOP:
                        case VELOCITY_CONTROL:
                            setKinematicLimits(Constants.kUncappedKinematicLimits);
                            //updateHeadingController();
                            break;
                        case AUTO_ALIGN:
                            // fall-through
                        case AUTO_ALIGN_Y_THETA:
                            setKinematicLimits(Constants.kUncappedKinematicLimits);
                            updateAutoAlign();
                            break;
                        default:
                            break;
                    }
                    updateDesiredStates();
                }
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }

    public synchronized void setEnableHeadingController(boolean enabled) {
        mUseHeadingController = enabled;
    }

    public synchronized void setHeadingControllerGoal(Rotation2d goal) {
        mHeadingControllerPlanner.setYawGoal(goal);
    }


    public DriveControlState getDriveControlState() {
        return mDriveControlState;
    }

    public synchronized double[] getSteerClosedLoopErrors() {
        double[] rv = new double[]{0, 0, 0, 0};
        for (int i = 0; i < mModules.length; ++i) {
            rv[i] = mModules[i].getSteerClosedLoopError();
        }
        return rv;
    }

    public synchronized double[] getDriveClosedLoopError() {
        double[] rv = new double[]{0, 0, 0, 0};
        for (int i = 0; i < mModules.length; ++i) {
            rv[i] = mModules[i].getDriveClosedLoopError();
        }
        return rv;
    }

    public synchronized Pose2d getAutonError() {
        return new Pose2d(mDriveMotionPlanner.getTranslationalError(), mDriveMotionPlanner.getHeadingError());
    }

    public synchronized boolean getAutoAlignComplete() {
        return mAutoAlignPlanner.getAutoAlignComplete();
    }

    public synchronized boolean isAutoAlignActive() {
        return mDriveControlState == DriveControlState.AUTO_ALIGN || mDriveControlState == DriveControlState.AUTO_ALIGN_Y_THETA;
    }

    @Override
    public void stop() {
        setVelocity(new ChassisSpeeds());
        for (int i = 0; i < mModules.length; i++) {
            if (mSkipConfigChecks) {
                mModules[i].setSteerCoastModeUnchecked();
            } else {
                mModules[i].setSteerCoastMode();
            }

            // Do not flip coast between auto and teleop.
            if (!RobotState.getInstance().getHasBeenEnabled()) {
                if (mSkipConfigChecks) {
                    mModules[i].setDriveCoastModeUnchecked();
                } else {
                    mModules[i].setDriveCoastMode();
                }
            }
        }
    }

    public double getLatestSychronizedTimestamp() {

        return mModules[0].getDrivePositionSignalValue().getTimestamp().getTime();
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public synchronized void outputTelemetry(boolean disabled) {
        if (disabled) {
            SmartDashboard.putString("Desired Chassis Speeds", mPeriodicIO.des_chassis_speeds.toString());
            SmartDashboard.putString("Desired Chassis Speeds from Setpoint", mPeriodicIO.setpoint.mChassisSpeeds.toString());
            SmartDashboard.putString("Gyro Angle Rot", getFieldRelativeGyroscopeRotation().toString());

            SmartDashboard.putNumber("Front Left Absolute Encoder Angle", mPeriodicIO.abs_azi_angles_deg[kFrontLeftModuleIdx]);
            SmartDashboard.putNumber("Front Right Absolute Encoder Angle", mPeriodicIO.abs_azi_angles_deg[kFrontRightModuleIdx]);
            SmartDashboard.putNumber("Back Left Absolute Encoder Angle", mPeriodicIO.abs_azi_angles_deg[kBackLeftModuleIdx]);
            SmartDashboard.putNumber("Back Right Absolute Encoder Angle", mPeriodicIO.abs_azi_angles_deg[kBackRightModuleIdx]);

            SmartDashboard.putNumber("X Error", mPeriodicIO.translational_error.x());
            SmartDashboard.putNumber("Y Error", mPeriodicIO.translational_error.y());

            SmartDashboard.putNumber("Auton heading error", mPeriodicIO.heading_error.getDegrees());

        SmartDashboard.putNumber("pitch", getPitch().getDegrees());
        SmartDashboard.putNumber("roll", getRoll().getDegrees());

        }
    }

    public void setSkipConfigChecks(boolean skip) {
        mSkipConfigChecks = skip;
    }

    @Override
    public void rewriteDeviceConfiguration() {
        for (SwerveModule module : mModules) {
            module.rewriteDeviceConfiguration();
        }
    }

    @Override
    public boolean checkDeviceConfiguration() {
        return Arrays.stream(mModules).allMatch((module) -> module.checkDeviceConfiguration());
    }

    public void printError() {
        if (mDriveMotionPlanner == null) return;
        mDriveMotionPlanner.getErrorTracker().printSummary();
    }
}
