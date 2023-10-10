// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team254.frc2023;

import java.util.Optional;

import com.team254.frc2023.planners.AutoAlignPointSelector;
import com.team254.frc2023.subsystems.*;
import com.team254.lib.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.LoggedRobot;

import com.team254.frc2023.auto.AutoModeExecutor;
import com.team254.frc2023.auto.modes.AutoModeBase;
import com.team254.frc2023.controlboard.ControlBoard;
import com.team254.frc2023.subsystems.Cancoders;
import com.team254.frc2023.subsystems.Drive;
import com.team254.frc2023.subsystems.LED.WantedAction;
import com.team254.frc2023.subsystems.Elevator;
import com.team254.frc2023.subsystems.LED;
import com.team254.frc2023.subsystems.Laterator;
import com.team254.frc2023.subsystems.Limelight;
import com.team254.frc2023.subsystems.RobotStateEstimator;
import com.team254.frc2023.subsystems.Superstructure;
import com.team254.lib.control.SwerveHeadingController;
import com.team254.lib.control.SwerveHeadingController.HeadingControllerState;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.loops.Looper;
import com.team254.lib.swerve.ChassisSpeeds;
import com.team254.lib.util.CrashTracker;
import com.team254.lib.util.LatchedBoolean;
import com.team254.lib.util.TimeDelayedBoolean;
import com.team254.lib.util.ToggleBoolean;
import com.team254.lib.util.Util;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends LoggedRobot {
    private final Looper mEnabledLooper = new Looper(Constants.kLooperDt);
    private final Looper mDisabledLooper = new Looper(Constants.kLooperDt);

    private final ControlBoard mControlBoard = ControlBoard.getInstance();

    private PowerDistribution mPowerDistribution;

    private SwerveHeadingController mSwerveHeadingController;

    private final AutoModeExecutor mAutoModeExecutor = new AutoModeExecutor();
    private final AutoModeSelector mAutoModeSelector = new AutoModeSelector();

    private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();
    private Cancoders mCancoders;
    private Drive mDrive;
    private Elevator mElevator;
    private Laterator mLaterator;
    private GroundIntakeDeploy mGroundIntakeDeploy;
    private GroundIntakeRoller mGroundIntakeRoller;
    private Superstructure mSuperstructure;
    private Optional<Double> mHeadingGoal;
    private RollerClaw mRollerClaw;
    private RobotStateEstimator mRobotStateEstimator;
    private Limelight mLimelight;
    private LED mLED;
    private Forks mForks;

    private int mIter = 0;
    private static final double kScaleTranslationInputs = 1;
    private static final double kScaleRotationInputs = 0.5;

    private static double mAutoInitStartTime;
    private boolean hasBeenInAuto = false;
    private LatchedBoolean mPushDesired = new LatchedBoolean();

    public Robot() {
        super();

    }

    @Override
    public void robotInit() {
        CrashTracker.logRobotInit();
        try {
            if (!Constants.kUseMagEncoders) {
                mCancoders = Cancoders.getInstance();
                double startInitTs = Timer.getFPGATimestamp();
                System.out.println("* Starting to init cancoders at ts " +  startInitTs);
                while (Timer.getFPGATimestamp() - startInitTs < Constants.kCancoderBootAllowanceSeconds && !mCancoders.allHaveBeenInitialized()) {
                    Timer.delay(0.1);
                }
                System.out.println("* Cancoders all inited: Took " + (Timer.getFPGATimestamp() - startInitTs) + " seconds");
            }

            mDrive = Drive.getInstance();
            mElevator = Elevator.getInstance();
            mLaterator = Laterator.getInstance();
            mRollerClaw = RollerClaw.getInstance();
            mLimelight = Limelight.getInstance();
            mGroundIntakeDeploy = GroundIntakeDeploy.getInstance();
            mGroundIntakeRoller = GroundIntakeRoller.getInstance();
            mSuperstructure = Superstructure.getInstance();
            mRobotStateEstimator = RobotStateEstimator.getInstance();
            mLED = LED.getInstance();
            mForks = Forks.getInstance();
            mHeadingGoal = Optional.of(mDrive.getFieldRelativeGyroscopeRotation().getDegrees());
            mSwerveHeadingController = SwerveHeadingController.getInstance();
            mPowerDistribution = new PowerDistribution();

            mSubsystemManager.setSubsystems(
                    mDrive,
                    mRobotStateEstimator,
                    mSuperstructure,
                    mElevator,
                    mLaterator,
                    mRollerClaw,
                    mGroundIntakeDeploy,
                    mGroundIntakeRoller,
                    mLimelight,
                    mForks
            );

            mDrive.setSkipConfigChecks(true);
            mAutoModeSelector.updateModeCreator();
            mDrive.getTrajectoryGenerator().generateTrajectories();
            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            mSubsystemManager.registerDisabledLoops(mDisabledLooper);
            mLED.registerEnabledLoops(mEnabledLooper);
            mLED.registerEnabledLoops(mDisabledLooper);
            mLimelight.start();
            mRobotStateEstimator.registerEnabledLoops(mDisabledLooper);
            mRobotStateEstimator.resetOdometry(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(180)));
            RobotState.getInstance().resetKalmanFilters();
            mDrive.setHeading(Rotation2d.fromDegrees(180));
            mSubsystemManager.stop();

            SmartDashboard.putBoolean("Is Practice Bot", Constants.kPracticeBot);
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void robotPeriodic() {}

    private void outputTelemetry(boolean disabled) {
        mSubsystemManager.outputToSmartDashboard(disabled);
        mAutoModeSelector.outputToSmartDashboard();
    }

    @Override
    public void autonomousInit() {
        try {
            mAutoInitStartTime = Timer.getFPGATimestamp();
            CrashTracker.logAutoInit();
            mLimelight.setDisableProcessing(true);
            hasBeenInAuto = true;

            mSuperstructure.setSkipConfigChecks(true);
            mDrive.setSkipConfigChecks(true);
            mDrive.zeroGyroscope();
            mRobotStateEstimator.resetOdometry(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(180)));
            mDrive.setHeading(Rotation2d.fromDegrees(180));
            mDisabledLooper.stop();
            mSubsystemManager.stop();
            mEnabledLooper.start();
            mAutoModeExecutor.start();
            RobotState.getInstance().setHasBeenEnabled(true);
            mLED.setWantedAction(WantedAction.DISPLAY_VISION);
            mSuperstructure.setHasBeenHomed(true);
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    public static double getAutoInitStartTime() {
        return mAutoInitStartTime;
    }

    @Override
    public void autonomousPeriodic() {
//        try {
//            outputTelemetry(true);
//        } catch (Throwable t) {
//            CrashTracker.logThrowableCrash(t);
//            throw t;
//        }
    }

    @Override
    public void teleopInit() {
        try {
            CrashTracker.logTeleopInit();
            mDrive.setSkipConfigChecks(false);
            mLimelight.setDisableProcessing(false);
            mDisabledLooper.stop();
            mSubsystemManager.stop();
            mSuperstructure.setSkipConfigChecks(false);
            mEnabledLooper.start();
            mShouldEngagePolarDrive.update(false);
            RobotState.getInstance().setHasBeenEnabled(true);
            if(!hasBeenInAuto) state = Superstructure.GoalState.STOW;
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    private final TimeDelayedBoolean mShouldChangeToMaintain = new TimeDelayedBoolean();
    private final ToggleBoolean mShouldLeaveHeadingController = new ToggleBoolean();
    private final LatchedBoolean mShouldEngagePolarDrive = new LatchedBoolean();


    private final LatchedBoolean mShouldCenterAutoAlign = new LatchedBoolean();
    private final LatchedBoolean mShouldLeftAutoAlign = new LatchedBoolean();
    private final LatchedBoolean mShouldRightAutoAlign = new LatchedBoolean();

    private final LatchedBoolean mShouldAutoAlign = new LatchedBoolean();
    private final TimeDelayedBoolean mWantZeroGyro = new TimeDelayedBoolean();
    private final LatchedBoolean mWantClimbMode = new LatchedBoolean();
    private double mClimbingStartTime = 0.0;
    private boolean mIsLoweringForks = false;
    private boolean mInClimbMode = false;

    private boolean hasRegeneratedTrajectoriesForBlue = false;

    double lastTimestamp = Timer.getFPGATimestamp();
    Superstructure.GoalState state = null;

    @Override
    public void teleopPeriodic() {
        try {
            double timestamp = Timer.getFPGATimestamp();
            lastTimestamp = timestamp;

            if (mWantClimbMode.update(mControlBoard.getToggleClimbMode())) {
                mInClimbMode = !mInClimbMode;
            }

            if (mInClimbMode) {
                mRollerClaw.wantNeutral();
                mGroundIntakeRoller.setDesiredState(GroundIntakeRoller.DesiredState.IDLE);
                mGroundIntakeDeploy.stow();
                state = Superstructure.GoalState.STOW;

                if (!Util.epsilonEquals(mControlBoard.getManualForksJog(), 0)) {
                    mForks.setOpenLoop(mControlBoard.getManualForksJog());
                    mIsLoweringForks = false;
                } else if (mControlBoard.getForksUp()) {
                    mForks.setSetpointPositionPID(Constants.kForksConstants.kMaxUnitsLimit);
                    mIsLoweringForks = false;
                } else if (mControlBoard.getForksDown() && !mIsLoweringForks) {
                    mForks.setSetpointPositionPID(Constants.kForksConstants.kMinUnitsLimit);
                    mIsLoweringForks = true;
                    mClimbingStartTime = Timer.getFPGATimestamp();
                } else if (mIsLoweringForks && Timer.getFPGATimestamp()-mClimbingStartTime > Constants.kForksClimbingTimeout) {
                    mForks.setOpenLoop(0.0);
                    mIsLoweringForks = false;
                }
            } else {
                mForks.setSetpointPositionPID(Constants.kForksConstants.kHomePosition);

                boolean pushDesired = mPushDesired.update(mControlBoard.getGroundIntakePush());

                if (mControlBoard.getScore() && mControlBoard.getFling() && mRollerClaw.hasGamePiece()) {
                    state = Superstructure.GoalState.CLOSE_FLING;
                } else if(mControlBoard.getScore() && mControlBoard.primeHighFling() && mRollerClaw.hasGamePiece()) {
                    state = Superstructure.GoalState.FAR_FlING;
                } else if (mControlBoard.getScore()) {
                    // When auto aligning, score button only activates when we're on target.
                    if (mDrive.isAutoAlignActive()) {
                        if (mDrive.getAutoAlignComplete()) {
                            state = Superstructure.GoalState.SCORE;
                        }
                    } else {
                        state = Superstructure.GoalState.SCORE;
                    }
                } else if (mControlBoard.getScoreHigh()) {
                    state = (mSuperstructure.getGameObjectType() == Superstructure.GameObjectType.CONE
                            ? Superstructure.GoalState.HIGH_SCORING_CONE : Superstructure.GoalState.HIGH_SCORING_CUBE);
                } else if (mControlBoard.getScoreMedium()) {
                    state = (mSuperstructure.getGameObjectType() == Superstructure.GameObjectType.CONE
                            ? Superstructure.GoalState.MID_SCORING_CONE : Superstructure.GoalState.MID_SCORING_CUBE);
                } else if (mControlBoard.getScoreLow()) {
                    state = (mSuperstructure.getGameObjectType() == Superstructure.GameObjectType.CONE
                            ? Superstructure.GoalState.LOW_SCORING_CONE : Superstructure.GoalState.LOW_SCORING_CUBE);
                } else if (mControlBoard.getStow()) {
                    state = Superstructure.GoalState.STOW;
                } else if (mControlBoard.getPickupFeederCone()) {
                    mSuperstructure.setGameObjectType(Superstructure.GameObjectType.CONE);
                    state = Superstructure.GoalState.PICKUP_HIGH_CONE;
                } else if (mControlBoard.getPickupFeederCube()) {
                    mSuperstructure.setGameObjectType(Superstructure.GameObjectType.CUBE);
                    state = Superstructure.GoalState.PICKUP_HIGH_CUBE;
                } else if (mControlBoard.getPickupGroundCone()) {
                    mSuperstructure.setGameObjectType(Superstructure.GameObjectType.CONE);
                    state = Superstructure.GoalState.PICKUP_LOW_CONE;
                } else if (mControlBoard.getPickupGroundCube()) {
                    mSuperstructure.setGameObjectType(Superstructure.GameObjectType.CUBE);
                    state = Superstructure.GoalState.PICKUP_LOW_CUBE;
                } else if (mControlBoard.getStow()) {
                    state = Superstructure.GoalState.STOW;
                } else if (pushDesired) {
                    var goal = mSuperstructure.getGoalState();
                    if (goal == null || goal == Superstructure.GoalState.PUSHING) {
                        state = Superstructure.GoalState.STOW;
                    } else {
                        state = Superstructure.GoalState.PUSHING;
                    }
                }
            }

            if (state != null) mSuperstructure.setGoalState(state);
            state = null;

            if (mControlBoard.getOpenClaw()) {
                mRollerClaw.wantOpen(mSuperstructure.getGameObjectType() == Superstructure.GameObjectType.CONE);
            } else if (mControlBoard.getCloseClaw()) {
                if (mSuperstructure.getGameObjectType() == Superstructure.GameObjectType.CONE) {
                    mRollerClaw.wantConeIntake();
                } else {
                    mRollerClaw.wantCubeIntake();
                }
            }

            double rot = mControlBoard.getRotation();
            rot = Math.signum(rot) * rot * rot;

            if (mControlBoard.getAutoAlign()) {
                Optional<Pose2d> targetPoint = AutoAlignPointSelector.chooseTargetPoint(RobotState.getInstance().getFieldToVehicleAbsolute(timestamp),
                        mSuperstructure.getGameObjectType() == Superstructure.GameObjectType.CONE ? AutoAlignPointSelector.RequestedAlignment.AUTO_CONE : AutoAlignPointSelector.RequestedAlignment.AUTO_CUBE);
                if (targetPoint.isPresent()) {
                    mDrive.setSnapToPoint(targetPoint.get());
                }
            } else {
                mSuperstructure.setScoringOffset(0.0);
                boolean drive_turning = !Util.epsilonEquals(mControlBoard.getRotation(), 0);
                boolean robot_translating = RobotState.getInstance().getMeasuredVelocity().norm() >= 0.1;
                if(mShouldChangeToMaintain.update(!drive_turning && robot_translating, 0.2)) {
                    mSwerveHeadingController.setHeadingControllerState(HeadingControllerState.MAINTAIN);
                    mHeadingGoal.ifPresent(mSwerveHeadingController::setGoal);
                } else {
                    mSwerveHeadingController.setHeadingControllerState(HeadingControllerState.OFF);
                    mHeadingGoal = Optional.of(mDrive.getFieldRelativeGyroscopeRotation().getDegrees());
                }

                rot = mSwerveHeadingController.getHeadingControllerState() == HeadingControllerState.OFF
                        ? rot : mSwerveHeadingController.update(mDrive.getFieldRelativeGyroscopeRotation().getDegrees());

                mDrive.setVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                        mControlBoard.getThrottle() * Constants.kMaxVelocityMetersPerSecond * kScaleTranslationInputs,
                        mControlBoard.getStrafe() * Constants.kMaxVelocityMetersPerSecond * kScaleTranslationInputs,
                        rot * Constants.kMaxAngularVelocityRadiansPerSecond * kScaleRotationInputs,
                        mDrive.getFieldRelativeGyroscopeRotation()));
            }

            if (mWantZeroGyro.update(mControlBoard.resetGyro(), 1)) {
                mHeadingGoal = Optional.of(0.0);
                mDrive.resetAzimuth();
                mDrive.zeroGyroscope();
                RobotState.getInstance().reset();
            }

            // write values to shuffleboard
            outputTelemetry(false);
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledInit() {
        CrashTracker.logDisabledInit();
        try {mSubsystemManager.stop();
            mEnabledLooper.stop();
            mDisabledLooper.start();
            // Reset all auto mode states.
            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }
            mAutoModeSelector.reset();
            mAutoModeSelector.updateModeCreator();
            RobotState.getInstance().resetKalmanFilters();
            mLimelight.start();
            mLimelight.setDisableProcessing(false);
            mDrive.printError();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledPeriodic() {
        try {
            if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
                if (!hasRegeneratedTrajectoriesForBlue) {
                    mDrive.getTrajectoryGenerator().forceRegenerateTrajectories();
                    mLimelight.setBlueTagMap();
                    SmartDashboard.putNumber("Regenerating Trajectories At", Timer.getFPGATimestamp());
                    hasRegeneratedTrajectoriesForBlue = true;
                }
            }

            if (mIter % 50 == 0) {
                if (!RobotState.getInstance().getHasBeenEnabled()) {
                    mDrive.resetAzimuth();
                }
            }
            mIter++;
            mAutoModeSelector.outputToSmartDashboard();
            mAutoModeSelector.updateModeCreator();

            if (mGroundIntakeDeploy.atHomingLocation()) {
                mGroundIntakeDeploy.zeroSensors();
            }

            Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode();
            if (autoMode.isPresent() && autoMode.get() != mAutoModeExecutor.getAutoMode()) {
                System.out.println("Set auto mode to: " + autoMode.get().getClass().toString());
                mAutoModeExecutor.setAutoMode(autoMode.get());
            }

            if (mPowerDistribution.getVoltage() < 12.3) {
                mLED.setWantedAction(WantedAction.DISPLAY_BATTERY_LOW);
            } else if (!Util.epsilonEquals(mSuperstructure.getState().getElevatorHeight(), Constants.kElevatorConstants.kHomePosition, 2)
                    || !Util.epsilonEquals(mSuperstructure.getState().getLateratorExtension(), Constants.kLateratorConstants.kHomePosition, 2)
                    || !Util.epsilonEquals(mGroundIntakeDeploy.getPosition(), Constants.kGroundIntakeDeployConstants.kHomePosition, 2)) {
                mLED.setWantedAction(WantedAction.DISPLAY_NOT_HOMED);
            } else if (!mLimelight.hasTarget()) {
                mLED.setWantedAction(WantedAction.DISPLAY_VISION);
            } else {
                mLED.setWantedAction(WantedAction.DISPLAY_GOOD_BATTERY);
            }

            // write shuffleboard data
            outputTelemetry(true);
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testInit() {
        try {
            CrashTracker.logTestInit();
            System.out.println("Starting check systems");

            mDisabledLooper.stop();
            mEnabledLooper.start();

            if (mDrive.checkSystem()) {
                System.out.println("ALL SYSTEMS PASSED");
            } else {
                System.out.println("CHECK ABOVE OUTPUT SOME SYSTEMS FAILED!!!");
            }
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testPeriodic() {
        try {
            outputTelemetry(false);
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }
}
