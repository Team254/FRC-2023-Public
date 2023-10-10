package com.team254.frc2023.subsystems;

import com.team254.frc2023.Constants;
import com.team254.frc2023.RobotState;
import com.team254.frc2023.planners.AutoAlignPointSelector;
import com.team254.frc2023.led.LEDState;
import com.team254.frc2023.statemachines.SuperstructureState;
import com.team254.frc2023.statemachines.SuperstructureStateMachine;
import com.team254.frc2023.led.TimedLEDState;
import com.team254.lib.drivers.Subsystem;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.loops.ILooper;
import com.team254.lib.loops.Loop;
import com.team254.lib.util.LatchedBoolean;
import com.team254.lib.util.TimeDelayedBoolean;
import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;


public class Superstructure extends Subsystem {

    public enum GameObjectType {
        CONE, CUBE
    }

    public static final double kElevatorHighConeScoringPosition = 39.75; // Our field is 41.0
    public static final double kElevatorHighCubeScoringPosition = 32.0;
    public static final double kElevatorMidConeScoringPosition = 31.5; // Our field is 31.5
    public static final double kElevatorMidCubeScoringPosition = 25.0; // Our field is 25.0
    public static final double kElevatorLowConeScoringPosition = 10.6;
    public static final double kElevatorLowCubeScoringPosition = 12.0;
    public static final double kElevatorFeederConePickupPosition = 35.15; // Our field is 36.4
    public static final double kElevatorFeederCubePickupPosition = 35.0;
    public static final double kElevatorGroundConePickupPosition = 0.0;
    public static final double kElevatorGroundCubePickupPosition = 1.0;
    public static final double kElevatorDribblePosition = 9.0;
    public static final double kElevatorUnstowPosition = 30.0;
    public static final double kElevatorStowPosition = 11.1; //10.6
    public static final double kLateratorHighConeScoringPosition = 48.5; // Our field is 48.5
    public static final double kLateratorHighCubeScoringPosition = 43.8;
    public static final double kLateratorMidConeScoringPosition = 30.5; // Our field is 29.5
    public static final double kLateratorMidCubeScoringPosition = 25.4; // Our field is 25.4
    public static final double kLateratorLowConeScoringPosition = 0.0;
    public static final double kLateratorLowCubeScoringPosition = 4.6;
    public static final double kLateratorFeederConePickupPosition = 16.0;
    public static final double kLateratorFeederCubePickupPosition = 13.0;
    public static final double kLateratorGroundConePickupPosition = 11.0;
    public static final double kLateratorGroundCubePickupPosition = 11.0;
    public static final double kLateratorDribblePosition = 11.3;
    public static final double kLateratorUnstowPosition = 0.0;
    public static final double kLateratorStowPositon = 0;
    public static final double kLateratorConeBumpPositon = 0;
    public static final double kLateratorFlingingPosition = 40.0;
    public static final double kElevatorConeBumpPosition = 5.0;
    public static final double kElevatorFlingingPosition = 31.0;

    public static final double kLEDClosenessDeadbandMeters = 0.03;

    private final Elevator mElevator = Elevator.getInstance();
    private final Laterator mLaterator = Laterator.getInstance();
    private final GroundIntakeRoller mGroundIntakeRoller = GroundIntakeRoller.getInstance();
    private final GroundIntakeDeploy mGroundIntakeDeploy = GroundIntakeDeploy.getInstance();

    private static Superstructure mSuperstructure;
    private final RollerClaw mRollerClaw = RollerClaw.getInstance();
    private final SuperstructureStateMachine mSuperstructureStateMachine = new SuperstructureStateMachine();

    private final LED mLEDs = LED.getInstance();

    private SuperstructureStateMachine.WantedAction mWantedAction = SuperstructureStateMachine.WantedAction.IDLE;

    private SuperstructureState mMeasuredState = new SuperstructureState();
    private SuperstructureState mExpectedState = new SuperstructureState();
    private SuperstructureState mCommandedState;
    private GoalState mGoalState;
    private GoalState mLastGoalState;
    private final LatchedBoolean mShouldOpenIntake = new LatchedBoolean();
    private final LatchedBoolean mShouldAutoStow = new LatchedBoolean();
    private double mIntakenGamePieceTime = 0.0;
    private boolean mLateratorInSpringyMode = false;
    private boolean mLateratorInFlingyMode = false;
    private boolean mGroundIntakeDeployInSpringyMode = false;
    private Optional<SuperstructureState.SuperstructureConfig> mCurrentSuperstructureConfig = Optional.empty();
    public static GameObjectType mGameObjectType = GameObjectType.CONE;
    private boolean mHasBeenHomed = false;
    public boolean mSkipConfigChecks = false;
    private TimeDelayedBoolean mNotHasGamePiece = new TimeDelayedBoolean();
    private boolean mHasConeBumped = false;

    public static synchronized Superstructure getInstance() {
        if(mSuperstructure == null) {
            mSuperstructure = new Superstructure();
        }
        return mSuperstructure;
    }

    public enum GoalState {
        PICKUP_HIGH_CONE(new SuperstructureState(kElevatorFeederConePickupPosition, kLateratorFeederConePickupPosition, Constants.kConservativeElevatorAllowableError, Constants.kConservativeLateratorAllowableError, SuperstructureState.Action.INTAKING_FROM_FEEDER, SuperstructureState.SuperstructureConfig.MEDIUM)),
        PICKUP_HIGH_CUBE(new SuperstructureState(kElevatorFeederCubePickupPosition, kLateratorFeederCubePickupPosition, Constants.kConservativeElevatorAllowableError, Constants.kConservativeLateratorAllowableError, SuperstructureState.Action.INTAKING_FROM_FEEDER, SuperstructureState.SuperstructureConfig.MEDIUM)),
        PICKUP_LOW_CONE(new SuperstructureState(kElevatorGroundConePickupPosition, kLateratorGroundConePickupPosition, Constants.kConservativeElevatorAllowableError, Constants.kConservativeLateratorAllowableError, SuperstructureState.Action.INTAKING_GROUND_CONE, SuperstructureState.SuperstructureConfig.MEDIUM)),
        PICKUP_LOW_CUBE(new SuperstructureState(kElevatorGroundCubePickupPosition, kLateratorGroundCubePickupPosition, Constants.kConservativeElevatorAllowableError, Constants.kConservativeLateratorAllowableError, SuperstructureState.Action.INTAKING_GROUND_CUBE, SuperstructureState.SuperstructureConfig.MEDIUM)),
        UNSTOW(new SuperstructureState(kElevatorUnstowPosition, kLateratorUnstowPosition, Constants.kConservativeElevatorAllowableError, Constants.kConservativeLateratorAllowableError, SuperstructureState.Action.NEUTRAL, SuperstructureState.SuperstructureConfig.MEDIUM)),
        STOW(new SuperstructureState(kElevatorStowPosition, kLateratorStowPositon, SuperstructureState.Action.NEUTRAL)),
        CONE_BUMP(new SuperstructureState(kElevatorConeBumpPosition, kLateratorConeBumpPositon, SuperstructureState.Action.NEUTRAL)),
        LOW_SCORING_CONE(new SuperstructureState(kElevatorLowConeScoringPosition, kLateratorLowConeScoringPosition, SuperstructureState.Action.SCORING_WITHOUT_LOWER, SuperstructureState.SuperstructureConfig.MEDIUM)),
        LOW_SCORING_CUBE(new SuperstructureState(kElevatorLowCubeScoringPosition, kLateratorLowCubeScoringPosition, SuperstructureState.Action.SCORING_WITHOUT_LOWER, SuperstructureState.SuperstructureConfig.MEDIUM)),
        MID_SCORING_CONE(new SuperstructureState(kElevatorMidConeScoringPosition, kLateratorMidConeScoringPosition, SuperstructureState.Action.SCORING_WITH_LOWER, SuperstructureState.SuperstructureConfig.MEDIUM)),

        MID_SCORING_CUBE(new SuperstructureState(kElevatorMidCubeScoringPosition, kLateratorMidCubeScoringPosition, SuperstructureState.Action.SCORING_WITHOUT_LOWER, SuperstructureState.SuperstructureConfig.MEDIUM)),
        PRE_MID_SCORING_CUBE(new SuperstructureState(kElevatorMidCubeScoringPosition, kLateratorStowPositon, SuperstructureState.Action.NEUTRAL)),

        HIGH_SCORING_CONE(new SuperstructureState(kElevatorHighConeScoringPosition, kLateratorHighConeScoringPosition, Constants.kConservativeElevatorAllowableError, Constants.kConservativeLateratorAllowableError, SuperstructureState.Action.SCORING_WITH_LOWER, SuperstructureState.SuperstructureConfig.MEDIUM)),
        HIGH_SCORING_CONE_AUTO(new SuperstructureState(kElevatorHighConeScoringPosition, kLateratorHighConeScoringPosition, SuperstructureState.Action.SCORING_WITH_LOWER_AUTO, SuperstructureState.SuperstructureConfig.MEDIUM)),
        PRE_HIGH_SCORING_CONE(new SuperstructureState(kElevatorHighConeScoringPosition, kLateratorStowPositon, SuperstructureState.Action.NEUTRAL)),

        HIGH_SCORING_CUBE(new SuperstructureState(kElevatorHighCubeScoringPosition, kLateratorHighCubeScoringPosition, SuperstructureState.Action.SCORING_WITHOUT_LOWER, SuperstructureState.SuperstructureConfig.MEDIUM)),
        PRE_HIGH_SCORING_CUBE(new SuperstructureState(kElevatorHighCubeScoringPosition, kLateratorStowPositon, SuperstructureState.Action.NEUTRAL)),

        SCORE(new SuperstructureState(kElevatorStowPosition, kLateratorStowPositon, Constants.kModerateElevatorAllowableError, Constants.kModerateLateratorAllowableError, SuperstructureState.Action.SCORING, SuperstructureState.SuperstructureConfig.MEDIUM)),
        DROP(new SuperstructureState(kElevatorMidCubeScoringPosition, kLateratorMidCubeScoringPosition, Constants.kConservativeElevatorAllowableError, Constants.kConservativeLateratorAllowableError, SuperstructureState.Action.DROPPING, SuperstructureState.SuperstructureConfig.HIGH)),
        CLOSE_FLING(new SuperstructureState(kElevatorFlingingPosition, kLateratorFlingingPosition, Constants.kConservativeElevatorAllowableError, Constants.kConservativeLateratorAllowableError, SuperstructureState.Action.FLINGING, SuperstructureState.SuperstructureConfig.HIGH)),
        FAR_FlING(new SuperstructureState(kElevatorFlingingPosition, kLateratorFlingingPosition, Constants.kConservativeElevatorAllowableError, Constants.kConservativeLateratorAllowableError, SuperstructureState.Action.FLINGING, SuperstructureState.SuperstructureConfig.HIGH)),
        PUSHING(new SuperstructureState(kElevatorStowPosition, kLateratorStowPositon, SuperstructureState.Action.DRIBBLING));

        public SuperstructureState state;

        GoalState(SuperstructureState superstructureGoal) {
            this.state = superstructureGoal;
        }
    }

    public synchronized void setScoringOffset(double x) {
        mSuperstructureStateMachine.setScoringOffset(x);
    }

    public synchronized GoalState getGoalState() {
        return mGoalState;
    }

    public synchronized void setGoalState(GoalState goalState) {
        if (mLastGoalState == null) mLastGoalState = goalState;

        // Exit if switching between ground intake states
        if (goalState == GoalState.PICKUP_LOW_CUBE && mGoalState == GoalState.PICKUP_LOW_CONE) {
            return;
        } else if (goalState == GoalState.PICKUP_LOW_CONE && mGoalState == GoalState.PICKUP_LOW_CUBE) {
            return;
        } else if (goalState == GoalState.PICKUP_HIGH_CONE && mGoalState == GoalState.PICKUP_LOW_CUBE) {
            return;
        } else if (goalState == GoalState.PICKUP_HIGH_CUBE && mGoalState == GoalState.PICKUP_LOW_CUBE) {
            return;
        }

        if (goalState == GoalState.PICKUP_HIGH_CONE || goalState == GoalState.PICKUP_LOW_CONE) {
            setGameObjectType(GameObjectType.CONE);
            if (mRollerClaw.hasGamePiece()) return;
        } else if (goalState.state.action != SuperstructureState.Action.DRIBBLING
                && (goalState == GoalState.PICKUP_HIGH_CUBE || goalState ==  GoalState.PICKUP_LOW_CUBE)) {
            setGameObjectType(GameObjectType.CUBE);
            if (mRollerClaw.hasGamePiece()) return;
        }

        if (mGoalState != goalState) {
            if (goalState == GoalState.STOW && mNotHasGamePiece.update(!mRollerClaw.hasGamePiece(), 0.75)) {
                mRollerClaw.wantNeutral();
            } else if (mGoalState == GoalState.CLOSE_FLING && goalState == GoalState.SCORE) {
                return;
            }

            mLastGoalState = mGoalState;
            mGoalState = goalState;
            mWantedAction = SuperstructureStateMachine.WantedAction.GO_TO_POSITION;

            // Logic for dynamically configuring motion magic configs
            SuperstructureState.SuperstructureConfig desiredConfigs = mGoalState.state.config;
            if (desiredConfigs == SuperstructureState.SuperstructureConfig.LOW
                    && (mCurrentSuperstructureConfig.isEmpty() || mCurrentSuperstructureConfig.get() != SuperstructureState.SuperstructureConfig.LOW)) {
                mElevator.setMotionMagicConfigsUnchecked(Constants.kElevatorSlowAcceleration, Constants.kElevatorSlowJerk);
                mLaterator.setMotionMagicConfigsUnchecked(Constants.kLateratorSlowAcceleration, Constants.kLateratorSlowJerk);

            } else if (desiredConfigs == SuperstructureState.SuperstructureConfig.MEDIUM
                    && (mCurrentSuperstructureConfig.isEmpty() || mCurrentSuperstructureConfig.get() != SuperstructureState.SuperstructureConfig.MEDIUM)) {
                mElevator.setMotionMagicConfigsUnchecked(Constants.kElevatorMediumAcceleration, Constants.kElevatorMediumJerk);
                mLaterator.setMotionMagicConfigsUnchecked(
                        (DriverStation.isAutonomous()) ? Constants.kLateratorMediumAuto : Constants.kLateratorMediumAcceleration,
                        Constants.kLateratorMediumJerk);

            } else if (desiredConfigs == SuperstructureState.SuperstructureConfig.HIGH
                    && (mCurrentSuperstructureConfig.isEmpty() || mCurrentSuperstructureConfig.get() != SuperstructureState.SuperstructureConfig.HIGH)) {
                mElevator.setMotionMagicConfigsUnchecked(Constants.kElevatorFastAcceleration, Constants.kElevatorFastJerk);
                mLaterator.setMotionMagicConfigsUnchecked(Constants.kLateratorFastAcceleration, Constants.kLateratorFastJerk);
            }
            mCurrentSuperstructureConfig = Optional.of(desiredConfigs);
        }
    }

    public synchronized SuperstructureState getState() {
        if (mExpectedState.isInRange(mMeasuredState, Constants.kExpectedVsMeasuredElevatorAllowableError, Constants.kExpectedVsMeasuredLateratorAllowableError)) {
            return mExpectedState;
        } else {
            return mMeasuredState;
        }
    }

    public synchronized Translation2d getSuperstructureVelocity() {
        return new Translation2d(mLaterator.getVelocity() * Math.cos(Constants.kLateratorAngle), mElevator.getVelocity());
    }

    public synchronized void setHasBeenHomed(boolean homed) {
        mHasBeenHomed = homed;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mMeasuredState = new SuperstructureState(mElevator.getPosition(), mLaterator.getPosition());
        mExpectedState = new SuperstructureState(mElevator.getActiveTrajectoryPosition(), mLaterator.getActiveTrajectoryPosition(), SuperstructureState.Action.NEUTRAL);

        if (!mHasBeenHomed) {
            if (getState().isInRange(GoalState.UNSTOW.state)) {
                mHasBeenHomed = true;
            }
        } else {
            if (mGoalState != null) mSuperstructureStateMachine.setScoringPosition(mGoalState.state);
            mCommandedState = mSuperstructureStateMachine.update(
                    Timer.getFPGATimestamp(),
                    mWantedAction,
                    getState()
            );
        }
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (!mHasBeenHomed) {
            mElevator.setSetpointMotionMagic(GoalState.UNSTOW.state.height);
            mLaterator.setSetpointMotionMagic(GoalState.UNSTOW.state.extension);
        } else if (mCommandedState != null) {
            mElevator.setSetpointMotionMagic(mCommandedState.getElevatorHeight());
            mLaterator.setSetpointMotionMagic(mCommandedState.getLateratorExtension());
        }
    }

    @Override
    public synchronized void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {

            }

            @Override
            public void onLoop(double timestamp) {
                Optional<TimedLEDState> ledState = handleLEDs(timestamp);
                if (ledState.isPresent()) {
                    if (DriverStation.isAutonomous()) {
                        mLEDs.setWantedAction(LED.WantedAction.DISPLAY_VISION);
                    } else {
                        mLEDs.setSuperstructureLEDState(ledState.get());
                        mLEDs.setWantedAction(LED.WantedAction.DISPLAY_SUPERSTRUCTURE);
                    }
                } else {
                    mLEDs.setWantedAction(LED.WantedAction.OFF);
                }

                if (mGoalState == null) return;

                if (getState().isInRange(
                        GoalState.STOW.state,
                        Constants.kConservativeElevatorAllowableError,
                        Constants.kConservativeLateratorAllowableError)
                        && mGoalState != GoalState.SCORE
                        && mRollerClaw.hasGamePiece()
                        && mGameObjectType == GameObjectType.CONE
                        && !mHasConeBumped) {
                    setGoalState(GoalState.CONE_BUMP);
                    mHasConeBumped = true;
                } else if (getState().isInRange(
                        GoalState.CONE_BUMP.state,
                        Constants.kConservativeElevatorAllowableError,
                        Constants.kConservativeLateratorAllowableError)
                        && mHasConeBumped) {
                    setGoalState(GoalState.STOW);
                } else if (!mRollerClaw.hasGamePiece()) {
                    mHasConeBumped = false;
                }

                // Logic for auto-stowing
                if (getState().isInRange(
                        GoalState.STOW.state,
                        Constants.kConservativeElevatorAllowableError,
                        Constants.kConservativeLateratorAllowableError)
                        && mGoalState == GoalState.SCORE
                        && !mRollerClaw.hasGamePiece()) {
                    setGoalState(GoalState.STOW);
                } else if (getState().isInRange(
                        GoalState.CLOSE_FLING.state,
                        Constants.kConservativeElevatorAllowableError,
                        Constants.kConservativeLateratorAllowableError)
                        && !mRollerClaw.hasGamePiece()) {
                    setGoalState(GoalState.STOW);
                } else if (getState().isInRange(
                        GoalState.FAR_FlING.state,
                        Constants.kConservativeElevatorAllowableError,
                        Constants.kConservativeLateratorAllowableError)
                        && !mRollerClaw.hasGamePiece()) {
                    setGoalState(GoalState.STOW);
                }

                boolean inIntakingState = mGoalState == GoalState.PICKUP_HIGH_CONE
                        || mGoalState == GoalState.PICKUP_HIGH_CUBE
                        || mGoalState == GoalState.PICKUP_LOW_CONE
                        || mGoalState == GoalState.PICKUP_LOW_CUBE;
                if (inIntakingState && mShouldAutoStow.update(mRollerClaw.hasGamePiece())) {
                    mIntakenGamePieceTime = Timer.getFPGATimestamp();
                } else if (inIntakingState && mRollerClaw.hasGamePiece()
                        && Timer.getFPGATimestamp()-mIntakenGamePieceTime > Constants.kAutoStowWaitTime) {
                    setGoalState(GoalState.STOW);
                }

                // Springy/Flingy laterator logic
                if (!mLateratorInSpringyMode &&
                        mGoalState.state.isInRange(getState(), Constants.kConservativeElevatorAllowableError, Constants.kConservativeLateratorAllowableError)
                        && mGoalState.state.action == SuperstructureState.Action.INTAKING_FROM_FEEDER) {
                    mLaterator.setStatorCurrentLimit(Constants.kSpringyLateratorStatorCurrentLimit, true);
                    mLaterator.setSupplyCurrentLimit(Constants.kSpringyLateratorSupplyCurrentLimit, true);
                    mLateratorInSpringyMode = true;
                    mLateratorInFlingyMode = false;
                    SmartDashboard.putBoolean("In Flingy Mode", mLateratorInFlingyMode);
                } else if (!mLateratorInFlingyMode && (mGoalState == GoalState.CLOSE_FLING || mGoalState == GoalState.FAR_FlING)) {
                    mLaterator.setStatorCurrentLimit(Constants.kLateratorConstants.kStatorCurrentLimit, Constants.kLateratorConstants.kEnableStatorCurrentLimit);
                    mLaterator.setSupplyCurrentLimit(Constants.kLateratorConstants.kSupplyCurrentLimit, false);
                    mLateratorInSpringyMode = false;
                    mLateratorInFlingyMode = true;
                } else if ((mLateratorInSpringyMode && mGoalState.state.action != SuperstructureState.Action.INTAKING_FROM_FEEDER)
                        || (mLateratorInFlingyMode && mGoalState != GoalState.CLOSE_FLING && mGoalState != GoalState.FAR_FlING)) {
                    mLaterator.setStatorCurrentLimit(Constants.kLateratorConstants.kStatorCurrentLimit, Constants.kLateratorConstants.kEnableStatorCurrentLimit);
                    mLaterator.setSupplyCurrentLimit(Constants.kLateratorConstants.kSupplyCurrentLimit, Constants.kLateratorConstants.kEnableSupplyCurrentLimit);
                    mLateratorInSpringyMode = false;
                    mLateratorInFlingyMode = false;
                }
                SmartDashboard.putBoolean("In Flingy Mode", mLateratorInFlingyMode);
                // Logic to deal with stowing and deploying ground intake
                if (mGoalState == GoalState.PUSHING) {
                    mGroundIntakeDeploy.push();
                    mGroundIntakeRoller.setDesiredState(GroundIntakeRoller.DesiredState.PUSH);
                } else if (mGoalState == GoalState.PICKUP_LOW_CUBE) {
                    mGroundIntakeDeploy.extend();
                    mGroundIntakeRoller.setDesiredState(GroundIntakeRoller.DesiredState.INTAKE);
                    if (!mRollerClaw.hasGamePiece() && getState().isInRange(
                                GoalState.PICKUP_LOW_CUBE.state,
                                Constants.kConservativeElevatorAllowableError,
                                Constants.kConservativeLateratorAllowableError)) {
                        mGroundIntakeRoller.setDesiredState(GroundIntakeRoller.DesiredState.INTAKE);
                    } else if (!mRollerClaw.hasGamePiece() && !getState().isInRange(
                                GoalState.PICKUP_LOW_CUBE.state,
                                Constants.kConservativeElevatorAllowableError,
                                Constants.kConservativeLateratorAllowableError)) {
                        mGroundIntakeRoller.setDesiredState(GroundIntakeRoller.DesiredState.EXHUAST);
                    } else {
                        mGroundIntakeRoller.setDesiredState(GroundIntakeRoller.DesiredState.IDLE);
                    }
                } else {
                    mGroundIntakeRoller.setDesiredState(GroundIntakeRoller.DesiredState.IDLE);
                    if (mElevator.getPosition() > kElevatorStowPosition - 4) {
                        mGroundIntakeDeploy.stow();
                    }
                }
                if (mGoalState != GoalState.PUSHING) {
                    // Springy ground intake logic
                    if (!mGroundIntakeDeployInSpringyMode && Util.epsilonEquals(mGroundIntakeDeploy.getPosition(), Constants.kGroundIntakeDeployExtendPosition, 1)) {
                        if (mSkipConfigChecks) {
                            mGroundIntakeDeploy.setSupplyCurrentLimitUnchecked(Constants.kGroundIntakeDeploySpringySupplyCurrentLimit, true);
                            mGroundIntakeDeploy.setStatorCurrentLimitUnchecked(Constants.kGroundIntakeDeploySpringyStatorCurrentLimit, true);
                        } else {
                            mGroundIntakeDeploy.setSupplyCurrentLimit(Constants.kGroundIntakeDeploySpringySupplyCurrentLimit, true);
                            mGroundIntakeDeploy.setStatorCurrentLimit(Constants.kGroundIntakeDeploySpringyStatorCurrentLimit, true);
                        }
                        mGroundIntakeDeployInSpringyMode = true;
                    } else if (mGroundIntakeDeployInSpringyMode &&
                            (!Util.epsilonEquals(mGroundIntakeDeploy.getPosition(), Constants.kGroundIntakeDeployExtendPosition, Constants.kGroundIntakeDeploySpringyAllowableError)
                                    || mGoalState != GoalState.PICKUP_LOW_CUBE)) {
                        if (mSkipConfigChecks) {
                            mGroundIntakeDeploy.setSupplyCurrentLimitUnchecked(Constants.kGroundIntakeDeployConstants.kSupplyCurrentLimit, Constants.kGroundIntakeDeployConstants.kEnableSupplyCurrentLimit);
                            mGroundIntakeDeploy.setStatorCurrentLimitUnchecked(Constants.kGroundIntakeDeployConstants.kStatorCurrentLimit, Constants.kGroundIntakeDeployConstants.kEnableStatorCurrentLimit);
                        } else {
                            mGroundIntakeDeploy.setSupplyCurrentLimit(Constants.kGroundIntakeDeployConstants.kSupplyCurrentLimit, Constants.kGroundIntakeDeployConstants.kEnableSupplyCurrentLimit);
                            mGroundIntakeDeploy.setStatorCurrentLimit(Constants.kGroundIntakeDeployConstants.kStatorCurrentLimit, Constants.kGroundIntakeDeployConstants.kEnableStatorCurrentLimit);
                        }
                        mGroundIntakeDeployInSpringyMode = false;
                    }
                }

                // Intake logic
                switch (mGoalState.state.action) {
                    case INTAKING_FROM_FEEDER:
                    case INTAKING_GROUND_CONE:
                    case INTAKING_GROUND_CUBE:
                        if (mShouldOpenIntake.update(true)) {
                            if (mGameObjectType == GameObjectType.CONE) {
                                mRollerClaw.wantConeIntake();
                            } else {
                                mRollerClaw.wantCubeIntake();
                            }
                        }
                        break;
                    case SCORING:
                        mShouldOpenIntake.update(false);
                        if (mCommandedState.action == SuperstructureState.Action.SCORING
                                || mCommandedState.action == SuperstructureState.Action.WAIT_WHILE_SCORING) { // Wait until after lower to score
                            SuperstructureState goal = (mGameObjectType == GameObjectType.CONE ? GoalState.LOW_SCORING_CONE.state : GoalState.LOW_SCORING_CUBE.state);
                            mRollerClaw.wantOpen(mGameObjectType == GameObjectType.CONE,
                                    getState().isInRange(
                                            goal,
                                            Constants.kModerateElevatorAllowableError,
                                            Constants.kModerateLateratorAllowableError)
                            ); // Constantly command intake open state to prevent double clutch
                        }
                        break;
                    case FLINGING:
                        mShouldOpenIntake.update(false);
                        if (getState().isInRange(
                                GoalState.CLOSE_FLING.state,
                                Constants.kFlingingElevatorAllowableError,
                                Constants.kFlingingLateratorAllowableError)) {
                            mRollerClaw.wantFling(getGameObjectType()==GameObjectType.CONE, false);
                        } else if (getState().isInRange(
                                GoalState.FAR_FlING.state,
                                Constants.kFlingingElevatorAllowableError,
                                Constants.kFlingingLateratorAllowableError)) {
                            mRollerClaw.wantFling(getGameObjectType()==GameObjectType.CONE, true);
                        }
                        break;
                    case DROPPING:
                        mShouldOpenIntake.update(false);
                        if (getState().isInRange(
                                GoalState.DROP.state,
                                0.5,
                                0.5)) {
                            mRollerClaw.wantOpen(true);
                        }
                        break;
                    case SCORING_WITH_LOWER:
                    case SCORING_WITH_LOWER_AUTO:
                    case SCORING_WITHOUT_LOWER:
                    case NEUTRAL:
                    default:
                        mShouldOpenIntake.update(false);
                }
            }

            @Override
            public void onStop(double timestamp) {

            }
        });
    }

    public synchronized boolean isAtGoal() {
        return getState().isInRange(mGoalState.state, Constants.kConservativeElevatorAllowableError, Constants.kConservativeLateratorAllowableError);
    }

    public void setGameObjectType(GameObjectType type) {
        mGameObjectType = type;
    }

    public synchronized GameObjectType getGameObjectType() {
        return mGameObjectType;
    }

    private synchronized Optional<TimedLEDState> handleLEDs(double timestamp) {
        Optional<TimedLEDState> signal = handleScoringAlignmentLEDs(timestamp);
        if (signal.isEmpty()) {
            signal = handleIntakingLEDs(timestamp);
        }
        return signal;
    }

    private synchronized Optional<TimedLEDState> handleIntakingLEDs(double timestamp) {
        Optional<TimedLEDState> display = Optional.empty();
        boolean hasPiece = false;
        switch (mGameObjectType) {
            case CONE:
                hasPiece = mRollerClaw.doneConeIntaking(timestamp);
                display = Optional.of(hasPiece ? TimedLEDState.StaticLEDState.kHasCone : TimedLEDState.BlinkingLEDState.kConeIntakeWaiting);
                break;
            case CUBE:
                hasPiece = mRollerClaw.doneCubeIntaking(timestamp);
                display = Optional.of(hasPiece ? TimedLEDState.StaticLEDState.kHasCube : TimedLEDState.BlinkingLEDState.kCubeIntakeWaiting);
                break;
        }
        return display;
    }

    private synchronized Optional<TimedLEDState> handleScoringAlignmentLEDs(double timestamp) {
        AutoAlignPointSelector.RequestedAlignment alignmentType = AutoAlignPointSelector.RequestedAlignment.AUTO;
        LEDState pieceColor = LEDState.kAutoAlign;
        double maxError = 0.56 / 2.0; // m (distance between low goals)
        switch (mGameObjectType) {
            case CONE:
                alignmentType = AutoAlignPointSelector.RequestedAlignment.AUTO_CONE;
                pieceColor = LEDState.kIntakingCone;
                maxError = 1.13 / 2.0; // m (distance between max cones)
                break;
            case CUBE:
                alignmentType = AutoAlignPointSelector.RequestedAlignment.AUTO_CUBE;
                pieceColor = LEDState.kIntakingCube;
                maxError = 1.68 / 2.0; // m (distance between tags)
                break;
        }

        if (mGoalState == GoalState.LOW_SCORING_CONE || mGoalState == GoalState.LOW_SCORING_CUBE) {
            // if we aren't low scoring, we need to pick cones or cubes for alignment hinter
            alignmentType = AutoAlignPointSelector.RequestedAlignment.AUTO;
        }
        Pose2d fieldToVehicle = RobotState.getInstance().getFieldToVehicleAbsolute(timestamp);
        Optional<Pose2d> targetPoint = AutoAlignPointSelector.chooseTargetPoint(fieldToVehicle, alignmentType);

        if (targetPoint.isEmpty()) {
            // the target point will be empty if we are too far away from alignment, and we shouldn't hint alignment
            return Optional.empty();
        } else {
            Translation2d error = targetPoint.get().getTranslation().add(fieldToVehicle.getTranslation().unaryMinus());
            double horizError = Math.abs(error.y());
            boolean auto_align_active = Drive.getInstance().isAutoAlignActive();
            boolean auto_align_on_target = Drive.getInstance().getAutoAlignComplete();
            if ((horizError < kLEDClosenessDeadbandMeters && !auto_align_active) || (auto_align_active && auto_align_on_target)) {
                return Optional.of(TimedLEDState.StaticLEDState.kAtAlignment);
            } else {
                if (horizError <= kLEDClosenessDeadbandMeters) {
                    horizError = 0.0;
                }
                double percentage = Util.limit((maxError - horizError) / maxError, 0.0, 1.0);
                return Optional.of(new TimedLEDState.PercentFullLEDState(percentage, pieceColor));
            }
        }
    }

    public void setSkipConfigChecks(boolean skip) {
        mSkipConfigChecks = skip;
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public synchronized void outputTelemetry(boolean disabled) {
        if (true) {
            if (mGoalState != null) {
                SmartDashboard.putString("Superstructure Goal", mGoalState.toString());
                SmartDashboard.putBoolean("Is At Goal", isAtGoal());
            }

            SmartDashboard.putBoolean("Superstructure Homed", mHasBeenHomed);
            if (mCommandedState != null)
                SmartDashboard.putString("Superstructure Commanded State", mCommandedState.toString());
            SmartDashboard.putString("Superstructure Measured State", mMeasuredState.toString());
            SmartDashboard.putString("Superstructure Expected State", mExpectedState.toString());
            SmartDashboard.putString("Superstructure Measured vs Expected Error", new SuperstructureState(
                    mExpectedState.height - mMeasuredState.height,
                    mExpectedState.extension - mMeasuredState.extension,
                    SuperstructureState.Action.NEUTRAL).toString()
            );
            SmartDashboard.putString("Game Object Type", mGameObjectType.toString());
            SmartDashboard.putString("Current Superstructure Configs", mCurrentSuperstructureConfig.toString());
            mSuperstructureStateMachine.outputTelemetry();
        }
    }
}