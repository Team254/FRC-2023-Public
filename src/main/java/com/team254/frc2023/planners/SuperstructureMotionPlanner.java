package com.team254.frc2023.planners;

import com.team254.frc2023.Constants;
import com.team254.frc2023.statemachines.SuperstructureState;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

import java.util.LinkedList;
import java.util.Optional;

public class SuperstructureMotionPlanner {
    private static final double kConeScoringLowerAmount = 8.0; // Inches down
    private static final double kFeederIntakingRaisingAmount = 5.0; // Inches up
    private static final double kGroundConeIntakingRaisingAmount = 3.0; // Inches up
    private static final double kScoringWaitTime = 0.1; // Seconds

    protected LinkedList<SuperstructureState> mIntermediateStateQueue = new LinkedList<>();
    private Optional<SuperstructureState> mCurrentCommandedState = Optional.empty();
    private Optional<SuperstructureState> mDesiredState = Optional.empty();
    private double mStartedWaitingTimestamp = 0.0;
    private double mScoringOffset = 0.0;

    public synchronized void setScoringOffset(double x) {
        mScoringOffset = x;
    }

    public synchronized void setDesiredState(SuperstructureState desiredState, SuperstructureState currentState) {
        Optional<SuperstructureState> mLastDesiredState = mDesiredState;
        mDesiredState = Optional.of(new SuperstructureState(desiredState));

        if (mDesiredState.get().action == SuperstructureState.Action.SCORING_WITH_LOWER) {
            mDesiredState.get().extension = mDesiredState.get().extension + mScoringOffset;
        }

        // Everything beyond this is probably do-able; clear queue
        mIntermediateStateQueue.clear();

        boolean isConservative = mDesiredState.get().action == SuperstructureState.Action.INTAKING_FROM_FEEDER
                || mDesiredState.get().action == SuperstructureState.Action.INTAKING_GROUND_CUBE
                || mDesiredState.get().action == SuperstructureState.Action.INTAKING_GROUND_CONE
                || mDesiredState.get().action == SuperstructureState.Action.FLINGING
                || mDesiredState.get().action == SuperstructureState.Action.SCORING_WITH_LOWER;

        double currentHeight = currentState.height;
        double elevatorAllowableError = (isConservative ? Constants.kConservativeElevatorAllowableError : Constants.kLiberalElevatorAllowableError);
        double lateratorAllowableError = (isConservative ? Constants.kConservativeLateratorAllowableError : Constants.kLiberalLateratorAllowableError);

        if (mDesiredState.get().action == SuperstructureState.Action.SCORING
                && mLastDesiredState.isPresent()
                && (mLastDesiredState.get().action == SuperstructureState.Action.SCORING_WITH_LOWER ||
                mLastDesiredState.get().action == SuperstructureState.Action.SCORING_WITH_LOWER_AUTO)) {
            currentHeight -= kConeScoringLowerAmount;
            mIntermediateStateQueue.add(new SuperstructureState(
                    currentHeight,
                    currentState.extension,
                    Constants.kConservativeElevatorAllowableError,
                    Constants.kConservativeLateratorAllowableError,
                    SuperstructureState.Action.NEUTRAL,
                    mDesiredState.get().config
            ));
            elevatorAllowableError = Constants.kModerateElevatorAllowableError;
            lateratorAllowableError = Constants.kModerateLateratorAllowableError;
        } else if (mDesiredState.get().action == SuperstructureState.Action.SCORING
                && mLastDesiredState.isPresent()
                && mLastDesiredState.get().action == SuperstructureState.Action.SCORING_WITHOUT_LOWER) {
            mIntermediateStateQueue.add(new SuperstructureState(
                    currentHeight,
                    currentState.extension,
                    Constants.kConservativeElevatorAllowableError,
                    Constants.kConservativeLateratorAllowableError,
                    SuperstructureState.Action.WAIT_WHILE_SCORING,
                    mDesiredState.get().config
            ));

            elevatorAllowableError = Constants.kModerateElevatorAllowableError;
            lateratorAllowableError = Constants.kModerateLateratorAllowableError;
        } else if (mLastDesiredState.isPresent() && mLastDesiredState.get().action == SuperstructureState.Action.INTAKING_FROM_FEEDER) {
            currentHeight += kFeederIntakingRaisingAmount;

            mIntermediateStateQueue.add(new SuperstructureState(
                    currentHeight,
                    currentState.extension,
                    Constants.kConservativeElevatorAllowableError,
                    Constants.kConservativeLateratorAllowableError,
                    SuperstructureState.Action.INTAKING_FROM_FEEDER,
                    mDesiredState.get().config
            ));

            mIntermediateStateQueue.add(new SuperstructureState(
                    currentHeight,
                    desiredState.extension,
                    Constants.kConservativeElevatorAllowableError,
                    Constants.kConservativeLateratorAllowableError,
                    SuperstructureState.Action.INTAKING_FROM_FEEDER,
                    mDesiredState.get().config
            ));
        } else if (mLastDesiredState.isPresent() && mLastDesiredState.get().action == SuperstructureState.Action.INTAKING_GROUND_CONE) {
            currentHeight = desiredState.height;

            mIntermediateStateQueue.add(new SuperstructureState(
                    currentHeight,
                    currentState.extension,
                    Constants.kConservativeElevatorAllowableError,
                    Constants.kConservativeLateratorAllowableError,
                    SuperstructureState.Action.INTAKING_GROUND_CONE,
                    mDesiredState.get().config
            ));
        }

        if (mDesiredState.get().action == SuperstructureState.Action.FLINGING) {
            // Do nothing, go straight to flinging state
        } else if (mDesiredState.get().height < currentHeight) {
            // Laterate first
            mIntermediateStateQueue.add(new SuperstructureState(currentHeight, desiredState.extension, elevatorAllowableError, lateratorAllowableError, mDesiredState.get().action, mDesiredState.get().config));
        } else {
            // Elevate first
            mIntermediateStateQueue.add(new SuperstructureState(desiredState.height, currentState.extension, elevatorAllowableError, lateratorAllowableError, mDesiredState.get().action, mDesiredState.get().config));
        }

        // Reset current command to start executing on next iteration
//        SmartDashboard.putNumber("Last Time Superstructure Regenerated Plan", Timer.getFPGATimestamp());
        mCurrentCommandedState = Optional.empty();
    }

    public void reset() {
        mIntermediateStateQueue.clear();
        mCurrentCommandedState = Optional.empty();
    }

    public boolean isFinished() {
        return mCurrentCommandedState.isEmpty() && mIntermediateStateQueue.isEmpty();
    }

    public SuperstructureState update(SuperstructureState currentState) {

        if (mCurrentCommandedState.isEmpty() && !mIntermediateStateQueue.isEmpty()) {
            mCurrentCommandedState = Optional.of(mIntermediateStateQueue.remove());
            if (mCurrentCommandedState.get().action == SuperstructureState.Action.WAIT_WHILE_SCORING) {
                mStartedWaitingTimestamp = Timer.getFPGATimestamp();
            }
        }

        if (mCurrentCommandedState.isPresent()
                && mCurrentCommandedState.get().action == SuperstructureState.Action.WAIT_WHILE_SCORING
                && Timer.getFPGATimestamp() - mStartedWaitingTimestamp < kScoringWaitTime) {
            // Do nothing while waiting
        } else if (mCurrentCommandedState.isPresent() && mCurrentCommandedState.get().isInRange(currentState)) {
            mCurrentCommandedState = Optional.empty(); // Commanded state will be updated at beginning of next update call
        }

        return mCurrentCommandedState.orElse(mDesiredState.orElse(currentState));
    }

    public synchronized void outputTelemetry() {
        SmartDashboard.putString("Superstructure Commanded State", mCurrentCommandedState.isEmpty() ? "Nothing" : mCurrentCommandedState.get().toString());
        SmartDashboard.putString("Superstructure State Queue", mIntermediateStateQueue.toString());
    }
}
