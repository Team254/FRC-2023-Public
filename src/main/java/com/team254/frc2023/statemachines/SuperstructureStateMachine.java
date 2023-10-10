package com.team254.frc2023.statemachines;

import com.team254.frc2023.planners.SuperstructureMotionPlanner;
import com.team254.lib.util.Util;

public class SuperstructureStateMachine {
    public enum WantedAction {
        IDLE,
        GO_TO_POSITION
    }

    public enum SystemState {
        HOLDING_POSITION,
        MOVING_TO_POSITION
    }

    private SystemState mSystemState = SystemState.HOLDING_POSITION;

    private SuperstructureState mCommandedState = new SuperstructureState();
    private SuperstructureState mLastDesireEndState = new SuperstructureState();
    private SuperstructureState mDesiredEndState = new SuperstructureState();

    private SuperstructureMotionPlanner mPlanner = new SuperstructureMotionPlanner();

    public synchronized void setScoringPosition(SuperstructureState scoringPosition) {
        mDesiredEndState = scoringPosition;
    }

    public synchronized boolean scoringPositionChanged() {
        boolean changed = !Util.epsilonEquals(mDesiredEndState.height, mLastDesireEndState.height)
                || !Util.epsilonEquals(mDesiredEndState.extension, mLastDesireEndState.extension)
                || mDesiredEndState.action != mLastDesireEndState.action;
        return changed;
    }

    public synchronized SystemState getSystemState() {
        return mSystemState;
    }

    public synchronized void setScoringOffset(double x) {mPlanner.setScoringOffset(x);}

    public synchronized SuperstructureState update(double timestamp, WantedAction wantedAction,
                                                     SuperstructureState currentState) {
        synchronized (SuperstructureStateMachine.this) {
            SystemState newState;

            // Handle state transitions
            switch (mSystemState) {
                case HOLDING_POSITION:
                    newState = handleDefaultTransitions(wantedAction, currentState);
                    break;
                case MOVING_TO_POSITION:
                    newState = handleDefaultTransitions(wantedAction, currentState);
                    break;
                default:
                    System.out.println("Unexpected superstructure system state: " + mSystemState);
                    newState = mSystemState;
                    break;
            }

            if (newState != mSystemState) {
                mSystemState = newState;
            }

            mCommandedState = mPlanner.update(currentState);
            return mCommandedState;
        }
    }

    private void updateMotionPlannerDesired(SuperstructureState currentState) {
        // TODO implement regeneration of desired state
        mPlanner.setDesiredState(mDesiredEndState, currentState);
    }

    private SystemState handleDefaultTransitions(WantedAction wantedAction, SuperstructureState currentState) {
        if (scoringPositionChanged()) {
            mLastDesireEndState = mDesiredEndState;
            updateMotionPlannerDesired(currentState);
        }
        return SystemState.MOVING_TO_POSITION;
    }

    public synchronized void outputTelemetry() {
        mPlanner.outputTelemetry();
    }
}