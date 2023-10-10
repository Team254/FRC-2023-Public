package com.team254.frc2023.statemachines;

import com.team254.frc2023.Constants;
import com.team254.lib.util.Util;

public class SuperstructureState {
    // position on x-z plane (adjust for laterator angle)
    public double height = Constants.kElevatorConstants.kHomePosition;
    public double extension = Constants.kLateratorConstants.kHomePosition;
    public double elevatorAllowableError = Constants.kLiberalElevatorAllowableError;
    public double lateratorAllowableError = Constants.kLiberalLateratorAllowableError;
    public Action action;
    public SuperstructureConfig config;

    public enum Action {
        SCORING_WITH_LOWER,
        SCORING_WITH_LOWER_AUTO,
        SCORING_WITHOUT_LOWER,
        SCORING,
        NEUTRAL,
        INTAKING_GROUND_CONE,
        INTAKING_FROM_FEEDER,
        INTAKING_GROUND_CUBE,
        WAIT_WHILE_SCORING,
        DROPPING,
        FLINGING,
        DRIBBLING
    }

    public enum SuperstructureConfig {
        HIGH,
        MEDIUM,
        LOW
    }

    public SuperstructureState(double height, double extension, double elevatorAllowableError, double lateratorAllowableError, Action action, SuperstructureConfig config) {
        this.height = height;
        this.extension = extension;
        this.elevatorAllowableError = elevatorAllowableError;
        this.lateratorAllowableError = lateratorAllowableError;
        this.action = action;
        this.config = config;
    }

    public SuperstructureState(SuperstructureState other) {
        this(other.height, other.extension, other.elevatorAllowableError, other.lateratorAllowableError, other.action, other.config);
    }

    public SuperstructureState(double height, double extension, Action action) {
        this(height, extension, Constants.kLiberalElevatorAllowableError, Constants.kLiberalLateratorAllowableError, action, SuperstructureConfig.MEDIUM);
    }

    public SuperstructureState(double height, double extension, Action action, SuperstructureConfig config) {
        this(height, extension, Constants.kLiberalElevatorAllowableError, Constants.kLiberalLateratorAllowableError, action, config);
    }

    public SuperstructureState(double height, double extension) {
        this(height, extension, Constants.kLiberalElevatorAllowableError, Constants.kLiberalLateratorAllowableError, Action.NEUTRAL, SuperstructureConfig.MEDIUM);
    }

    public SuperstructureState() {}

    public double getElevatorHeight() {
        return Util.limit(height, Constants.kElevatorConstants.kMinUnitsLimit, Constants.kElevatorConstants.kMaxUnitsLimit);
    }

    public double getLateratorExtension() {
        return Util.limit(extension, Constants.kLateratorConstants.kMinUnitsLimit, Constants.kLateratorConstants.kMaxUnitsLimit);
    }

    public boolean isInRange(SuperstructureState other) {
        return isInRange(other, Math.min(this.elevatorAllowableError, other.elevatorAllowableError), Math.min(this.lateratorAllowableError, other.lateratorAllowableError));
    }

    public boolean isInRange(SuperstructureState other, double elevatorAllowableError, double lateratorAllowableError) {
        return Util.epsilonEquals(this.getElevatorHeight(), other.getElevatorHeight(), elevatorAllowableError)
                && Util.epsilonEquals(this.getLateratorExtension(), other.getLateratorExtension(), lateratorAllowableError);
    }

    public String toString() {
        return "Superstructure state: (" + getElevatorHeight() + ", " + getLateratorExtension() + ", " + action.toString() + ")";
    }
}
