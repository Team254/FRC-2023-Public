package com.team254.lib.drivers;

import com.ctre.phoenixpro.StatusSignalValue;
import com.ctre.phoenixpro.configs.FeedbackConfigs;
import com.ctre.phoenixpro.configs.MotionMagicConfigs;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.*;
import com.ctre.phoenixpro.signals.*;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.team254.frc2023.Constants;
import com.team254.lib.loops.ILooper;
import com.team254.lib.loops.Loop;
import com.team254.lib.motion.IMotionProfileGoal;
import com.team254.lib.motion.MotionProfileConstraints;
import com.team254.lib.motion.MotionState;
import com.team254.lib.motion.SetpointGenerator;
import com.team254.lib.motion.SetpointGenerator.Setpoint;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.UnaryOperator;

/**
 * Abstract base class for a subsystem with a single sensored servo-mechanism.
 */
public abstract class ServoMotorSubsystem extends Subsystem {
    private static final int kMotionMagicSlot = 1;
    private static final int kPositionPIDSlot = 0;

    // Recommend initializing in a static block!
    public static class TalonFXConstants {
        public CanDeviceId id = new CanDeviceId(-1);
        public boolean counterClockwisePositive = true;
        public boolean invert_sensor_phase = false;
        public int encoder_ppr = 2048;
    }

    // Recommend initializing in a static block!
    public static class ServoMotorSubsystemConstants {
        public String kName = "ERROR_ASSIGN_A_NAME";

        public double kLooperDt = 0.01;
        public double kCANTimeout = 0.010; // use for important on the fly updates
        public int kLongCANTimeoutMs = 100; // use for constructors

        public TalonFXConstants kMasterConstants = new TalonFXConstants();
        public TalonFXConstants[] kSlaveConstants = new TalonFXConstants[0];

        public NeutralModeValue kNeutralMode = NeutralModeValue.Brake;
        public double kHomePosition = 0.0; // Units
        public double kRotationsPerUnitDistance = 1.0;
        public double kSoftLimitDeadband = 0.0;
        public double kKp = 0;  // Raw output / raw error
        public double kKi = 0;  // Raw output / sum of raw error
        public double kKd = 0;  // Raw output / (err - prevErr)
        public double kKf = 0;  // Raw output / velocity rps
        public double kKa = 0;  // Raw output / accel in rps / s
        public double kKs = 0;
        public int kDeadband = 0; // rotation

        public double kPositionKp = 0;
        public double kPositionKi = 0;
        public double kPositionKd = 0;
        public double kPositionKf = 0;
        public int kPositionDeadband = 0; // Ticks

        public double kVelocityFeedforward = 0;
        public double kArbitraryFeedforward = 0;
        public double kCruiseVelocity = 0; // Units/s
        public double kAcceleration = 0; // Units/s^2
        public double kJerk = 0; // Units/s^3
        public double kRampRate = 0.0; // s
        public double kMaxVoltage = 12.0;

        public int kSupplyCurrentLimit = 60; // amps
        public boolean kEnableSupplyCurrentLimit = false;

        public int kStatorCurrentLimit = 40; // amps
        public boolean kEnableStatorCurrentLimit = false;

        public double kMaxUnitsLimit = Double.POSITIVE_INFINITY;
        public double kMinUnitsLimit = Double.NEGATIVE_INFINITY;

        public int kStatusFrame8UpdateRate = 1000;
    }

    protected final ServoMotorSubsystemConstants mConstants;
    protected final TalonFX mMaster;
    protected final TalonFX[] mSlaves;

    private TalonFXConfiguration mMasterConfig;
    protected final TalonFXConfiguration[] mSlaveConfigs;


    private final StatusSignalValue<Double> mMasterPositionSignal;
    private final StatusSignalValue<Double> mMasterVelocitySignal;
    private final StatusSignalValue<Double> mMasterClosedLoopError;
    private final StatusSignalValue<Double> mMasterStatorCurrentSignal;
    private final StatusSignalValue<Double> mMasterOutputVoltageSignal;
    private final StatusSignalValue<Double> mMasterOutputPercentageSignal;
    private final StatusSignalValue<Double> mMasterClosedLoopOutputSignal;
    private final StatusSignalValue<Double> mMasterClosedLoopReferenceSignal;
    private final StatusSignalValue<Double> mMasterClosedLoopReferenceSlopeSignal;

    protected MotionState mMotionStateSetpoint = null;

    protected final double mForwardSoftLimitRotations;
    protected final double mReverseSoftLimitRotations;

    protected ServoMotorSubsystem(final ServoMotorSubsystemConstants constants) {
        mConstants = constants;
        mMaster = TalonFXFactory.createDefaultTalon(mConstants.kMasterConstants.id, false);
        mSlaves = new TalonFX[mConstants.kSlaveConstants.length];
        mSlaveConfigs = new TalonFXConfiguration[mConstants.kSlaveConstants.length];

        PhoenixProUtil.checkErrorAndRetry(() -> mMaster.getBridgeOuput().setUpdateFrequency(200, 0.05));
        PhoenixProUtil.checkErrorAndRetry(() -> mMaster.getFault_Hardware().setUpdateFrequency(4, 0.05));

        mMasterPositionSignal = mMaster.getRotorPosition();
        mMasterVelocitySignal = mMaster.getRotorVelocity();
        mMasterClosedLoopError = mMaster.getClosedLoopError();
        mMasterStatorCurrentSignal = mMaster.getStatorCurrent();
        mMasterOutputVoltageSignal = mMaster.getSupplyVoltage();
        mMasterOutputPercentageSignal = mMaster.getDutyCycle();
        mMasterClosedLoopReferenceSignal = mMaster.getClosedLoopReference();
        mMasterClosedLoopOutputSignal = mMaster.getClosedLoopOutput();
        mMasterClosedLoopReferenceSlopeSignal = mMaster.getClosedLoopReferenceSlope();

        PhoenixProUtil.checkErrorAndRetry(() -> mMasterPositionSignal.setUpdateFrequency(200, 0.05));
        PhoenixProUtil.checkErrorAndRetry(() -> mMasterVelocitySignal.setUpdateFrequency(200, 0.05));
        PhoenixProUtil.checkErrorAndRetry(() -> mMasterClosedLoopError.setUpdateFrequency(200, 0.05));
        PhoenixProUtil.checkErrorAndRetry(() -> mMasterStatorCurrentSignal.setUpdateFrequency(200, 0.05));
        PhoenixProUtil.checkErrorAndRetry(() -> mMasterOutputVoltageSignal.setUpdateFrequency(200, 0.05));
        PhoenixProUtil.checkErrorAndRetry(() -> mMasterOutputPercentageSignal.setUpdateFrequency(200, 0.05));
        PhoenixProUtil.checkErrorAndRetry(() -> mMasterClosedLoopReferenceSignal.setUpdateFrequency(200, 0.05));
        PhoenixProUtil.checkErrorAndRetry(() -> mMasterClosedLoopOutputSignal.setUpdateFrequency(200, 0.05));
        PhoenixProUtil.checkErrorAndRetry(() -> mMasterClosedLoopReferenceSlopeSignal.setUpdateFrequency(200, 0.05));

        mMasterConfig = TalonFXFactory.getDefaultConfig();

        mMasterConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        mForwardSoftLimitRotations = (((mConstants.kMaxUnitsLimit - mConstants.kHomePosition) * mConstants.kRotationsPerUnitDistance) - mConstants.kSoftLimitDeadband);
        mMasterConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = mForwardSoftLimitRotations;
        mMasterConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

        mReverseSoftLimitRotations = (((mConstants.kMinUnitsLimit - mConstants.kHomePosition) * mConstants.kRotationsPerUnitDistance) + mConstants.kSoftLimitDeadband);
        mMasterConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = mReverseSoftLimitRotations;
        mMasterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        mMasterConfig.Slot0.kP = mConstants.kPositionKp;
        mMasterConfig.Slot0.kI = mConstants.kPositionKi;
        mMasterConfig.Slot0.kD = mConstants.kPositionKd;
        mMasterConfig.Slot0.kV = mConstants.kPositionKf;

        mMasterConfig.Slot1.kP = mConstants.kPositionKp;
        mMasterConfig.Slot1.kI = mConstants.kPositionKi;
        mMasterConfig.Slot1.kD = mConstants.kPositionKd;
        mMasterConfig.Slot1.kV = mConstants.kVelocityFeedforward;
        mMasterConfig.MotionMagic.MotionMagicCruiseVelocity = mConstants.kCruiseVelocity;
        mMasterConfig.MotionMagic.MotionMagicAcceleration = mConstants.kAcceleration;
        mMasterConfig.MotionMagic.MotionMagicJerk = mConstants.kJerk;

        mMasterConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = mConstants.kRampRate;
        mMasterConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = mConstants.kRampRate;
        mMasterConfig.OpenLoopRamps.TorqueOpenLoopRampPeriod = mConstants.kRampRate;

        mMasterConfig.CurrentLimits.SupplyCurrentLimit = mConstants.kSupplyCurrentLimit;
        mMasterConfig.CurrentLimits.SupplyCurrentLimitEnable = mConstants.kEnableSupplyCurrentLimit;
        mMasterConfig.CurrentLimits.StatorCurrentLimit = mConstants.kStatorCurrentLimit;
        mMasterConfig.CurrentLimits.StatorCurrentLimitEnable = mConstants.kEnableStatorCurrentLimit;

        mMasterConfig.MotorOutput.Inverted = (
                mConstants.kMasterConstants.counterClockwisePositive ?
                        InvertedValue.CounterClockwise_Positive :
                        InvertedValue.Clockwise_Positive
        );

        mMasterConfig.Feedback.SensorToMechanismRatio = (mConstants.kMasterConstants.invert_sensor_phase ? -1 : 1);
        mMasterConfig.MotorOutput.NeutralMode = mConstants.kNeutralMode;

        for (int i = 0; i < mSlaves.length; ++i) {
            mSlaves[i] = TalonFXFactory.createPermanentSlaveTalon(mConstants.kSlaveConstants[i].id, mConstants.kMasterConstants.id, false);

            TalonFX slave = mSlaves[i];
            TalonFXConfiguration slaveConfig = mSlaveConfigs[i];
            PhoenixProUtil.checkErrorAndRetry(() -> slave.getConfigurator().refresh(slaveConfig));

            slaveConfig.MotorOutput.Inverted = (
                    mConstants.kMasterConstants.counterClockwisePositive ?
                            InvertedValue.CounterClockwise_Positive :
                            InvertedValue.Clockwise_Positive
            );
            slaveConfig.MotorOutput.NeutralMode = mConstants.kNeutralMode;
            slave.setControl(new Follower(mConstants.kMasterConstants.id.getDeviceNumber(), false));

            TalonUtil.applyAndCheckConfiguration(slave, slaveConfig);
        }

        // The accel term can re-use the velocity unit conversion because both input and output units are per second.
        mMotionProfileConstraints = new MotionProfileConstraints(rotationsToUnits(mConstants.kCruiseVelocity),
            rotationsToUnits(-mConstants.kCruiseVelocity),
            rotationsToUnits(mConstants.kAcceleration));

        TalonUtil.applyAndCheckConfiguration(mMaster, mMasterConfig);

        // Send a neutral command.
        stop();
    }

    public void setStatorCurrentLimit(double currentLimit, boolean enable) {
        changeTalonConfig((conf) -> {
            conf.CurrentLimits.StatorCurrentLimit = currentLimit;
            conf.CurrentLimits.StatorCurrentLimitEnable = enable;
            return conf;
        });
    }

    public void enableSoftLimits(boolean enable) {
        changeTalonConfig((conf) -> {
            conf.SoftwareLimitSwitch.ForwardSoftLimitEnable = enable;
            conf.SoftwareLimitSwitch.ReverseSoftLimitEnable = enable;
            return conf;
        });
    }

    public void setNeutralMode(NeutralModeValue mode) {
        changeTalonConfig((conf) -> {conf.MotorOutput.NeutralMode = mode; return conf;});
    }

    public void changeTalonConfig(UnaryOperator<TalonFXConfiguration> configChanger) {
        for (int i = 0; i < mSlaves.length; ++i) {
            mSlaveConfigs[i] = configChanger.apply(mSlaveConfigs[i]);
        }
        mMasterConfig = configChanger.apply(mMasterConfig);
        writeConfigs();
    }

    public void writeConfigs() {
        for (int i = 0; i < mSlaves.length; ++i) {
            TalonFX slave = mSlaves[i];
            TalonFXConfiguration slaveConfig = mSlaveConfigs[i];
            TalonUtil.applyAndCheckConfiguration(slave, slaveConfig);
        }
        TalonUtil.applyAndCheckConfiguration(mMaster, mMasterConfig);
    }

    public static class PeriodicIO {
        // INPUTS
        public double timestamp;
        public double position_rots; // motor rotations
        public double position_units;
        public double velocity_rps;
        public double prev_vel_rps;
        public double output_percent;
        public double output_voltage;
        public double master_current;
        public double error_rotations;
        public boolean reset_occured;
        public double active_trajectory_position;
        public double active_trajectory_velocity;
        public double active_trajectory_acceleration;

        // OUTPUTS
        public double demand; // position (motor rots) or percent output
        public double feedforward;
    }

    protected enum ControlState {
        OPEN_LOOP, MOTION_MAGIC, POSITION_PID, MOTION_PROFILING
    }

    protected PeriodicIO mPeriodicIO = new PeriodicIO();
    protected ControlState mControlState = ControlState.OPEN_LOOP;
    protected ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;
    protected boolean mHasBeenZeroed = false;
    protected SetpointGenerator mSetpointGenerator = new SetpointGenerator();
    protected MotionProfileConstraints mMotionProfileConstraints;
    protected StatusSignalValue<Integer> mMasterStickyFault;

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();

        if (mMaster.hasResetOccurred()) {
            DriverStation.reportError(mConstants.kName + ": Talon Reset! ", false);
            mPeriodicIO.reset_occured = true;
            return;
        } else {
            mPeriodicIO.reset_occured = false;
        }

        mMasterStickyFault = mMaster.getStickyFaultField();

        /*
        StatusSignalValue.waitForAll(0.01,
                mMasterClosedLoopError,
                mMasterStatorCurrentSignal,
                mMasterOutputVoltageSignal,
                mMasterOutputPercentageSignal,
                mMasterPositionSignal,
                mMasterVelocitySignal,
                mMasterClosedLoopDerivativeOutputSignal,
                mMasterClosedLoopOutputSignal,
                mMasterClosedLoopReferenceSlopeSignal
        );*/
        mMasterClosedLoopError.refresh();
        mMasterStatorCurrentSignal.refresh();
        mMasterOutputVoltageSignal.refresh();
        mMasterOutputPercentageSignal.refresh();
        mMasterPositionSignal.refresh();
        mMasterVelocitySignal.refresh();
        mMasterClosedLoopOutputSignal.refresh();
        mMasterClosedLoopReferenceSlopeSignal.refresh();

        if (mMaster.getControlMode().getValue() == ControlModeValue.PositionDutyCycle) {
            mPeriodicIO.error_rotations = mMasterClosedLoopError.asSupplier().get();
        } else {
            mPeriodicIO.error_rotations = 0;
        }
        mPeriodicIO.error_rotations = mMasterClosedLoopError.asSupplier().get();
        mPeriodicIO.master_current = mMasterStatorCurrentSignal.asSupplier().get();
        mPeriodicIO.output_voltage = mMasterOutputVoltageSignal.asSupplier().get();
        mPeriodicIO.output_percent = mMasterOutputPercentageSignal.asSupplier().get();
        mPeriodicIO.position_rots = mMasterPositionSignal.asSupplier().get();
        mPeriodicIO.position_units = rotationsToHomedUnits(mPeriodicIO.position_rots);
        mPeriodicIO.velocity_rps = mMasterVelocitySignal.asSupplier().get();
        mPeriodicIO.active_trajectory_position = mMasterClosedLoopReferenceSignal.asSupplier().get();

        final double newVelocity = mMasterClosedLoopReferenceSlopeSignal.asSupplier().get();
        if (Util.epsilonEquals(newVelocity, mConstants.kCruiseVelocity, Math.max(1, mConstants.kDeadband)) || Util
                .epsilonEquals(newVelocity, mPeriodicIO.active_trajectory_velocity, Math.max(1, mConstants.kDeadband))) {
            // Mechanism is ~constant velocity.
            mPeriodicIO.active_trajectory_acceleration = 0.0;
        } else {
            // Mechanism is accelerating.
            mPeriodicIO.active_trajectory_acceleration = Math.signum(newVelocity - mPeriodicIO.active_trajectory_velocity) * mConstants.kAcceleration;
        }
        mPeriodicIO.active_trajectory_velocity = newVelocity;

        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mControlState == ControlState.MOTION_MAGIC) {
            mMaster.setControl(new MotionMagicVoltage(mPeriodicIO.demand).withFeedForward(mPeriodicIO.feedforward).withSlot(kMotionMagicSlot));
        } else if (mControlState == ControlState.POSITION_PID || mControlState == ControlState.MOTION_PROFILING) {
            mMaster.setControl(new PositionDutyCycle(mPeriodicIO.demand).withFeedForward(mPeriodicIO.feedforward).withSlot(kPositionPIDSlot));
        } else {
            mMaster.setControl(new DutyCycleOut(mPeriodicIO.demand));
        }
    }

    public synchronized void handleMasterReset(boolean reset) {}

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                // if (mCSVWriter == null) {
                //     mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/"
                //             + mConstants.kName.replaceAll("[^A-Za-z0-9]+", "").toUpperCase() + "-LOGS.csv",
                //             PeriodicIO.class);
                // }
            }

            @Override
            public void onLoop(double timestamp) {
                if (mPeriodicIO.reset_occured) {
                    System.out.println(mConstants.kName + ": Master Talon reset occurred; resetting frame rates.");
                    PhoenixProUtil.checkErrorAndRetry(() -> mMasterPositionSignal.setUpdateFrequency(200, 0.05));
                    PhoenixProUtil.checkErrorAndRetry(() -> mMasterVelocitySignal.setUpdateFrequency(200, 0.05));
                    PhoenixProUtil.checkErrorAndRetry(() -> mMasterClosedLoopError.setUpdateFrequency(200, 0.05));
                    PhoenixProUtil.checkErrorAndRetry(() -> mMasterStatorCurrentSignal.setUpdateFrequency(200, 0.05));
                    PhoenixProUtil.checkErrorAndRetry(() -> mMasterOutputVoltageSignal.setUpdateFrequency(200, 0.05));
                    PhoenixProUtil.checkErrorAndRetry(() -> mMasterOutputPercentageSignal.setUpdateFrequency(200, 0.05));

                    resetIfAtHome();
                }
                handleMasterReset(mPeriodicIO.reset_occured);
                for (TalonFX slave : mSlaves) {
                    if (slave.hasResetOccurred()) {
                        System.out.println(mConstants.kName + ": Slave Talon reset occurred");
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {
                if (mCSVWriter != null) {
                    mCSVWriter.flush();
                    mCSVWriter = null;
                }

                stop();
            }
        });
    }

    public synchronized double getPositionRotations() {
        return mPeriodicIO.position_rots;
    }

    // In "Units"
    public synchronized double getPosition() {
        return rotationsToHomedUnits(mPeriodicIO.position_rots);
    }

    // In "Units per second"
    public synchronized double getVelocity() {
        return rotationsToUnits(mPeriodicIO.velocity_rps);
    }

    public synchronized double getVelError() {
        if(mMotionStateSetpoint == null) {
            return 0.0;
        }
        return rotationsToUnits(mMotionStateSetpoint.vel() - mPeriodicIO.velocity_rps);
    }

    public synchronized boolean hasFinishedTrajectory() { 
        return Util.epsilonEquals(mPeriodicIO.active_trajectory_position, getSetpoint(), Math.max(1, mConstants.kDeadband));
    }

    public synchronized double getSetpoint() {
        return (mControlState == ControlState.MOTION_MAGIC ||
                mControlState == ControlState.POSITION_PID ||
                mControlState == ControlState.MOTION_PROFILING) ?
                rotationsToHomedUnits(mPeriodicIO.demand) : Double.NaN;
    }

    public synchronized double getSetpointHomed() {
        return (mControlState == ControlState.MOTION_MAGIC ||
                mControlState == ControlState.POSITION_PID ||
                mControlState == ControlState.MOTION_PROFILING) ?
                rotationsToHomedUnits(mPeriodicIO.demand) : Double.NaN;
    }

    public synchronized void setSetpointMotionMagic(double units, double feedforward_v) {
        mPeriodicIO.demand = constrainRotations(homeAwareUnitsToRotations(units));
        mPeriodicIO.feedforward = mConstants.kArbitraryFeedforward;
        if (mControlState != ControlState.MOTION_MAGIC) {
            mControlState = ControlState.MOTION_MAGIC;
        }
    }

    public synchronized void setSetpointMotionMagic(double units) {
        setSetpointMotionMagic(units, 0.0);
    }

    public synchronized void setSetpointPositionPID(double units, double feedforward_v) {
        mPeriodicIO.demand = constrainRotations(homeAwareUnitsToRotations(units));
        double feedforward_ticks_per_100ms = unitsToRotations(feedforward_v);
        mPeriodicIO.feedforward = feedforward_ticks_per_100ms * (mConstants.kKf + mConstants.kKd / 100.0) / 1023.0; 
        if (mControlState != ControlState.POSITION_PID) {
            mControlState = ControlState.POSITION_PID;
        }
    }

    public synchronized void setSetpointPositionPID(double units) {
        setSetpointPositionPID(units, 0.0);
    }

    public synchronized void setSetpointMotionProfiling(IMotionProfileGoal goal, double feedforward_v) {
        if (mControlState != ControlState.MOTION_PROFILING) {
            mControlState = ControlState.MOTION_PROFILING;
            mMotionStateSetpoint = new MotionState(mPeriodicIO.timestamp, mPeriodicIO.position_units, rotationsToUnits(mPeriodicIO.velocity_rps), 0.0);
            mSetpointGenerator.reset();
        }
        Setpoint setpoint = mSetpointGenerator.getSetpoint(mMotionProfileConstraints, goal, mMotionStateSetpoint, mPeriodicIO.timestamp + mConstants.kLooperDt);
        mPeriodicIO.demand = constrainRotations(homeAwareUnitsToRotations(setpoint.motion_state.pos()));
        mPeriodicIO.feedforward = mConstants.kPositionKf + (unitsToRotations(feedforward_v + setpoint.motion_state.vel()) * mConstants.kKf +
            unitsToRotations(setpoint.motion_state.acc()) * mConstants.kKa);
        mMotionStateSetpoint = setpoint.motion_state;
    }

    public double getMPGoal() {
        if(mControlState != ControlState.MOTION_PROFILING) {
            return Double.NaN;
        }
        return mSetpointGenerator.getGoal().pos();
    }

    protected double rotationsToUnits(double rotations) {
        return rotations / mConstants.kRotationsPerUnitDistance;
    }

    protected double rotationsToHomedUnits(double rotations) {
        double val = rotationsToUnits(rotations);
        return val + mConstants.kHomePosition;
    }

    protected double unitsToRotations(double units) {
        return units * mConstants.kRotationsPerUnitDistance;
    }

    protected double homeAwareUnitsToRotations(double units) {
        return unitsToRotations(units - mConstants.kHomePosition);
    }

    protected double constrainRotations(double rotations) {
        return Util.limit(rotations, mReverseSoftLimitRotations, mForwardSoftLimitRotations);
    }

    public synchronized void setOpenLoop(double percentage) {
        if (mControlState != ControlState.OPEN_LOOP) {
            mControlState = ControlState.OPEN_LOOP;
        }
        mPeriodicIO.demand = percentage;
    }

    public synchronized double getActiveTrajectoryPosition() {
        return rotationsToHomedUnits((mPeriodicIO.active_trajectory_position));
    }

    public synchronized String getControlState() {
        return mControlState.toString();
    }

    public synchronized double getPredictedPositionUnits(double lookahead_secs) {
        double predicted_units = mPeriodicIO.active_trajectory_position +
                lookahead_secs * mPeriodicIO.active_trajectory_velocity +
                0.5 * mPeriodicIO.active_trajectory_acceleration * lookahead_secs * lookahead_secs;
        if (mPeriodicIO.demand >= mPeriodicIO.active_trajectory_position) {
            return Math.min(predicted_units, mPeriodicIO.demand);
        } else {
            return Math.max(predicted_units, mPeriodicIO.demand);
        }
    }

    public boolean atHomingLocation() {
        return false;
    }

    public synchronized void resetIfAtHome() {
        if (atHomingLocation()) {
            zeroSensors();
        }
    }

    @Override
    public synchronized void zeroSensors() {
        PhoenixProUtil.checkErrorAndRetry(() -> mMaster.setRotorPosition(0, mConstants.kCANTimeout));
        mHasBeenZeroed = true;
    }

    public synchronized void forceZero() {
        PhoenixProUtil.checkErrorAndRetry(() -> mMaster.setRotorPosition(0, mConstants.kCANTimeout));
    }

    public synchronized boolean hasBeenZeroed() {
        return mHasBeenZeroed;
    }

    public synchronized void setSupplyCurrentLimit(double value, boolean enable) {
        mMasterConfig.CurrentLimits.SupplyCurrentLimit = value;
        mMasterConfig.CurrentLimits.SupplyCurrentLimitEnable = enable;

        TalonUtil.applyAndCheckConfiguration(mMaster, mMasterConfig);
    }

    public synchronized void setSupplyCurrentLimitUnchecked(double value, boolean enable) {
        mMasterConfig.CurrentLimits.SupplyCurrentLimit = value;
        mMasterConfig.CurrentLimits.SupplyCurrentLimitEnable = enable;

        mMaster.getConfigurator().apply(mMasterConfig);
    }

    public synchronized void setStatorCurrentLimitUnchecked(double value, boolean enable) {
        mMasterConfig.CurrentLimits.StatorCurrentLimit = value;
        mMasterConfig.CurrentLimits.StatorCurrentLimitEnable = enable;

        mMaster.getConfigurator().apply(mMasterConfig);
    }

    public synchronized void setMotionMagicConfigsUnchecked(double accel, double jerk) {
        mMasterConfig.MotionMagic.MotionMagicAcceleration = accel;
        mMasterConfig.MotionMagic.MotionMagicJerk = jerk;
        mMaster.getConfigurator().apply(mMasterConfig.MotionMagic);
//        System.out.println("Status:"+ mMaster.getConfigurator().apply(mMasterConfig.MotionMagic));
        //mMaster.setControl(new NeutralOut());
    }

    public synchronized void setMotionMagicConfigs(double accel, double jerk) {
        mMasterConfig.MotionMagic.MotionMagicAcceleration = accel;
        mMasterConfig.MotionMagic.MotionMagicJerk = jerk;
        
        TalonUtil.applyAndCheckConfiguration(mMaster, mMasterConfig);
    }

    @Override
    public void stop() {
        setOpenLoop(0.0);
        mMaster.stopMotor();
    }

    @Override
    public synchronized void outputTelemetry(boolean disabled) {
        if (disabled) {
            SmartDashboard.putNumber(mConstants.kName + ": Position (units)", rotationsToHomedUnits(mPeriodicIO.position_rots));
            SmartDashboard.putBoolean(mConstants.kName + ": Homing Location", atHomingLocation());
            SmartDashboard.putString(mConstants.kName + " control state", mControlState.toString());
            SmartDashboard.putNumber(mConstants.kName + " setpoint units", getSetpoint());
            SmartDashboard.putNumber(mConstants.kName + " vel error", getVelError());
            SmartDashboard.putNumber(mConstants.kName + " mp goal", getMPGoal());
            SmartDashboard.putNumber(mConstants.kName + " Current", mPeriodicIO.master_current);
        }
//        SmartDashboard.putNumber(mConstants.kName + " velocity (rps)", mMasterVelocitySignal.asSupplier().get());
//        SmartDashboard.putNumber(mConstants.kName + " position error(units) ", mMasterClosedLoopError.asSupplier().get() / mConstants.kRotationsPerUnitDistance);
//        SmartDashboard.putNumber(mConstants.kName + " Closed Loop Error", mMasterClosedLoopError.asSupplier().get());
//        SmartDashboard.putNumber(mConstants.kName + " Closed Loop Output", mMasterClosedLoopOutputSignal.asSupplier().get());
//        SmartDashboard.putNumber(mConstants.kName + " Closed Loop Reference", mMasterClosedLoopReferenceSignal.asSupplier().get());
//        SmartDashboard.putNumber(mConstants.kName + " Closed Loop Reference Slope", mMasterClosedLoopReferenceSlopeSignal.asSupplier().get());
//        SmartDashboard.putNumber(mConstants.kName + " Closed Loop Velocity Error", mMasterClosedLoopReferenceSlopeSignal.asSupplier().get() - mMasterVelocitySignal.asSupplier().get());
    }

    @Override
    public void rewriteDeviceConfiguration() {
        writeConfigs();
    }

    @Override
    public boolean checkDeviceConfiguration() {
        if (!TalonUtil.readAndVerifyConfiguration(mMaster, mMasterConfig)) {
            return false;
        }
        for (int i = 0; i < mSlaves.length; ++i) {
            TalonFX slave = mSlaves[i];
            TalonFXConfiguration slaveConfig = mSlaveConfigs[i];
            if (!TalonUtil.readAndVerifyConfiguration(slave, slaveConfig)) {
                return false;
            }
        }
        return true;
    }
}