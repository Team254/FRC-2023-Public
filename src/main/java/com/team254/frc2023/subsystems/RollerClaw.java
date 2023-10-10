package com.team254.frc2023.subsystems;

import com.ctre.phoenixpro.StatusSignalValue;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.*;
import com.team254.frc2023.Constants;
import com.team254.lib.drivers.*;
import com.team254.lib.loops.ILooper;
import com.team254.lib.loops.Loop;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RollerClaw extends Subsystem {
    private static final double kConeIntakeTriggeringDelay = 0.5;//(Constants.kPracticeBot ? 2.0 : 0.5);
    private static final double kCubeIntakeTriggeringDelay = 0.0;
    private static final double kIntakeOpeningTimeout = 1.0;
    private static final double kReGrippingTimeout = 1.0;

    private static final double kIntakingConePercentOutput = -1.0;
    private static final double kIntakingCubePercentOutput = -1.0;
    private static final double kGrippingPercentOutput = -0.08;
    private static final double kConeOpeningPercentOutput = 1.0;
    private static final double kCubeOpeningPercentOutput = 0.35;
    private static final double kConeCloseFlingingPercentOutput = 0.3;
    private static final double kConeFarFlingingPercentOutput = 0.9;
    private static final double kCubeCloseFlingingPercentOutput = 0.4;
    private static final double kCubeFarFlingingPercentOutput = 0.7;
    private static final double kConeLowScoringPercentOutput = 0.4;
    private static final double kCubeLowScoringPercentOutput = 0.25;

    private static RollerClaw mInstance = null;
    public synchronized static RollerClaw getInstance() {
        if (mInstance == null) {
            mInstance = new RollerClaw();
        }

        return mInstance;
    }

    private RollerClaw() {
        mMaster = TalonFXFactory.createDefaultTalon(new CanDeviceId(Constants.kIntakeId, Constants.kCANivoreCANBusName));

        mMasterConfig = new TalonFXConfiguration();
        PhoenixProUtil.checkErrorAndRetry(() -> mMaster.getConfigurator().refresh(mMasterConfig));
        mMasterConfig.CurrentLimits.StatorCurrentLimitEnable = false;
        mMasterConfig.CurrentLimits.SupplyCurrentLimit = 30;
        mMasterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        mMasterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        mMasterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;// (Constants.kPracticeBot ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive);
        mMasterConfig.HardwareLimitSwitch.ForwardLimitEnable = false;
        mMasterConfig.HardwareLimitSwitch.ReverseLimitEnable = false;
        TalonUtil.applyAndCheckConfiguration(mMaster, mMasterConfig);

        mMasterStatorCurrentSignal = mMaster.getStatorCurrent();
        PhoenixProUtil.checkErrorAndRetry(() -> mMasterStatorCurrentSignal.setUpdateFrequency(200));

        mMasterReverseLimitSignal = mMaster.getReverseLimit();
        PhoenixProUtil.checkErrorAndRetry(() -> mMasterReverseLimitSignal.setUpdateFrequency(200));
    }

    public static class PeriodicIO {
        public double intakeCurrent;
        public boolean limitSwitchTriggered;

        public boolean lastLimitSwitchTriggered;

        public double triggerTime = 0;

        public double untriggerTime = 0;

        public double stateTransitionTime = 0;
        public double openLoopDemand;
        public boolean gameObjectCube = false;
        public boolean isReGripping = false;
    }

    public enum SystemState {
        INTAKING_CONE, INTAKING_CUBE, IDLE, OPENING, GRIPPING, FAR_FLINGING_CONE, CLOSE_FLINGING_CONE, CLOSE_FLINGING_CUBE, FAR_FLINGING_CUBE
    }

    private static StatusSignalValue<Double> mMasterStatorCurrentSignal;
    private static StatusSignalValue<ReverseLimitValue> mMasterReverseLimitSignal;
    private final PeriodicIO mPeriodicIO = new PeriodicIO();
    private static SystemState mSystemState = SystemState.IDLE;
    private static TalonFX mMaster;
    private TalonFXConfiguration mMasterConfig;

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.intakeCurrent = mMasterStatorCurrentSignal.asSupplier().get();
        // very important to do the next 2 lines in this order.
        mPeriodicIO.lastLimitSwitchTriggered = mPeriodicIO.limitSwitchTriggered;
        mPeriodicIO.limitSwitchTriggered = mMasterReverseLimitSignal.asSupplier().get() == ReverseLimitValue.Open;
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {

            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (RollerClaw.this) {
                    if (mPeriodicIO.limitSwitchTriggered && !mPeriodicIO.lastLimitSwitchTriggered) {
                        mPeriodicIO.triggerTime = timestamp;
                    } else if (!mPeriodicIO.limitSwitchTriggered && mPeriodicIO.lastLimitSwitchTriggered) {
                        mPeriodicIO.untriggerTime = timestamp;

                        if (mSystemState == SystemState.GRIPPING && mPeriodicIO.gameObjectCube) {
                            wantCubeIntake(); // Re-intake cubes if slipping out
                            mPeriodicIO.isReGripping = true;
                        }
                    }

                    if (mPeriodicIO.isReGripping
                            && Timer.getFPGATimestamp() - mPeriodicIO.untriggerTime > kReGrippingTimeout) {
                        mPeriodicIO.isReGripping = false;
                        wantNeutral();
                    }

                    SystemState nextState = mSystemState;
                    switch (mSystemState) {
                        case IDLE:
                            mPeriodicIO.openLoopDemand = 0.0;
                            break;
                        case INTAKING_CONE:
                            if (doneConeIntaking(timestamp)) {
                                nextState = SystemState.GRIPPING;
//                                mPeriodicIO.openLoopDemand = kConeGrippingPercentOutput;
                                mPeriodicIO.openLoopDemand = kGrippingPercentOutput;
                            }
                            break;
                        case INTAKING_CUBE:
                            if (doneCubeIntaking(timestamp)) {
                                nextState = SystemState.GRIPPING;
//                                mPeriodicIO.openLoopDemand = kCubeGrippingPercentOutput;
                                mPeriodicIO.openLoopDemand = kGrippingPercentOutput;
                            }
                            break;
                        case OPENING:
                            if (doneOpening(timestamp)) {
                                nextState = SystemState.IDLE;
                                mPeriodicIO.openLoopDemand = 0.0;
                            }
                            break;
                        case GRIPPING:
                            mPeriodicIO.openLoopDemand = kGrippingPercentOutput;
                            break;
                        case FAR_FLINGING_CONE:
                            break;
                        case CLOSE_FLINGING_CONE:
                            break;
                        case CLOSE_FLINGING_CUBE:
                            break;
                        case FAR_FLINGING_CUBE:
                            break;
                    }

                    if (mSystemState != nextState) {
                        mPeriodicIO.stateTransitionTime = timestamp;
                    }
                    mSystemState = nextState;
                }
            }

            public void onStop(double timestamp) {
                stop();
            }
        });
    }

    @Override
    public void writePeriodicOutputs() {
        mMaster.setControl(new DutyCycleOut(mPeriodicIO.openLoopDemand));
    }

    public boolean doneOpening(double timestamp) {
        return timestamp - mPeriodicIO.stateTransitionTime > kIntakeOpeningTimeout;
    }

    public boolean doneConeIntaking(double timestamp) {
        return  mPeriodicIO.limitSwitchTriggered && ((timestamp - mPeriodicIO.triggerTime) > kConeIntakeTriggeringDelay);
    }

    public boolean doneCubeIntaking(double timestamp) {
        return mPeriodicIO.limitSwitchTriggered && ((timestamp - mPeriodicIO.triggerTime) > kCubeIntakeTriggeringDelay);
    }

    public void wantConeIntake() {
        mSystemState = SystemState.INTAKING_CONE;
        mPeriodicIO.gameObjectCube = false;
        mPeriodicIO.openLoopDemand = kIntakingConePercentOutput;
    }

    public void wantCubeIntake() {
        mSystemState = SystemState.INTAKING_CUBE;
        mPeriodicIO.gameObjectCube = true;
        mPeriodicIO.openLoopDemand = kIntakingCubePercentOutput;
    }

    public void wantOpen(boolean isCone) {
        mSystemState = SystemState.OPENING;
        mPeriodicIO.stateTransitionTime = Timer.getFPGATimestamp();
        if (isCone) {
            mPeriodicIO.openLoopDemand = kConeOpeningPercentOutput;
        } else {
            mPeriodicIO.openLoopDemand = kCubeOpeningPercentOutput;
        }
    }

    public void wantOpen(boolean isCone, boolean isLow) {
        mSystemState = SystemState.OPENING;
        mPeriodicIO.stateTransitionTime = Timer.getFPGATimestamp();
        if (isLow && isCone) {
            mPeriodicIO.openLoopDemand = kConeLowScoringPercentOutput;
        } else if (isLow) {
            mPeriodicIO.openLoopDemand = kCubeLowScoringPercentOutput;
        } else if (isCone) {
            mPeriodicIO.openLoopDemand = kConeOpeningPercentOutput;
        } else {
            mPeriodicIO.openLoopDemand = kCubeOpeningPercentOutput;
        }
    }
    public void wantFling(boolean isCone, boolean isFar){
        if (isFar){
            mSystemState = isCone ?  SystemState.FAR_FLINGING_CONE : SystemState.FAR_FLINGING_CUBE;
            mPeriodicIO.openLoopDemand = isCone ? kConeFarFlingingPercentOutput : kCubeFarFlingingPercentOutput;
        }else {
            mSystemState = isCone ? SystemState.CLOSE_FLINGING_CONE : SystemState.CLOSE_FLINGING_CUBE;
            mPeriodicIO.openLoopDemand = isCone ?  kConeCloseFlingingPercentOutput : kCubeCloseFlingingPercentOutput;
        }
    }
    public void wantNeutral() {

        mSystemState = SystemState.GRIPPING;
    }

    public boolean hasGamePiece() {
        return mPeriodicIO.limitSwitchTriggered;
    }


    @Override
    public void stop() {

    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public synchronized void outputTelemetry(boolean disabled) {
        if (disabled) {
            SmartDashboard.putNumber("Roller Claw Current", mPeriodicIO.intakeCurrent);
            SmartDashboard.putString("Roller Claw State", mSystemState.toString());
            SmartDashboard.putNumber("Roller Claw Open Loop Demand", mPeriodicIO.openLoopDemand);
            SmartDashboard.putBoolean("Roller Claw Limit Triggered", mMaster.getReverseLimit().asSupplier().get() == ReverseLimitValue.Open);
        }
    }

    @Override
    public void rewriteDeviceConfiguration() {
        TalonUtil.applyAndCheckConfiguration(mMaster, mMasterConfig);
    }

    @Override
    public boolean checkDeviceConfiguration() {
        return TalonUtil.readAndVerifyConfiguration(mMaster, mMasterConfig);
    }
}
