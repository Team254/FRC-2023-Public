package com.team254.frc2023.subsystems;

import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.team254.frc2023.Constants;
import com.team254.lib.drivers.*;
import com.team254.lib.loops.ILooper;
import com.team254.lib.loops.Loop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GroundIntakeRoller extends Subsystem {
    private TalonFX mTalon;
    private TalonFXConfiguration mTalonConfiguration;

    private static GroundIntakeRoller sInstance = null;

    public GroundIntakeRoller() {
        mTalon = TalonFXFactory.createDefaultTalon(new CanDeviceId(Constants.kGroundIntakeRollerId, Constants.kCANivoreCANBusName));

        mTalonConfiguration = new TalonFXConfiguration();
        PhoenixProUtil.checkErrorAndRetry(() -> mTalon.getConfigurator().refresh(mTalonConfiguration));
        mTalonConfiguration.CurrentLimits.StatorCurrentLimit = 60;
        mTalonConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        mTalonConfiguration.CurrentLimits.SupplyCurrentLimit = 20;
        mTalonConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        mTalonConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        mTalonConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        mTalonConfiguration.HardwareLimitSwitch.ForwardLimitEnable = false;
        mTalonConfiguration.HardwareLimitSwitch.ReverseLimitEnable = false;
        TalonUtil.applyAndCheckConfiguration(mTalon, mTalonConfiguration);
    }

    public static GroundIntakeRoller getInstance() {
        if (sInstance == null) {
            sInstance = new GroundIntakeRoller();
        }
        return sInstance;
    }

    public enum DesiredState {
        IDLE, INTAKE, EXHUAST, PUSH
    }

    private DesiredState mDesiredState = DesiredState.IDLE;

    public void setDesiredState(DesiredState state) {
        mDesiredState = state;
    }

    @Override
    public void stop() {
        mDesiredState = DesiredState.IDLE;
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {

            }

            @Override
            public void onLoop(double timestamp) {
                switch (mDesiredState) {
                    case IDLE:
                        mTalon.setControl(new DutyCycleOut(0));
                        break;
                    case INTAKE:
                        mTalon.setControl(new DutyCycleOut(0.5));
                        break;
                    case PUSH:
                        mTalon.setControl(new DutyCycleOut(0.0));
                        break;
                    case EXHUAST:
                        mTalon.setControl(new DutyCycleOut(0.0));
                        break;
                }
            }

            @Override
            public void onStop(double timestamp) {

            }
        });
    }

    @Override
    public synchronized void outputTelemetry(boolean disabled) {
        if (disabled) {
            SmartDashboard.putString("Ground intake roller state", mDesiredState.toString());
        }
    }

    @Override
    public void rewriteDeviceConfiguration() {
        TalonUtil.applyAndCheckConfiguration(mTalon, mTalonConfiguration);
    }

    @Override
    public boolean checkDeviceConfiguration() {
        return TalonUtil.readAndVerifyConfiguration(mTalon, mTalonConfiguration);
    }
}
