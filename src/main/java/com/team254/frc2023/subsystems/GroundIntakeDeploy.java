package com.team254.frc2023.subsystems;

import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.ctre.phoenixpro.signals.ReverseLimitSourceValue;
import com.ctre.phoenixpro.signals.ReverseLimitTypeValue;
import com.ctre.phoenixpro.signals.ReverseLimitValue;
import com.team254.lib.drivers.*;
import com.team254.frc2023.Constants;
import com.team254.lib.loops.ILooper;
import com.team254.lib.loops.Loop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GroundIntakeDeploy extends ServoMotorSubsystem {
    private static GroundIntakeDeploy mInstance = null;

    public synchronized static GroundIntakeDeploy getInstance() {
        if (mInstance == null) {
            mInstance = new GroundIntakeDeploy(Constants.kGroundIntakeDeployConstants);
        }

        return mInstance;
    }

    private boolean reverse_limit_switch = false;

    private GroundIntakeDeploy(final ServoMotorSubsystemConstants constants) {
        super(constants);

        TalonFXConfiguration config = new TalonFXConfiguration();
        PhoenixProUtil.checkErrorAndRetry(() -> mMaster.getConfigurator().refresh(config));
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
        config.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
        config.HardwareLimitSwitch.ReverseLimitEnable = true;
        TalonUtil.applyAndCheckConfiguration(mMaster, config);
        zeroSensors();
    }

    @Override
    public synchronized void readPeriodicInputs() {
        super.readPeriodicInputs();
        reverse_limit_switch = atHomingLocation();
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
            }

            @Override
            public void onLoop(double timestamp) {

            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }

    @Override
    public boolean atHomingLocation() {
        return mMaster.getReverseLimit().asSupplier().get() == ReverseLimitValue.ClosedToGround;
    }

    @Override
    public synchronized void outputTelemetry(boolean disabled) {
        // TODO: Move reading this limit switch to mPeriodicIO in servomotor subsystem
        if (disabled) {
            SmartDashboard.putBoolean("Ground Intake Reverse Limit Switch closed", reverse_limit_switch);
            SmartDashboard.putNumber("Ground Intake Position Inches", this.getPosition());
            SmartDashboard.putNumber("Ground Intake Position Rotations", this.getPositionRotations());
        }
        super.outputTelemetry(disabled);
    }

    public void push() {
        setSetpointMotionMagic(Constants.kGroundIntakePushinPosition);
    }

    public void extend() {
        setSetpointMotionMagic(Constants.kGroundIntakeDeployExtendPosition);
    }

    public void stow() {
        setSetpointMotionMagic(Constants.kGroundIntakeDeployStowPosition);
    }

    @Override
    public boolean checkSystem() {
        return false;
    }
}