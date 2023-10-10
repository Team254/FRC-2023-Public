package com.team254.frc2023.subsystems;

import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.ctre.phoenixpro.signals.ReverseLimitSourceValue;
import com.ctre.phoenixpro.signals.ReverseLimitTypeValue;
import com.ctre.phoenixpro.signals.ReverseLimitValue;
import com.team254.lib.drivers.*;
import com.team254.frc2023.Constants;
import com.team254.lib.loops.ILooper;
import com.team254.lib.loops.Loop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Laterator extends ServoMotorSubsystem {
    private static Laterator mInstance = null;

    public synchronized static Laterator getInstance() {
        if (mInstance == null) {
            mInstance = new Laterator(Constants.kLateratorConstants);
        }

        return mInstance;
    }

    private boolean reverse_limit_switch = false;

    private Laterator(final ServoMotorSubsystemConstants constants) {
        super(constants);

        changeTalonConfig((cfg) -> {
            cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            cfg.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
            cfg.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
            cfg.HardwareLimitSwitch.ReverseLimitEnable = true;
            return cfg;
        });
        reverse_limit_switch = atHomingLocation();

        zeroSensors();
    }

    @Override
    public synchronized void readPeriodicInputs() {
        super.readPeriodicInputs();
        reverse_limit_switch = atHomingLocation();

        // Reset if below former zero
        if (mPeriodicIO.position_units < mConstants.kHomePosition) {
            zeroSensors();
        }
    }

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
        if (disabled) {
            SmartDashboard.putBoolean("Laterator Reverse Limit Switch closed", reverse_limit_switch);
            SmartDashboard.putNumber("Laterator Position Inches", this.getPosition());
            SmartDashboard.putNumber("Laterator Position Rotations", this.getPositionRotations());
        }
        super.outputTelemetry(disabled);
    }

    @Override
    public boolean checkSystem() {
        return false;
    }
}