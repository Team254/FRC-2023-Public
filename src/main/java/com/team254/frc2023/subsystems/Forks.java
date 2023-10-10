package com.team254.frc2023.subsystems;

import com.ctre.phoenixpro.signals.ReverseLimitSourceValue;
import com.ctre.phoenixpro.signals.ReverseLimitTypeValue;
import com.ctre.phoenixpro.signals.ReverseLimitValue;
import com.team254.frc2023.Constants;
import com.team254.lib.drivers.ServoMotorSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Forks extends ServoMotorSubsystem {
    private static Forks mInstance = null;

    public synchronized static Forks getInstance() {
        if (mInstance == null) {
            mInstance = new Forks(Constants.kForksConstants);
        }

        return mInstance;
    }

    private Forks(final ServoMotorSubsystemConstants constants) {
        super(constants);

        changeTalonConfig((cfg) -> {
            cfg.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
            cfg.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
            cfg.HardwareLimitSwitch.ReverseLimitEnable = true;
            return cfg;
        });

        zeroSensors();
    }

    @Override
    public boolean atHomingLocation() {
        return mMaster.getReverseLimit().asSupplier().get() == ReverseLimitValue.ClosedToGround;
    }

    @Override
    public synchronized void outputTelemetry(boolean disabled) {
        if (disabled) {
            SmartDashboard.putBoolean("Forks Reverse Limit Switch closed", mMaster.getReverseLimit().asSupplier().get() != ReverseLimitValue.Open);
            SmartDashboard.putNumber("Forks Position Inches", this.getPosition());
            SmartDashboard.putNumber("Forks Position Rotations", this.getPositionRotations());
        }
        super.outputTelemetry(disabled);
    }

    @Override
    public boolean checkSystem() {
        return false;
    }
}