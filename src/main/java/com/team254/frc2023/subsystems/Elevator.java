package com.team254.frc2023.subsystems;

import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.ctre.phoenixpro.signals.ReverseLimitSourceValue;
import com.ctre.phoenixpro.signals.ReverseLimitTypeValue;
import com.ctre.phoenixpro.signals.ReverseLimitValue;
import com.team254.lib.drivers.*;
import com.team254.frc2023.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator extends ServoMotorSubsystem {
    private static Elevator mInstance = null;

    private boolean reverse_limit_switch = false;

    public synchronized static Elevator getInstance() {
        if (mInstance == null) {
            mInstance = new Elevator(Constants.kElevatorConstants);
        }

        return mInstance;
    }

    private Elevator(final ServoMotorSubsystemConstants constants) {
        super(constants);

        changeTalonConfig((cfg) -> {
            cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            cfg.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
            cfg.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
            cfg.HardwareLimitSwitch.ReverseLimitEnable = true;
            return cfg;
        });
        zeroSensors();
    }

    @Override
    public synchronized void readPeriodicInputs() {
        super.readPeriodicInputs();
        reverse_limit_switch = atHomingLocation();
    }

    @Override
    public boolean atHomingLocation() {
        return mMaster.getReverseLimit().asSupplier().get() == ReverseLimitValue.ClosedToGround;
    }

    @Override
    public synchronized void outputTelemetry(boolean disabled) {
        if (disabled) {
            SmartDashboard.putBoolean("Elevator Reverse Limit Switch closed", reverse_limit_switch);
            SmartDashboard.putNumber("Elevator Position Inches", this.getPosition());
            SmartDashboard.putNumber("Elevator Position Rotations", this.getPositionRotations());
        }
        super.outputTelemetry(disabled);
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

}
