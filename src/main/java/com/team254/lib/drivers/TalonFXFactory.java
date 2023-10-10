package com.team254.lib.drivers;

import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.configs.*;
import com.ctre.phoenixpro.controls.*;
import com.ctre.phoenixpro.signals.*;

/**
 * Creates CANTalon objects and configures all the parameters we care about to factory defaults. Closed-loop and sensor
 * parameters are not set, as these are expected to be set by the application.
 */
public class TalonFXFactory {

    public static NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;
    public static InvertedValue INVERT_VALUE = InvertedValue.CounterClockwise_Positive;
    public static double NEUTRAL_DEADBAND = 0.04;

    // create a CANTalon with the default (out of the box) configuration
    public static TalonFX createDefaultTalon(CanDeviceId id) {
        return createDefaultTalon(id, true);
    }

    public static TalonFX createDefaultTalon(CanDeviceId id, boolean trigger_config) {
        var talon = createTalon(id);
        if (trigger_config) {
            TalonUtil.applyAndCheckConfiguration(talon, getDefaultConfig());
        }
        return talon;
    }

    public static TalonFX createPermanentSlaveTalon(CanDeviceId slave_id, CanDeviceId master_id, boolean opposeMasterDirection) {
        if(!slave_id.getBus().equals(master_id.getBus())) {
                throw new RuntimeException("Master and Slave Talons must be on the same CAN bus");
        }
        final TalonFX talon = createTalon(slave_id);
        talon.setControl(new Follower(master_id.getDeviceNumber(), opposeMasterDirection));
        return talon;
    }

    public static TalonFXConfiguration getDefaultConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NEUTRAL_MODE;
        config.MotorOutput.Inverted = INVERT_VALUE;
        config.MotorOutput.DutyCycleNeutralDeadband = NEUTRAL_DEADBAND;
        config.MotorOutput.PeakForwardDutyCycle = 1.0;
        config.MotorOutput.PeakReverseDutyCycle = -1.0;

        config.CurrentLimits.SupplyCurrentLimitEnable = false;
        config.CurrentLimits.StatorCurrentLimitEnable = false;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.FeedbackRotorOffset = 0;
        config.Feedback.SensorToMechanismRatio = 1;

        config.HardwareLimitSwitch.ForwardLimitEnable = false;
        config.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = false;
        config.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;
        config.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyOpen;
        config.HardwareLimitSwitch.ReverseLimitEnable = false;
        config.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = false;
        config.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
        config.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;

        config.Audio.BeepOnBoot = true;

        return config;
    }

    private static TalonFX createTalon(CanDeviceId id) {
        TalonFX talon = new TalonFX(id.getDeviceNumber(), id.getBus());
        talon.clearStickyFaults();

        return talon;
    }
}