package com.team254.frc2023.subsystems;

import com.ctre.phoenixpro.hardware.CANcoder;
import com.team254.lib.drivers.CanDeviceId;
import com.team254.lib.drivers.SwerveModule;
import com.team254.lib.geometry.Rotation2d;

// SDS Mk4i L3 Internals
public class FalconCANCoderSwerveModule extends SwerveModule {
    private final CANcoder mCanCoder;

    public FalconCANCoderSwerveModule(CanDeviceId driveId, CanDeviceId steeringId,  CANcoder cancoder, Rotation2d encoderZero) {
        super(driveId, steeringId, encoderZero);
        mCanCoder = cancoder;

        rezeroSteeringMotor();

        stop();
    }

    @Override
    public Rotation2d getSteerEncoderAngle() {
        return Rotation2d.fromDegrees(mCanCoder.getAbsolutePosition().refresh().getValue() * 360);
    }

}
