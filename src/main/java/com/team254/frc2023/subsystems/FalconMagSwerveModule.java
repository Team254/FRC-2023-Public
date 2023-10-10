package com.team254.frc2023.subsystems;

import com.team254.lib.drivers.CanDeviceId;
import com.team254.lib.drivers.SwerveModule;
import com.team254.lib.geometry.Rotation2d;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class FalconMagSwerveModule extends SwerveModule {

    private final DutyCycleEncoder mMagEncoder;

    public FalconMagSwerveModule(CanDeviceId driveId, CanDeviceId steeringId, int magEncoderPortID, Rotation2d encoderZero) {
        super(driveId, steeringId, encoderZero);
        DigitalInput in = new DigitalInput(magEncoderPortID);
        DutyCycle cycle = new DutyCycle(in);
        mMagEncoder =  new DutyCycleEncoder(cycle);
        mMagEncoder.setDistancePerRotation(360);

        rezeroSteeringMotor();

        stop();
    }

     @Override
    public Rotation2d getSteerEncoderAngle() {
        return Rotation2d.fromDegrees(mMagEncoder.getDistance());
    }

}
