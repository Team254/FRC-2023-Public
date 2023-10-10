package com.team254.frc2023.auto.actions;

import com.team254.frc2023.RobotState;
import com.team254.frc2023.planners.AutoAlignPointSelector;
import com.team254.frc2023.subsystems.Drive;
import com.team254.frc2023.subsystems.Superstructure;
import com.team254.lib.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;

import java.util.Optional;


public class AutoAlignAction implements Action{
    private final Superstructure mSuperstructure = Superstructure.getInstance();
    private final Drive mDrive = Drive.getInstance();

    @Override
    public void start() {
        setAutoAlignDriveOutput();
    }

    @Override
    public void update() {
        setAutoAlignDriveOutput();
    }

    private void setAutoAlignDriveOutput() {
        double t = Timer.getFPGATimestamp();
        Optional<Pose2d> targetPoint = AutoAlignPointSelector.chooseTargetPoint(RobotState.getInstance().getFieldToVehicle(t),
                mSuperstructure.getGameObjectType() == Superstructure.GameObjectType.CONE ? AutoAlignPointSelector.RequestedAlignment.AUTO_CONE : AutoAlignPointSelector.RequestedAlignment.AUTO_CUBE);
        if (!targetPoint.isEmpty()) {
            mDrive.setSnapToPoint(targetPoint.get());
        }
    }


    @Override
    public boolean isFinished() {
        return mDrive.getAutoAlignComplete();
    }

    @Override
    public void done() {}
}