package com.team254.frc2023;

import com.team254.frc2023.auto.modes.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

public class AutoModeSelector {
    enum DesiredMode {
        DO_NOTHING,
        TEST_TRAJECTORY,
        NCP_2_5,
        NCP_3,
        CP_2_5_OUTSIDE,
        CP_2_5_INSIDE,
        CENTER_1,
        CENTER_2,
        CP_3_OUTSIDE,
        CP_3_INSIDE
    }

    public enum DockMode {
        DOCK,
        NO_DOCK
    }

    public enum PoopMode {
        POOP,
        NO_POOP
    }

    private DesiredMode mCachedDesiredMode = DesiredMode.DO_NOTHING;
    private DockMode mCachedDockMode = DockMode.DOCK;
    private PoopMode mCachedPoopMode = PoopMode.NO_POOP;

    private final SendableChooser<DesiredMode> mModeChooser;
    private final SendableChooser<DockMode> mDockModeChooser;
    private final SendableChooser<PoopMode> mPoopModeChooser;

    private Optional<AutoModeBase> mAutoMode = Optional.empty();

    public AutoModeSelector() {
        mModeChooser = new SendableChooser<>();
        mModeChooser.setDefaultOption("Do Nothing", DesiredMode.DO_NOTHING);
        mModeChooser.addOption("Test Trajectory", DesiredMode.TEST_TRAJECTORY);
        mModeChooser.addOption("NCP 2.5", DesiredMode.NCP_2_5);
        mModeChooser.addOption("NCP 3", DesiredMode.NCP_3);
        mModeChooser.addOption("Center 1", DesiredMode.CENTER_1);
        mModeChooser.addOption("Center 2", DesiredMode.CENTER_2);
        mModeChooser.addOption("CP 2.5 Outside First", DesiredMode.CP_2_5_OUTSIDE);
        mModeChooser.addOption("CP 3 Outside First", DesiredMode.CP_3_OUTSIDE);
        mModeChooser.addOption("CP 2.5 Inside First", DesiredMode.CP_2_5_INSIDE);
        mModeChooser.addOption("CP 3 Inside First", DesiredMode.CP_3_INSIDE);
        SmartDashboard.putData("Auto mode", mModeChooser);

        mDockModeChooser = new SendableChooser<>();
        mDockModeChooser.setDefaultOption("Enable Docking", DockMode.DOCK);
        mDockModeChooser.addOption("Disable Docking", DockMode.NO_DOCK);

        mPoopModeChooser = new SendableChooser<>();
        mPoopModeChooser.setDefaultOption("Disable Poop", PoopMode.NO_POOP);
        mPoopModeChooser.addOption("Enable Poop", PoopMode.POOP);
        SmartDashboard.putData("Auto Mode Docking", mDockModeChooser);
        SmartDashboard.putData("Auto Mode Pooping", mPoopModeChooser);
    }

    public void updateModeCreator() {
        DesiredMode desiredMode = mModeChooser.getSelected();
        DockMode dockMode = mDockModeChooser.getSelected();
        PoopMode poopMode = mPoopModeChooser.getSelected();

        if (desiredMode == null) {
            desiredMode = DesiredMode.DO_NOTHING;
        }

        if (mCachedDesiredMode != desiredMode || mCachedDockMode != dockMode || mCachedPoopMode != poopMode) {
            System.out.println("Auto selection changed, updating creator: desiredMode->" + desiredMode.name() + ((dockMode == DockMode.DOCK) ? " with " : " without ") + "dock" );
            mAutoMode = getAutoModeForParams(desiredMode,dockMode, poopMode);
        }
        mCachedDesiredMode = desiredMode;
        mCachedDockMode = dockMode;
        mCachedPoopMode = poopMode;
    }

    private Optional<AutoModeBase> getAutoModeForParams(DesiredMode mode, DockMode dockMode, PoopMode poopMode) {
        switch (mode) {
            case DO_NOTHING:
                return Optional.of(new DoNothingAutoMode());
            case TEST_TRAJECTORY:
                return Optional.of(new TestTrajectoryFollowingMode(dockMode));
            case NCP_2_5:
                return Optional.of(new NCPTwoPointFiveMode(dockMode, poopMode));
            case NCP_3:
                return Optional.of(new NCPThreeMode());
            case CENTER_1:
                return Optional.of(new CenterOneMode(dockMode));
            case CENTER_2:
                return Optional.of(new CenterTwoMode(dockMode));
            case CP_2_5_OUTSIDE:
                return Optional.of(new CPTwoPointFiveOutsideFirstMode(dockMode));
            case CP_3_OUTSIDE:
                return Optional.of(new CPThreeOutsideFirstMode(dockMode));
            case CP_2_5_INSIDE:
                return Optional.of(new CPTwoPointFiveInsideFirstMode(dockMode));
            case CP_3_INSIDE:
                return Optional.of(new CPThreeInsideFirstMode(dockMode));
            default:
                break;
        }

        System.err.println("No valid auto mode found for  " + mode);
        return Optional.empty();
    }

    public void reset() {
        mAutoMode = Optional.empty();
        mCachedDesiredMode = DesiredMode.DO_NOTHING;
        mCachedDockMode = DockMode.DOCK;
        mCachedPoopMode = PoopMode.NO_POOP;
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putString("AutoModeSelected", mCachedDesiredMode.name() + ((mCachedDockMode == DockMode.DOCK) ? " with " : " without ") + "dock" + " and" +  ((mCachedPoopMode == PoopMode.POOP) ? " with " : " without ") + "poop" );
    }

    public Optional<AutoModeBase> getAutoMode() {
        return mAutoMode;
    }
}