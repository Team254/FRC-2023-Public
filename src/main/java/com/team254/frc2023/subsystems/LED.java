package com.team254.frc2023.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.team254.frc2023.Constants;
import com.team254.frc2023.led.LEDStateContainer;
import com.team254.frc2023.led.TimedLEDState;
import com.team254.lib.drivers.Subsystem;
import com.team254.lib.loops.ILooper;
import com.team254.lib.loops.Loop;

import edu.wpi.first.wpilibj.DriverStation;

public class LED extends Subsystem {

    public enum WantedAction {
        DISPLAY_BATTERY_LOW,
        DISPLAY_GOOD_BATTERY,
        DISPLAY_NOT_HOMED,
        DISPLAY_SUPERSTRUCTURE,
        DISPLAY_VISION,
        DISPLAY_CONFIGURE_FAULT,
        OFF
    }

    private enum SystemState {
        DISPLAYING_BATTERY_LOW,
        DISPLAYING_CONFIGURE_FAULT,
        DISPLAYING_GOOD_BATTERY,
        DISPLAYING_NOT_HOMED,
        DISPLAYING_SUPERSTRUCTURE,
        DISPLAYING_VISION,
        OFF
    }

    private static LED mInstance;

    private CANdle mCANdle;
    private SystemState mSystemState = SystemState.OFF;
    private WantedAction mWantedAction = WantedAction.OFF;

    private LEDStateContainer mDesiredLEDState = new LEDStateContainer();

    private TimedLEDState mSuperstructureLEDState = TimedLEDState.StaticLEDState.kStaticOff;


    public synchronized static LED getInstance() {
        if (mInstance == null) {
            mInstance = new LED();
        }
        return mInstance;
    }

    private LED() {
        mCANdle = new CANdle(Constants.kCANdleId, Constants.kCANivoreCANBusName);
    }

    public synchronized void setSuperstructureLEDState(TimedLEDState intakeLEDState) {
        mSuperstructureLEDState = intakeLEDState;
    }

    public synchronized void setWantedAction(WantedAction wantedAction) {
        mWantedAction = wantedAction;
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            double stateStartTime;

            @Override
            public void onStart(double timestamp) {
                stateStartTime = timestamp;
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (LED.this) {
                    SystemState newState = getStateTransition();
                    if (mSystemState != newState) {
                        System.out.println(timestamp + ": LED changed state: " + mSystemState + " -> " + newState);
                        mSystemState = newState;
                        stateStartTime = timestamp;
                    }
                    double timeInState = timestamp - stateStartTime;
                    switch (mSystemState) {
                        case DISPLAYING_SUPERSTRUCTURE:
                            setSuperstructureLEDCommand(timeInState);
                            break;
                        case DISPLAYING_BATTERY_LOW:
                            setBatteryLowCommand(timeInState);
                            break;
                        case DISPLAYING_GOOD_BATTERY:
                            setGoodBattery(timeInState);
                            break;
                        case DISPLAYING_NOT_HOMED:
                            setNotHomedCommand(timeInState);
                            break;
                        case DISPLAYING_VISION:
                            setDisplayingVision(timeInState);
                            break;
                        case OFF:
                            setOffCommand(timeInState);
                            break;
                        case DISPLAYING_CONFIGURE_FAULT:
                            setConfigureFault(timeInState);
                            break;
                        default:
                            System.out.println("Fell through on LED commands: " + mSystemState);
                            // setOffCommand(timeInState);
                            break;
                    }
                    mDesiredLEDState.writePixels(mCANdle);
                }
            }

            @Override
            public void onStop(double timestamp) {}
        });
    }

    private void setSuperstructureLEDCommand(double timeInState) {
        mSuperstructureLEDState.getCurrentLEDState(mDesiredLEDState, timeInState);
    }

    private void setOffCommand(double timeInState) {
        TimedLEDState.StaticLEDState.kStaticOff.getCurrentLEDState(mDesiredLEDState, timeInState);
    }

    private void setConfigureFault(double timeInState) {
        TimedLEDState.BlinkingLEDState.kConfigureFail.getCurrentLEDState(mDesiredLEDState, timeInState);
    }

    private void setDisplayingVision(double timeInState) {
        Limelight limelight = Limelight.getInstance();
        if (DriverStation.isDisabled()) {
            if (!limelight.hasTarget()) {
                // If in disabled, and don't have target show yellow rapid blink.
                TimedLEDState.BlinkingLEDState.kVisionMissing.getCurrentLEDState(mDesiredLEDState, timeInState);
            } else {
                // Otherwise, go green.
                TimedLEDState.BlinkingLEDState.kVisionPresent.getCurrentLEDState(mDesiredLEDState, timeInState);
            }
        } else {
            // If we are in auto, show when limelight goes active.
            if (limelight.getIsDisabled()) {
                TimedLEDState.StaticLEDState.kVisionDisabled.getCurrentLEDState(mDesiredLEDState, timeInState);
            } else {
                TimedLEDState.StaticLEDState.kStaticRobotZeroedWithGoodBattery.getCurrentLEDState(mDesiredLEDState, timeInState);
            }
        }
    }

    private void setNotHomedCommand (double timeInState) {
        TimedLEDState.StaticLEDState.kStaticNotHomed.getCurrentLEDState(mDesiredLEDState, timeInState);
    }

    private void setGoodBattery(double timeInState) {
        TimedLEDState.StaticLEDState.kStaticRobotZeroedWithGoodBattery.getCurrentLEDState(mDesiredLEDState, timeInState);
    }

    private void setBatteryLowCommand(double timeInState) {
        TimedLEDState.StaticLEDState.kStaticBatteryLow.getCurrentLEDState(mDesiredLEDState, timeInState);
    }

    private boolean configure_fault = false;
    public synchronized void setConfigureFault(boolean fault){
        configure_fault = fault;
    }

    private SystemState getStateTransition() {
        if (configure_fault) return SystemState.DISPLAYING_CONFIGURE_FAULT;
        switch (mWantedAction) {
            case DISPLAY_SUPERSTRUCTURE:
                return SystemState.DISPLAYING_SUPERSTRUCTURE;
            case DISPLAY_GOOD_BATTERY:
                return SystemState.DISPLAYING_GOOD_BATTERY;
            case DISPLAY_BATTERY_LOW:
                return SystemState.DISPLAYING_BATTERY_LOW; 
            case DISPLAY_NOT_HOMED:
                return SystemState.DISPLAYING_NOT_HOMED;
            case DISPLAY_VISION:
                return SystemState.DISPLAYING_VISION;
            case OFF:
                return SystemState.OFF;
            default:
                System.out.println("Fell through on LED wanted action check: " + mWantedAction);
                return SystemState.OFF;
        }
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public synchronized void outputTelemetry(boolean disabled) {}

    @Override
    public void stop() {}
}