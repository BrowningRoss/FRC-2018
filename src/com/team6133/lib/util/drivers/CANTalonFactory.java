package com.team6133.lib.util.drivers;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.MotorSafety;


/**
 * Creates CANTalon objects and configures all the parameters we care about to
 * factory defaults. Closed-loop and sensor parameters are not set, as these are
 * expected to be set by the application.
 */
public class CANTalonFactory {

    private static final Configuration kDefaultConfiguration = new Configuration();
    private static final Configuration kSlaveConfiguration = new Configuration();

    static {
        kSlaveConfiguration.CONTROL_FRAME_PERIOD_MS = 1000;
        kSlaveConfiguration.MOTION_CONTROL_FRAME_PERIOD_MS = 1000;
        kSlaveConfiguration.GENERAL_STATUS_FRAME_RATE_MS = 1000;
        kSlaveConfiguration.FEEDBACK_STATUS_FRAME_RATE_MS = 1000;
        kSlaveConfiguration.QUAD_ENCODER_STATUS_FRAME_RATE_MS = 1000;
        kSlaveConfiguration.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 1000;
        kSlaveConfiguration.PULSE_WIDTH_STATUS_FRAME_RATE_MS = 1000;
    }

    // Create a CANTalon with the default (out of the box) configuration.
    public static WPI_TalonSRX createDefaultTalon(int id) {
        return createTalon(id, kDefaultConfiguration);
    }

    public static WPI_TalonSRX createTalon(int id, Configuration config) {
        WPI_TalonSRX talon = new LazyCANTalon(id);
        talon.setControlFramePeriod(ControlFrame.Control_3_General, config.CONTROL_FRAME_PERIOD_MS);
        talon.set(ControlMode.PercentOutput, 0);
        talon.changeMotionControlFramePeriod(config.MOTION_CONTROL_FRAME_PERIOD_MS);
        talon.clearMotionProfileHasUnderrun(config.TIMEOUT_MS);
        talon.clearMotionProfileTrajectories();
        talon.clearStickyFaults(config.TIMEOUT_MS);
        talon.configForwardSoftLimitEnable(config.ENABLE_FORWARD_SOFT_LIMIT, config.TIMEOUT_MS);
        talon.configVoltageCompSaturation(config.MAX_OUTPUT_VOLTAGE, config.TIMEOUT_MS);
        talon.configNominalOutputForward(config.NOMINAL_VOLTAGE, config.TIMEOUT_MS);
        talon.configNominalOutputReverse(-config.NOMINAL_VOLTAGE, config.TIMEOUT_MS);
        talon.configPeakOutputForward(config.PEAK_VOLTAGE, config.TIMEOUT_MS);
        talon.configPeakOutputReverse(-config.PEAK_VOLTAGE, config.TIMEOUT_MS);
        talon.configReverseSoftLimitEnable(config.ENABLE_REVERSE_SOFT_LIMIT, config.TIMEOUT_MS);
        talon.enableCurrentLimit(config.ENABLE_CURRENT_LIMIT);
        talon.setNeutralMode(config.BRAKE_OR_COAST);
        talon.setSensorPhase(false);
        talon.configContinuousCurrentLimit(config.CONTINUOUS_CURRENT_LIMIT, config.TIMEOUT_MS);
        talon.configPeakCurrentLimit(config.PEAK_CURRENT_LIMIT, config.TIMEOUT_MS);
        talon.configPeakCurrentDuration(config.PEAK_CURRENT_TIMEOUT_MS, config.TIMEOUT_MS);
        talon.setExpiration(config.EXPIRATION_TIMEOUT_SECONDS);
        talon.configForwardSoftLimitThreshold(config.FORWARD_SOFT_LIMIT, config.TIMEOUT_MS);
        talon.setInverted(config.INVERTED);
        talon.setSelectedSensorPosition(0, 0, 0);
        talon.selectProfileSlot(0, 0);
        talon.configReverseSoftLimitThreshold(config.REVERSE_SOFT_LIMIT, config.TIMEOUT_MS);
        talon.setSafetyEnabled(config.SAFETY_ENABLED);
        talon.configVelocityMeasurementPeriod(config.VELOCITY_MEASUREMENT_PERIOD, config.TIMEOUT_MS);
        talon.configVelocityMeasurementWindow(config.VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW, config.TIMEOUT_MS);
        talon.configOpenloopRamp(config.OPEN_LOOP_RAMP_RATE, config.TIMEOUT_MS);
        talon.configClosedloopRamp(config.CLOSED_LOOP_RAMP_RATE, config.TIMEOUT_MS);
        talon.configForwardLimitSwitchSource(config.FORWARDLIMIT_SWITCH_TYPE, config.FORWARD_LIMIT_SWITCH_NORMALLY_OPEN, config.TIMEOUT_MS);
        talon.configReverseLimitSwitchSource(config.REVERSE_LIMIT_SWITCH_TYPE, config.REVERSE_LIMIT_SWITCH_NORMALLY_OPEN, config.TIMEOUT_MS);

        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, config.GENERAL_STATUS_FRAME_RATE_MS, config.TIMEOUT_MS);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, config.FEEDBACK_STATUS_FRAME_RATE_MS, config.TIMEOUT_MS);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, config.QUAD_ENCODER_STATUS_FRAME_RATE_MS, config.TIMEOUT_MS);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat,
                config.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS, config.TIMEOUT_MS);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, config.PULSE_WIDTH_STATUS_FRAME_RATE_MS, config.TIMEOUT_MS);

        return talon;
    }

    /**
     * Configuration class that manages all of the default configurations for the Talon SRX.
     */
    public static class Configuration {
        public LimitSwitchNormal FORWARD_LIMIT_SWITCH_NORMALLY_OPEN = LimitSwitchNormal.Disabled;
        public LimitSwitchSource FORWARDLIMIT_SWITCH_TYPE = LimitSwitchSource.Deactivated;
        public LimitSwitchNormal REVERSE_LIMIT_SWITCH_NORMALLY_OPEN = LimitSwitchNormal.Disabled;
        public LimitSwitchSource REVERSE_LIMIT_SWITCH_TYPE = LimitSwitchSource.Deactivated;

        public double MAX_OUTPUT_VOLTAGE = 12;
        public double NOMINAL_VOLTAGE = 0.0;
        public double PEAK_VOLTAGE = 1.0;
        public boolean ENABLE_CURRENT_LIMIT = false;
        public boolean ENABLE_FORWARD_SOFT_LIMIT = false;
        public boolean ENABLE_REVERSE_SOFT_LIMIT = false;
        public int CONTINUOUS_CURRENT_LIMIT = 0;
        public int PEAK_CURRENT_LIMIT = 0;
        public int PEAK_CURRENT_TIMEOUT_MS = 613;
        public double EXPIRATION_TIMEOUT_SECONDS = MotorSafety.DEFAULT_SAFETY_EXPIRATION;
        public int FORWARD_SOFT_LIMIT = 0;
        public boolean INVERTED = false;
        public int REVERSE_SOFT_LIMIT = 0;
        public boolean SAFETY_ENABLED = false;
        public NeutralMode BRAKE_OR_COAST = NeutralMode.Brake;

        public int CONTROL_FRAME_PERIOD_MS = 5;
        public int MOTION_CONTROL_FRAME_PERIOD_MS = 100;
        public int GENERAL_STATUS_FRAME_RATE_MS = 5;
        public int FEEDBACK_STATUS_FRAME_RATE_MS = 100;
        public int QUAD_ENCODER_STATUS_FRAME_RATE_MS = 100;
        public int ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 100;
        public int PULSE_WIDTH_STATUS_FRAME_RATE_MS = 100;
        public int TIMEOUT_MS = 10;

        public VelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD = VelocityMeasPeriod.Period_100Ms;
        public int VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 64;

        public double OPEN_LOOP_RAMP_RATE = 0;
        public double CLOSED_LOOP_RAMP_RATE = 0;
    }

    /*
     * Run this on a fresh talon to produce good values for the defaults.

     public static String getFullTalonInfo(WPI_TalonSRX talon) {
     StringBuilder sb = new StringBuilder().append("isRevLimitSwitchClosed = ")
     .append(talon.isRevLimitSwitchClosed()).append("\n").append("getBusVoltage = ")
     .append(talon.getBusVoltage()).append("\n").append("isForwardSoftLimitEnabled = ")
     .append(talon.isForwardSoftLimitEnabled()).append("\n").append("getFaultRevSoftLim = ")
     .append(talon.getFaultRevSoftLim()).append("\n").append("getStickyFaultOverTemp = ")
     .append(talon.getStickyFaultOverTemp()).append("\n").append("isZeroSensorPosOnFwdLimitEnabled = ")
     .append(talon.isZeroSensorPosOnFwdLimitEnabled()).append("\n")
     .append("getMotionProfileTopLevelBufferCount = ").append(talon.getMotionProfileTopLevelBufferCount())
     .append("\n").append("getNumberOfQuadIdxRises = ").append(talon.getNumberOfQuadIdxRises()).append("\n")
     .append("getInverted = ").append(talon.getInverted()).append("\n")
     .append("getPulseWidthRiseToRiseUs = ").append(talon.getPulseWidthRiseToRiseUs()).append("\n")
     .append("getError = ").append(talon.getError()).append("\n").append("isSensorPresent = ")
     .append(talon.isSensorPresent(CANTalon.FeedbackDevice.CtreMagEncoder_Relative)).append("\n")
     .append("isControlEnabled = ").append(talon.isControlEnabled()).append("\n").append("getTable = ")
     .append(talon.getTable()).append("\n").append("isEnabled = ").append(talon.isEnabled()).append("\n")
     .append("isZeroSensorPosOnRevLimitEnabled = ").append(talon.isZeroSensorPosOnRevLimitEnabled())
     .append("\n").append("isSafetyEnabled = ").append(talon.isSafetyEnabled()).append("\n")
     .append("getOutputVoltage = ").append(talon.getOutputVoltage()).append("\n").append("getTemperature = ")
     .append(talon.getTemperature()).append("\n").append("getSmartDashboardType = ")
     .append(talon.getSmartDashboardType()).append("\n").append("getPulseWidthPosition = ")
     .append(talon.getPulseWidthPosition()).append("\n").append("getOutputCurrent = ")
     .append(talon.getOutputCurrent()).append("\n").append("get = ").append(talon.get()).append("\n")
     .append("isZeroSensorPosOnIndexEnabled = ").append(talon.isZeroSensorPosOnIndexEnabled()).append("\n")
     .append("getMotionMagicCruiseVelocity = ").append(talon.getMotionMagicCruiseVelocity()).append("\n")
     .append("getStickyFaultRevSoftLim = ").append(talon.getStickyFaultRevSoftLim()).append("\n")
     .append("getFaultRevLim = ").append(talon.getFaultRevLim()).append("\n").append("getEncPosition = ")
     .append(talon.getEncPosition()).append("\n").append("getIZone = ").append(talon.getIZone()).append("\n")
     .append("getAnalogInPosition = ").append(talon.getAnalogInPosition()).append("\n")
     .append("getFaultUnderVoltage = ").append(talon.getFaultUnderVoltage()).append("\n")
     .append("getCloseLoopRampRate = ").append(talon.getCloseLoopRampRate()).append("\n")
     .append("toString = ").append(talon.toString()).append("\n")
     // .append("getMotionMagicActTrajPosition =
     // ").append(talon.getMotionMagicActTrajPosition()).append("\n")
     .append("getF = ").append(talon.getF()).append("\n").append("getClass = ").append(talon.getClass())
     .append("\n").append("getAnalogInVelocity = ").append(talon.getAnalogInVelocity()).append("\n")
     .append("getI = ").append(talon.getI()).append("\n").append("isReverseSoftLimitEnabled = ")
     .append(talon.isReverseSoftLimitEnabled()).append("\n").append("getPIDSourceType = ")
     .append(talon.getPIDSourceType()).append("\n").append("getEncVelocity = ")
     .append(talon.getEncVelocity()).append("\n").append("GetVelocityMeasurementPeriod = ")
     .append(talon.GetVelocityMeasurementPeriod()).append("\n").append("getP = ").append(talon.getP())
     .append("\n").append("GetVelocityMeasurementWindow = ").append(talon.GetVelocityMeasurementWindow())
     .append("\n").append("getDeviceID = ").append(talon.getDeviceID()).append("\n")
     .append("getStickyFaultRevLim = ").append(talon.getStickyFaultRevLim()).append("\n")
     // .append("getMotionMagicActTrajVelocity =
     // ").append(talon.getMotionMagicActTrajVelocity()).append("\n")
     .append("getReverseSoftLimit = ").append(talon.getReverseSoftLimit()).append("\n").append("getD = ")
     .append(talon.getD()).append("\n").append("getFaultOverTemp = ").append(talon.getFaultOverTemp())
     .append("\n").append("getForwardSoftLimit = ").append(talon.getForwardSoftLimit()).append("\n")
     .append("GetFirmwareVersion = ").append(talon.GetFirmwareVersion()).append("\n")
     .append("getLastError = ").append(talon.getLastError()).append("\n").append("isAlive = ")
     .append(talon.isAlive()).append("\n").append("getPinStateQuadIdx = ").append(talon.getPinStateQuadIdx())
     .append("\n").append("getAnalogInRaw = ").append(talon.getAnalogInRaw()).append("\n")
     .append("getFaultForLim = ").append(talon.getFaultForLim()).append("\n").append("getSpeed = ")
     .append(talon.getSpeed()).append("\n").append("getStickyFaultForLim = ")
     .append(talon.getStickyFaultForLim()).append("\n").append("getFaultForSoftLim = ")
     .append(talon.getFaultForSoftLim()).append("\n").append("getStickyFaultForSoftLim = ")
     .append(talon.getStickyFaultForSoftLim()).append("\n").append("getClosedLoopError = ")
     .append(talon.getClosedLoopError()).append("\n").append("getSetpoint = ").append(talon.getSetpoint())
     .append("\n").append("isMotionProfileTopLevelBufferFull = ")
     .append(talon.isMotionProfileTopLevelBufferFull()).append("\n").append("getDescription = ")
     .append(talon.getDescription()).append("\n").append("hashCode = ").append(talon.hashCode()).append("\n")
     .append("isFwdLimitSwitchClosed = ").append(talon.isFwdLimitSwitchClosed()).append("\n")
     .append("getPinStateQuadA = ").append(talon.getPinStateQuadA()).append("\n")
     .append("getPinStateQuadB = ").append(talon.getPinStateQuadB()).append("\n").append("GetIaccum = ")
     .append(talon.GetIaccum()).append("\n").append("getFaultHardwareFailure = ")
     .append(talon.getFaultHardwareFailure()).append("\n").append("pidGet = ").append(talon.pidGet())
     .append("\n").append("getBrakeEnableDuringNeutral = ").append(talon.getBrakeEnableDuringNeutral())
     .append("\n").append("getStickyFaultUnderVoltage = ").append(talon.getStickyFaultUnderVoltage())
     .append("\n").append("getPulseWidthVelocity = ").append(talon.getPulseWidthVelocity()).append("\n")
     .append("GetNominalClosedLoopVoltage = ").append(talon.GetNominalClosedLoopVoltage()).append("\n")
     .append("getPosition = ").append(talon.getPosition()).append("\n").append("getExpiration = ")
     .append(talon.getExpiration()).append("\n").append("getPulseWidthRiseToFallUs = ")
     .append(talon.getPulseWidthRiseToFallUs()).append("\n")
     // .append("createTableListener =
     // ").append(talon.createTableListener()).append("\n")
     .append("getControlMode = ").append(talon.getControlMode()).append("\n")
     .append("getMotionMagicAcceleration = ").append(talon.getMotionMagicAcceleration()).append("\n")
     .append("getControlMode = ").append(talon.getControlMode());

     return sb.toString();
     }
     */
}