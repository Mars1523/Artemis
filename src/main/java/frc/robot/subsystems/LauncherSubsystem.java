// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.CanIdConstants;
import frc.robot.NTDouble;
import org.littletonrobotics.junction.Logger;

public class LauncherSubsystem extends SubsystemBase {

    // PhotonCameraSubsystem photon;

    TalonFX leftMotor = new TalonFX(CanIdConstants.kLeftShooterCanId);
    TalonFX rightMotor = new TalonFX(CanIdConstants.kRightShooterCanId);

    // note Talon uses rotations/sec
    public static final String leftMotorRpsEntry = "Launcher/LeftMotorRps";
    public static final String rightMotorRpsEntry = "Launcher/RightMotorRps";
    public static final String leftMotorPositionEntry = "Launcher/LeftMotorPosition";
    public static final String rightMotorPositionEntry = "Launcher/RightMotorPosition";
    public static final String leftMotorVoltageEntry = "Launcher/LeftMotorVoltage";
    public static final String rightMotorVoltageEntry = "Launcher/RightMotorVoltage";

    NTDouble speed = new NTDouble(0.1, "LauncherDuty");

    // feedforward constants
    // SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.16075, 0.0021313, 0.00057038);
    SparkClosedLoopController motorController;

    // feedforward terms determined using sysid, see below
    public static final double kS = 0.069104;
    public static final double kV = 0.10922;
    // public static final double kA = 0.0067792;
    public static final double kA = 0;

    // public LauncherSubsystem(PhotonCameraSubsystem photon) {
    public LauncherSubsystem() {
        // this.photon = photon;

        // set left motor to clockwise leader and right motor to opposed follower (should rotate ccw)
        // set both motors to coast mode as well
        // see docs here: https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/examples/quickstart.html
        var outputConfigsLeft = new MotorOutputConfigs();
        outputConfigsLeft.NeutralMode = NeutralModeValue.Coast;
        outputConfigsLeft.Inverted = InvertedValue.Clockwise_Positive;
        leftMotor.getConfigurator().apply(outputConfigsLeft);
        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), MotorAlignmentValue.Opposed));

        var outputConfigsRight = new MotorOutputConfigs();
        outputConfigsRight.NeutralMode = NeutralModeValue.Coast;
        rightMotor.getConfigurator().apply(outputConfigsRight);

        // smart current limits at 40 A
        // see docs here:
        // https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html
        var limitConfigs = new CurrentLimitsConfigs();
        limitConfigs.StatorCurrentLimit = 40;
        limitConfigs.StatorCurrentLimitEnable = true;
        leftMotor.getConfigurator().apply(limitConfigs);
        rightMotor.getConfigurator().apply(limitConfigs);

        // uncomment if running sysid to determine feedforward constants
        initializeSysId();
    }

    public void setMotorVolts(Voltage speed) {
        leftMotor.setVoltage(speed.in(Units.Volts));
    }

    /**
     * sysid is the routine for calibrating feedforward constants
     * info about code setup: https://docs.advantagekit.org/data-flow/sysid-compatibility/
     * more general info about sysid: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/creating-routine.html
     * used Elastic to control sysid routine
     *      - note: add widget for "Launcher", need to enable robot, click each button
     * used AdvantageScope for storing and exporting wpilog data
     *      - note: when exporting use wpilog format and AdvantageKit Cycles for timestamps
     * used SysId 2026 for fitting constants
     * note that we needed to log position, velocity, and voltage manually in periodic(),
     *      since Talon motors don't get saved with URCL
     * should be recalibrated if anything is changed on the Launcher
     */
    private void initializeSysId() {
        SysIdRoutine routine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null, (state) -> Logger.recordOutput("sysid-state", state.toString())),
                new SysIdRoutine.Mechanism((volts) -> setMotorVolts(volts), null, this));

        SmartDashboard.putData("Launcher/QuasiForward", routine.quasistatic(SysIdRoutine.Direction.kForward));
        SmartDashboard.putData("Launcher/QuasiReverse", routine.quasistatic(SysIdRoutine.Direction.kReverse));
        SmartDashboard.putData("Launcher/DynamicForward", routine.dynamic(SysIdRoutine.Direction.kForward));
        SmartDashboard.putData("Launcher/DynamicReverse", routine.dynamic(SysIdRoutine.Direction.kReverse));
    }

    public Command shootDuty() {
        return run(() -> setMotorDuty(speed.get())).finallyDo(() -> setMotorDuty(0));
    }

    public void setMotorDuty(double speed) {
        leftMotor.set(speed);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        Logger.recordOutput(leftMotorRpsEntry, leftMotor.getVelocity().getValueAsDouble());
        Logger.recordOutput(rightMotorRpsEntry, rightMotor.getVelocity().getValueAsDouble());
        Logger.recordOutput(leftMotorPositionEntry, leftMotor.getPosition().getValueAsDouble());
        Logger.recordOutput(rightMotorPositionEntry, rightMotor.getPosition().getValueAsDouble());
        Logger.recordOutput(leftMotorVoltageEntry, leftMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput(rightMotorVoltageEntry, rightMotor.getMotorVoltage().getValueAsDouble());
    }
}
