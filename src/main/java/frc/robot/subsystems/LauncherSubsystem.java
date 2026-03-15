// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
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
    private final SparkMax turretFeedMotor = new SparkMax(CanIdConstants.kTurretFeedCanId, MotorType.kBrushless);

    VelocityVoltage velocityRequest = new VelocityVoltage(0);

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

    public Slot0Configs slot0ConfigsLeft = new Slot0Configs();

    // gets unstable around ~1.0, we can fine-tune this further
    public NTDouble kP = new NTDouble(0.2, "launcher/P");
    public NTDouble kI = new NTDouble(0, "launcher/I");
    public NTDouble kD = new NTDouble(0, "launcher/D");
    public NTDouble targetRps = new NTDouble(10, "launcher/targetRPS");
    public NTDouble targetDistance = new NTDouble(0, "launcher/distance");

    // public LauncherSubsystem(PhotonCameraSubsystem photon) {
    public LauncherSubsystem() {
        // this.photon = photon;

        // set left motor to clockwise leader and right motor to opposed follower (should rotate ccw)
        // set both motors to coast mode as well
        // see docs here: https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/examples/quickstart.html
        var outputConfigsLeft = new MotorOutputConfigs();
        outputConfigsLeft.NeutralMode = NeutralModeValue.Coast;
        outputConfigsLeft.Inverted = InvertedValue.Clockwise_Positive;

        // see docs here:
        // https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/basic-pid-control.html
        slot0ConfigsLeft.kS = kS;
        slot0ConfigsLeft.kV = kV;
        slot0ConfigsLeft.kA = kA;
        slot0ConfigsLeft.kP = this.kP.get();
        slot0ConfigsLeft.kI = this.kI.get();
        slot0ConfigsLeft.kD = this.kD.get();
        leftMotor.getConfigurator().apply(slot0ConfigsLeft);

        kP.subscribe((newP) -> {
            slot0ConfigsLeft.kP = newP;
            leftMotor.getConfigurator().apply(slot0ConfigsLeft);
        });
        kI.subscribe((newI) -> {
            slot0ConfigsLeft.kI = newI;
            leftMotor.getConfigurator().apply(slot0ConfigsLeft);
        });
        kD.subscribe((newD) -> {
            slot0ConfigsLeft.kD = newD;
            leftMotor.getConfigurator().apply(slot0ConfigsLeft);
        });

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

    public double rps = 0;

    public Command shootVelocityCommand() {
        return run(() -> shootVelocity(RotationsPerSecond.of(rps))).finallyDo(() -> leftMotor.set(0));
    }

    public Command shootFeed() {
        if (leftMotor.getVelocity().getValueAsDouble() > rps * 0.9
                && leftMotor.getVelocity().getValueAsDouble() < rps * 1.1) {
            return run(() -> turretFeedMotor.set(0.4));
        } else {
            return run(() -> turretFeedMotor.set(0));
        }
    }

    public void shootVelocity(AngularVelocity speed) {
        leftMotor.setControl(velocityRequest.withVelocity(speed));
    }

    public void shootDistance(double distance) {
        double rps = launcherRpsForDistance(distance);
        shootVelocity(RotationsPerSecond.of(rps));
    }

    public void shootDistance1(double distance) {
        rps = launcherRpsForDistance(distance);
    }

    // Used the Exel data sheet On discord in the programming general channel for the equation and data points
    // fit using cubic
    public double launcherRpsForDistance(double distance) {
        // returns rps

        // double[] distances = {63,89,140,168,188,203,227};
        // double[] rpsValues = {45,47,59,64,67,73,78.5};

        // To Do: make ranges for each of the distance values
        /*
        for(int i =0; i<distances.length; i++){
            if(distance == distances[i]){
                launcherRps = rpsValues[i];
                return launcherRps;
            }
        }*/

        // launcherRps = -2.0e-06 * Math.pow(distance, 3) + 0.0014 * Math.pow(distance, 2) - 0.003 * distance + 37.202;
        double a = -1.29e-06;
        double b = 9.63e-04;
        double c = 1.72e-02;
        double d = 3.99e01;
        double distanceInches = distance * 39.37;
        double launcherRps = a * Math.pow(distanceInches, 3) + b * Math.pow(distanceInches, 2) + c * distanceInches + d;
        return launcherRps;
    }

    /*public Command shootPhotonCommand() {
        return run(() -> shootPhoton())
            .finallyDo(() -> setMotorDuty(0));
    }*/

    /*public void shootPhoton() {
        double distance = targetDistance.get();
        rps = launcherRpsForDistance(Distance.ofBaseUnits(distance, distance);
        Logger.recordOutput("flywheel/speedRPS", rps);
    }*/

    @Override
    public void periodic() {
        // shootPhoton();
        // This method will be called once per scheduler run
        Logger.recordOutput(leftMotorRpsEntry, leftMotor.getVelocity().getValueAsDouble());
        Logger.recordOutput(rightMotorRpsEntry, rightMotor.getVelocity().getValueAsDouble());
        Logger.recordOutput(leftMotorPositionEntry, leftMotor.getPosition().getValueAsDouble());
        Logger.recordOutput(rightMotorPositionEntry, rightMotor.getPosition().getValueAsDouble());
        Logger.recordOutput(leftMotorVoltageEntry, leftMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput(rightMotorVoltageEntry, rightMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Launcher/LauncherRPS", launcherRpsForDistance(rps));
    }
}
