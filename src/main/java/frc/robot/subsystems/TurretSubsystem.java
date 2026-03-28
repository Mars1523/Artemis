package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIdConstants;
import frc.robot.NTDouble;
import org.littletonrobotics.junction.Logger;

public class TurretSubsystem extends SubsystemBase {
    // private Servo turret = new Servo(1);
    private SparkMax turretMotor = new SparkMax(CanIdConstants.kTurretRotateCanId, MotorType.kBrushless);
    SparkClosedLoopController turretController;

    // absolute min is -0.302
    // absolute max is 0.793
    // go from -0.28 to 0.72?
    private static final double kMinAngle = -0.28;
    private static final double kMaxAngle = 0.72;

    NTDouble aimingTurretTolerance = new NTDouble(0.05, "AimingTurretTolerance"); // units of rotations

    SparkMaxConfig turretConfig = new SparkMaxConfig();
    // for PID constants finding
    public static double scale(double value, double minValue, double maxValue, double minOut, double maxOut) {
        return minOut + ((value - minValue) / (maxValue - minValue)) * (maxOut - minOut);
    }

    TrapezoidProfile trapezoidProfile = new TrapezoidProfile(new Constraints(30, 10));
    TrapezoidProfile.State trapezoidSetpoint = new TrapezoidProfile.State();

    public TurretSubsystem() {
        turretConfig.absoluteEncoder.zeroCentered(true);
        turretConfig.absoluteEncoder.inverted(true);
        turretConfig.encoder.positionConversionFactor(0.01111111111111).velocityConversionFactor(0.01111111111111);
        turretConfig.absoluteEncoder.positionConversionFactor(1.5);
        turretConfig.closedLoop.feedForward.kS(0.17).kV(.127);
        turretConfig.closedLoop.maxMotion.cruiseVelocity(10).maxAcceleration(10);
        turretConfig.smartCurrentLimit(20, 20);
        turretConfig.apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2);
        turretConfig
                .softLimit
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimitEnabled(true)
                .forwardSoftLimit(kMaxAngle)
                .reverseSoftLimit(kMinAngle);
        turretConfig.closedLoop.outputRange(-0.8, 0.8).pid(10, 0, 0).feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

        turretController = turretMotor.getClosedLoopController();

        turretMotor.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turretMotor.getEncoder().setPosition(turretMotor.getAbsoluteEncoder().getPosition());
    }

    private final double manualRotationFactor = 0.01;

    public void rotateTurret(double input) {
        double currAngleRotations = getTurretSetpoint().getRotations();
        double newAngleRotations = currAngleRotations + input * manualRotationFactor;
        Rotation2d newAngle = new Rotation2d(Rotations.of(newAngleRotations));
        setTurretAngle(newAngle);
    }

    public Command setTurretAngleCommand(Rotation2d angle) {
        return runOnce(() -> setTurretAngle(angle));
    }

    public void setTurretAngle(Rotation2d angle) {
        Logger.recordOutput("Turret/SetTurretAngle", angle.getDegrees());
        double turretsetpoint = angle.getRotations();
        if (turretsetpoint > kMaxAngle) {
            turretsetpoint -= 1;
        }
        if (turretsetpoint < kMinAngle) {
            turretsetpoint += 1;
        }
        turretController.setSetpoint(turretsetpoint, ControlType.kPosition);
    }

    public boolean isTurretReady() {
        return Math.abs(getEncoderError()) < aimingTurretTolerance.get();
    }

    public Rotation2d getTurretSetpoint() {
        double currentSetpoint = turretController.getSetpoint();
        return new Rotation2d(Rotations.of(currentSetpoint));
    }

    public Rotation2d getTurretAngle() {
        Angle angle = Rotations.of(turretMotor.getAbsoluteEncoder().getPosition());
        return new Rotation2d(angle.in(Radians));
    }

    public double getEncoderError() {
        return turretController.getSetpoint() - turretMotor.getAbsoluteEncoder().getPosition();
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Turret/IsTurretReady", isTurretReady());
        Logger.recordOutput(
                "Turret/TurretAngleRotations", turretMotor.getAbsoluteEncoder().getPosition());
        Logger.recordOutput("Turret/TurretSetpointRotations", turretController.getSetpoint());
        Logger.recordOutput("Turret/TurretEncoderError", getEncoderError());
    }
}
