package frc.robot.subsystems;

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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIdConstants;
import org.littletonrobotics.junction.Logger;

public class TurretSubsystem extends SubsystemBase {
    // private Servo turret = new Servo(1);
    private SwerveDriveSubsystem swerve;
    private SparkMax turretMotor = new SparkMax(CanIdConstants.kTurretRotateCanId, MotorType.kBrushless);
    SparkClosedLoopController turretController;

    // absolute min is -0.302
    // absolute max is 0.793
    // go from -0.28 to 0.72?
    private static final double kMinAngle = -0.28;
    private static final double kMaxAngle = 0.72;

    SparkMaxConfig turretConfig = new SparkMaxConfig();
    // for PID constants finding
    public static double scale(double value, double minValue, double maxValue, double minOut, double maxOut) {
        return minOut + ((value - minValue) / (maxValue - minValue)) * (maxOut - minOut);
    }

    TrapezoidProfile trapezoidProfile = new TrapezoidProfile(new Constraints(30, 10));
    TrapezoidProfile.State trapezoidSetpoint = new TrapezoidProfile.State();

    public TurretSubsystem(SwerveDriveSubsystem swerve) {
        this.swerve = swerve;

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

    // private void setServoAngle(Rotation2d angle) {
    //     var turretPercent = scale(angle.getDegrees(), -135, 135, 0, 1);
    //     turret.set(turretPercent);
    //     Logger.recordOutput("Turret/servoAngleDeg", angle.getDegrees());
    //     Logger.recordOutput("Turret/servoTurnPercent", turretPercent);
    // }

    // private void checkAlliance() {
    // if (isRed()) {
    // targetHopper = redHopper;
    // DownHome = new Translation2d(11.4, 2.5);
    // UpHome = new Translation2d(11.4, 5.5);
    // } else {
    // targetHopper = blueHopper;
    // DownHome = new Translation2d(4, 5.5);
    // UpHome = new Translation2d(4, 2.5);
    // }
    // }

    private Translation2d blueHopper = new Translation2d(4.621, 4.016);
    private Translation2d redHopper = new Translation2d(11.945, 3.990);

    Translation2d targetHopper() {
        return isRed() ? redHopper : blueHopper;
    }

    // TODO: Make functions
    // TODO: Make names better
    // Translation2d UpHome;
    // Translation2d DownHome;
    // Translation2d target;

    public Command shootAtHomeCommand() {
        return run(() -> shootAtHome());
    }

    public Command shootAtHubCommand() {
        return run(() -> shootAtHub());
    }

    private final double manualRotationFactor = 0.01;

    public void rotateTurret(double input) {
        double currAngleRotations = getTurretAngle().getRotations();
        double newAngleRotations = currAngleRotations + input * manualRotationFactor;
        Rotation2d newAngle = new Rotation2d(Rotations.of(newAngleRotations));
        setTurretAngle(newAngle);
    }

    public Command setTurretAngleCommand(Rotation2d angle) {
        return run(() -> setTurretAngle(angle));
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
        Logger.recordOutput("Turret/setpoint", turretsetpoint);
        turretController.setSetpoint(turretsetpoint, ControlType.kPosition);
    }

    public Rotation2d getTurretAngle() {
        double currentSetpoint = turretController.getSetpoint();
        return new Rotation2d(Rotations.of(currentSetpoint));
    }

    public void shootAtHub() {
        var robotFieldPosition = swerve.getPose().getTranslation();
        var robotPoseAngle = swerve.getPose().getRotation();
        var robotToHopperFieldAngle = targetHopper().minus(robotFieldPosition).getAngle();
        var robotPoseToHopperAngle = robotToHopperFieldAngle.minus(robotPoseAngle);

        Logger.recordOutput("Turret/BlueHopperPosition", blueHopper);
        Logger.recordOutput("Turret/RedHopperPosiiton", redHopper);
        Logger.recordOutput("Turret/RobotFieldPosition", robotFieldPosition);
        Logger.recordOutput("Turret/RobotPoseAngle", robotPoseAngle.getDegrees());
        Logger.recordOutput("Turret/robotToHopperFieldAngle", robotToHopperFieldAngle.getDegrees());
        Logger.recordOutput("Turret/robotPoseToHopperAngle", robotPoseToHopperAngle.getDegrees());
        // Constants.kField.;
        // turret.se(scaletarget.getDegrees());
        setTurretAngle(robotPoseToHopperAngle);
    }

    public static boolean isRed() {
        return getAlliance() == Alliance.Red;
    }

    public static Alliance getAlliance() {
        System.out.println("returning alliance: " + DriverStation.getAlliance());
        return DriverStation.getAlliance().orElse(Alliance.Blue);
    }

    Translation2d UphomeR = new Translation2d(11.4, 5.5);
    Translation2d DownhomeR = new Translation2d(11.4, 2.5);
    Translation2d UphomeB = new Translation2d(4, 5.5);
    Translation2d DownhomeB = new Translation2d(4, 2.5);
    Translation2d target;

    public void getHomeDirection(boolean RedAlliance, double yPos) {
        if (RedAlliance) {
            if (yPos > 4) {
                target = UphomeR;
            } else {
                target = DownhomeR;
            }
        } else {
            if (yPos > 4) {
                target = UphomeB;
            } else {
                target = DownhomeB;
            }
        }
    }

    public void shootAtHome() {
        Translation2d robotFieldPosition = swerve.getPose().getTranslation();
        Rotation2d robotPoseAngle = swerve.getRotation();

        getHomeDirection(isRed(), robotFieldPosition.getY());

        var robotToTargetFieldAngle = target.minus(robotFieldPosition).getAngle();
        var robotPoseToTargetAngle = robotToTargetFieldAngle.minus(robotPoseAngle);
        setTurretAngle(robotPoseToTargetAngle);

        // if(robotPoseToTargetAngle > 0){

        // }

    }

    /*
    public Command turretting() {
        return run(() -> turretController.setSetpoint(finalSetpoint, ControlType.kPosition));
    }
        */

    @Override
    public void periodic() {
        /*
        i = i.plus(Rotation2d.fromDegrees(1));
        setTurretSetpoint(i);
        Logger.recordOutput("Rotationi", i);
        Logger.recordOutput("RotationFinal", finalSetpoint);
        SmartDashboard.putNumber("ROtationI", i.getRotations());
        SmartDashboard.putNumber("ROtationFinal", finalSetpoint);
        */
        // System.err.println(".,");
    }
}
