// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import java.util.EnumSet;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

public class LauncherSubsystemSparkMax extends SubsystemBase {

    // pid terms determined using REV Hardware Client 2
    // basically try to get flywheel to match setpoint RPM quickly without oscillating
    public static final double kP = .0001;
    public static final double kI = 0.00000001;
    public static final double kD = 0.01;
    public static final double vMax = 5500;
    public static final double aMax = 2000;

    // feedforward terms determined using sysid, see below
    public static final double kS = 0.16075;
    public static final double kV = 0.0021313;
    // public static final double kA = 0.00057038;
    public static final double kA = 0;

    PhotonCameraSubsystem photon;

    SparkMax motor1 = new SparkMax(58, MotorType.kBrushless);
    SparkMax motor2 = new SparkMax(7, MotorType.kBrushless);

    public static final String motor1RpmEntry = "Launcher/Motor1Rpm";
    public static final String motor2RpmEntry = "Launcher/Motor2Rpm";

    double targetRpm = 2000;

    // feedforward constants
    // SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.16075, 0.0021313, 0.00057038);
    SparkClosedLoopController motorController;

    public LauncherSubsystemSparkMax(PhotonCameraSubsystem photon) {
        this.photon = photon;

        SparkMaxConfig motor1Config = new SparkMaxConfig();
        SparkMaxConfig motor2Config = new SparkMaxConfig();

        motor1Config.closedLoop.p(kP).i(kI).d(kD);
        motor1Config.closedLoop.feedForward.kS(kS).kV(kV).kA(kA);
        motor1Config.closedLoop.maxMotion.cruiseVelocity(vMax).maxAcceleration(aMax);
        motorController = motor1.getClosedLoopController();
        motor1Config.inverted(true);

        motor1Config.encoder.quadratureAverageDepth(6);
        motor1Config.encoder.quadratureMeasurementPeriod(6);
        motor2Config.encoder.quadratureAverageDepth(6);
        motor2Config.encoder.quadratureMeasurementPeriod(6);

        motor1Config.smartCurrentLimit(40, 40);
        motor2Config.smartCurrentLimit(40, 40);

        motor2Config.follow(motor1, true);

        motor1.configure(motor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motor2.configure(motor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // uncomment if running sysid to determine feedforward constants
        // initializeSysId();

        var entry = NetworkTableInstance.getDefault().getEntry("/Tune/ShooterRpm");
        entry.setDouble(targetRpm);
        NetworkTableInstance.getDefault().addListener(entry, EnumSet.of(NetworkTableEvent.Kind.kValueRemote), e -> {
            targetRpm = e.valueData.value.getDouble();
            System.out.println("Updated shooter rpm to " + targetRpm);
        });
    }

    /**
     * sysid is the routine for calibrating feedforward constants
     * info about code setup: https://docs.advantagekit.org/data-flow/sysid-compatibility/
     * more general info about sysid: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/creating-routine.html
     * used AdvantageScope for storing and exporting wpilog data, and SysId 2026 for fitting constants
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

    public void shootRpm(double rpm) {
        Logger.recordOutput("flywheel/MotorControllerRPM", rpm);
        motorController.setSetpoint(rpm, ControlType.kMAXMotionVelocityControl);
    }

    public Command shootVolts(Voltage volts) {
        return run(() -> setMotorVolts(volts)).finallyDo(() -> setMotorVolts(Volts.of(0)));
    }

    public Command shootPercent(double duty) {
        return run(() -> setMotorDuty(duty)).finallyDo(() -> setMotorDuty(0));
    }

    public Command shootFF() {
        return run(() -> shootRpm(targetRpm)).finallyDo(() -> setMotorDuty(0));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        Logger.recordOutput(motor1RpmEntry, motor1.getEncoder().getVelocity());
        Logger.recordOutput(motor2RpmEntry, motor2.getEncoder().getVelocity());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public void setMotorVolts(Voltage speed) {
        motor1.setVoltage(speed.in(Volts));
    }

    public void setMotorDuty(double speed) {
        motor1.set(speed);
    }

    static final double[] getTargetPoseLL() {
        if (LimelightHelpers.getTV(Constants.kLimelightCameraName)) {
            double[] pose = LimelightHelpers.getTargetPose_CameraSpace(Constants.kLimelightCameraName);
            return pose;
        } else {
            return null;
        }
    }

    public void shoot() {
        if (getTargetPoseLL() == null) {
            // Change to odometry but for now:
            setMotorDuty(0);
        } else {
            double rpm = getTargetPoseLL()[2];
            shootRpm(rpm * 500);
        }
    }

    public void shootPhoton() {
        var result = photon.getLastResult();
        PhotonTrackedTarget target = result.getBestTarget();

        Logger.recordOutput("flywheel/photonresults", result);
        if (!result.hasTargets() || target == null) {
            Logger.recordOutput("flywheel/hasTarget", false);
            setMotorDuty(0);
        } else {
            Logger.recordOutput("flywheel/hasTarget", true);

            Transform3d bestCameratoTarget = target.getBestCameraToTarget();
            double distance = bestCameratoTarget.getTranslation().getNorm();

            double height = bestCameratoTarget.getZ();
            double speed = Math.sqrt((9.8 * Math.pow(distance, 2))
                    / (2 * Math.pow(Math.cos(Math.PI / 3), 2) * (distance * Math.tan(Math.PI / 3) - (height - 0.7))));
            Logger.recordOutput("flywheel/speedMpS", speed);
            double rpm = ((speed / (2 * Math.PI * 0.076)) * 120) / 0.7;
            Logger.recordOutput("flywheel/speedRPM", rpm);

            shootRpm(rpm);
        }
    }
}