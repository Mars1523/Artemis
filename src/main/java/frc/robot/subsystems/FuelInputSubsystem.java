package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIdConstants;
import frc.robot.NTDouble;

public class FuelInputSubsystem extends SubsystemBase {
    // small green bars outside the robot
    private final SparkMax intakeMotor = new SparkMax(CanIdConstants.kIntakeCanId, MotorType.kBrushless);

    // red bars inside the robot
    private final SparkMax hopperMotor = new SparkMax(CanIdConstants.kHopperCanId, MotorType.kBrushless);

    // blue motor under the turret
    private final SparkMax turretFeedMotor = new SparkMax(CanIdConstants.kTurretFeedCanId, MotorType.kBrushless);

    private final SparkMax intakeArm = new SparkMax(CanIdConstants.kIntakeArmCanId, MotorType.kBrushless);

    public double intakeArmUpPosition = 0;
    public double intakeArmDownPosition = -10;

    SparkClosedLoopController armController = intakeArm.getClosedLoopController();

    NTDouble intakeMotorSpeed = new NTDouble(0.5, "intakeMotorSpeed");
    NTDouble hopperMotorSpeed = new NTDouble(0.5, "hopperMotorSpeed");
    NTDouble turretFeedMotorSpeed = new NTDouble(0.8, "turretFeedMotorSpeed");

    NTDouble intakeMotorReverseSpeed = new NTDouble(-0.5, "intakeMotorReverseSpeed");
    NTDouble hopperMotorReverseSpeed = new NTDouble(-0.5, "hopperMotorReverseSpeed");
    NTDouble turretFeedMotorReverseSpeed = new NTDouble(-0.8, "turretFeedMotorReverseSpeed");

    public FuelInputSubsystem() {
        intakeMotor.configure(
                new SparkMaxConfig().smartCurrentLimit(20).inverted(true),
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        hopperMotor.configure(
                new SparkMaxConfig().smartCurrentLimit(20).inverted(true),
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        var intakeArmConfig = new SparkMaxConfig();
        intakeArmConfig.smartCurrentLimit(20);

        intakeArmConfig
                .softLimit
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimitEnabled(true)
                .forwardSoftLimit(0)
                .reverseSoftLimit(-10);

        intakeArmConfig.closedLoop.outputRange(-0.2, 0.2).pid(0.07, 0, 0);
        intakeArm.configure(intakeArmConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command runIntake() {
        return Commands.run(() -> {
                    intakeMotor.set(intakeMotorSpeed.get());
                    hopperMotor.set(hopperMotorSpeed.get());
                    turretFeedMotor.set(turretFeedMotorSpeed.get());
                })
                .finallyDo(() -> {
                    intakeMotor.set(0);
                    hopperMotor.set(0);
                    turretFeedMotor.set(0);
                });
    }

    // needed potentially for getting fuel unstuck
    public Command runIntakeReverse() {
        return Commands.run(() -> {
                    intakeMotor.set(intakeMotorReverseSpeed.get());
                    hopperMotor.set(hopperMotorReverseSpeed.get());
                    turretFeedMotor.set(turretFeedMotorReverseSpeed.get());
                })
                .finallyDo(() -> {
                    intakeMotor.set(0);
                    hopperMotor.set(0);
                    turretFeedMotor.set(0);
                });
    }

    public boolean isIntakeDown() {
        if (armController.getSetpoint() == intakeArmDownPosition) {
            return true;
        }
        return false;
    }

    public Command intakeDown() {
        return Commands.runOnce(() -> {
            armController.setSetpoint(intakeArmDownPosition, ControlType.kPosition);
        });
    }

    public Command intakeUp() {
        return Commands.runOnce(() -> {
            armController.setSetpoint(intakeArmUpPosition, ControlType.kPosition);
        });
    }
}
