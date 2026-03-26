package frc.robot.subsystems;

import com.revrobotics.spark.FeedbackSensor;
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
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
    public final double kIntakeArmAbsoluteUpSetpoint = 0.635;
    public final double kIntakeArmAbsoluteDownSetpoint = 0.370;

    // small green bars outside the robot
    private final SparkMax intakeMotor = new SparkMax(CanIdConstants.kIntakeCanId, MotorType.kBrushless);

    private final SparkMax intakeArm = new SparkMax(CanIdConstants.kIntakeArmCanId, MotorType.kBrushless);

    SparkClosedLoopController armController = intakeArm.getClosedLoopController();

    NTDouble intakeMotorSpeed = new NTDouble(1, "intakeMotorSpeed");

    NTDouble intakeMotorReverseSpeed = new NTDouble(-0.5, "intakeMotorReverseSpeed");
    NTDouble hopperMotorReverseSpeed = new NTDouble(-0.5, "hopperMotorReverseSpeed");

    SparkMaxConfig intakeArmConfig;
    NTDouble intakeArmP = new NTDouble(2.8, "intakeArmP");

    public IntakeSubsystem() {
        intakeMotor.configure(
                new SparkMaxConfig().smartCurrentLimit(20).inverted(true),
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        intakeArmConfig = new SparkMaxConfig();
        intakeArmConfig.smartCurrentLimit(20);

        /*
        intakeArmConfig
                .softLimit
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimitEnabled(true)
                .forwardSoftLimit(0)
                .reverseSoftLimit(-10);*/

        intakeArmConfig.absoluteEncoder.inverted(false);
        intakeArmConfig
                .closedLoop
                .outputRange(-0.2, 0.5)
                .pid(intakeArmP.get(), 0, 0)
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        intakeArm.configure(intakeArmConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        intakeArmP.subscribe((newP) -> {
            intakeArmConfig.closedLoop.p(newP);
            intakeArm.configure(intakeArmConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        });
    }

    public Command runIntake() {
        return Commands.run(() -> {
                    intakeMotor.set(intakeMotorSpeed.get());
                })
                .finallyDo(() -> {
                    intakeMotor.set(0);
                });
    }

    // needed potentially for getting fuel unstuck
    public Command runIntakeReverse() {
        return Commands.run(() -> {
                    intakeMotor.set(intakeMotorReverseSpeed.get());
                })
                .finallyDo(() -> {
                    intakeMotor.set(0);
                });
    }

    /*
    public boolean isIntakeDown() {
        if (armController.getSetpoint() == intakeArmDownPosition) {
            return true;
        }
        return false;
    }*/

    public Command intakeDown() {
        return Commands.runOnce(() -> {
            armController.setSetpoint(kIntakeArmAbsoluteDownSetpoint, ControlType.kPosition);
        });
    }

    public Command intakeUp() {
        return Commands.runOnce(() -> {
            armController.setSetpoint(kIntakeArmAbsoluteUpSetpoint, ControlType.kPosition);
        });
    }

    @Override
    public void periodic() {
        Logger.recordOutput("FuelInput/IntakeArmSetpoint", armController.getSetpoint());
        Logger.recordOutput(
                "FuelInput/IntakeArmAbsoluteEncoderPosition",
                intakeArm.getAbsoluteEncoder().getPosition());
        Logger.recordOutput(
                "FuelInput/IntakeArmEncoderPosition", intakeArm.getEncoder().getPosition());
        Logger.recordOutput("FuelInput/IntakeArmDuty", intakeArm.getAppliedOutput());
    }
}
