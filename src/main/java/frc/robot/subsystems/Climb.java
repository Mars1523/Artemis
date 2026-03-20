// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.NTDouble;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
    NTDouble climbSpeed = new NTDouble(.6, "ClimbSpeedUp");

    SwerveDriveSubsystem swerve;
    private final SparkMax climb1 = new SparkMax(61, MotorType.kBrushless);
    private final SparkMax climb2 = new SparkMax(62, MotorType.kBrushless);
    // private SparkClosedLoopController climbController;
    /** Creates a new Climb. */
    public Climb() {
        SparkMaxConfig climb1Config = new SparkMaxConfig();
        SparkMaxConfig climb2Config = new SparkMaxConfig();

        climb1Config.softLimit.reverseSoftLimit(0).reverseSoftLimitEnabled(true);
        climb2Config.softLimit.reverseSoftLimit(0).reverseSoftLimitEnabled(true);
        // climb1Config.inverted(true);
        // climb2Config.inverted(false);
        climb1Config.smartCurrentLimit(40, 40);
        climb2Config.smartCurrentLimit(40, 40);
        climb2Config.follow(climb1, true);
        climb1.configure(climb1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        climb2.configure(climb2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // climbController = climb1.getClosedLoopController();
        // climb1.getEncoder().setPosition(0);
        // climb2.getEncoder().setPosition(0);
    }

    public void moveArm(double speed) {
        climb1.set(speed);
    }

    public Command armUpCommand() {
        return run(() -> moveArm(climbSpeed.get())).finallyDo(() -> moveArm(0));
    }

    public Command armDownCommand() {
        return run(() -> moveArm(-climbSpeed.get())).finallyDo(() -> moveArm(0));
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Climb/motor1Duty", climb1.getAppliedOutput());
        Logger.recordOutput("Climb/motor2Duty", climb2.getAppliedOutput());
        // This method will be called once per scheduler run
    }
}
