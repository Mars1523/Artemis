// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
    SwerveDriveSubsystem swerve;
    private final SparkMax climb1 = new SparkMax(1, MotorType.kBrushless);
    private final SparkMax climb2 = new SparkMax(2, MotorType.kBrushless);
    /** Creates a new Climb. */
    public Climb(SwerveDriveSubsystem swerve) {
        this.swerve = swerve;
        SparkMaxConfig climb1Config = new SparkMaxConfig();
        SparkMaxConfig climb2Config = new SparkMaxConfig();

        climb1Config.closedLoop.p(0).i(0).d(0);
        climb1Config.closedLoop.feedForward.kS(0).kV(0).kA(0);
        climb1Config.closedLoop.maxMotion.cruiseVelocity(0).maxAcceleration(0);
        climb1Config.smartCurrentLimit(40, 40);
        climb2Config.follow(climb1);
        climb1.configure(climb1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        climb2.configure(climb2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
