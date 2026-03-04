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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIdConstants;
import frc.robot.NTDouble;

public class LauncherSubsystem extends SubsystemBase {

    // PhotonCameraSubsystem photon;

    TalonFX leftMotor = new TalonFX(CanIdConstants.kLeftShooterCanId);
    TalonFX rightMotor = new TalonFX(CanIdConstants.kRightShooterCanId);

    public static final String leftMotorRpmEntry = "Launcher/LeftMotorRpm";
    public static final String rightMotorRpmEntry = "Launcher/RightMotorRpm";

    NTDouble speed = new NTDouble(0.1, "LauncherDuty");

    // feedforward constants
    // SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.16075, 0.0021313, 0.00057038);
    SparkClosedLoopController motorController;

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
    }

    public Command shootDuty() {
        return run(() -> setMotorDuty(speed.get())).finallyDo(() -> setMotorDuty(0));
    }

    public void setMotorDuty(double speed) {
        leftMotor.set(speed);
    }
}
