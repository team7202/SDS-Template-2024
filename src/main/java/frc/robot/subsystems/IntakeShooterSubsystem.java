// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.IntakeShooter.CLOSED_LOOP_RAMP_RATE;
import static frc.robot.Constants.IntakeShooter.INTAKE_MOTOR;
import static frc.robot.Constants.IntakeShooter.LEFT_SHOOTER_MOTOR;
import static frc.robot.Constants.IntakeShooter.RIGHT_SHOOTER_MOTOR;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeShooterSubsystem extends SubsystemBase {

  public double shooterSpeed = 0.0;
  public double intakeSpeed = 0.0;

  private final CANSparkMax leftShooter;
  private final CANSparkMax rightShooter;
  private final CANSparkMax intake;

  /** Creates a new IntakeShooterSubsystem. */
  public IntakeShooterSubsystem() {
    this.leftShooter = new CANSparkMax(LEFT_SHOOTER_MOTOR, MotorType.kBrushless);
    this.rightShooter = new CANSparkMax(RIGHT_SHOOTER_MOTOR, MotorType.kBrushless);
    this.leftShooter.setClosedLoopRampRate(CLOSED_LOOP_RAMP_RATE);
    this.rightShooter.setClosedLoopRampRate(CLOSED_LOOP_RAMP_RATE);
    this.intake = new CANSparkMax(INTAKE_MOTOR, MotorType.kBrushless);
  }

  public void setSpeed(double shooterSpeed, double intakeSpeed) {
    this.intakeSpeed = intakeSpeed;
    this.shooterSpeed = shooterSpeed;
  }

  @Override
  public void periodic() {
    this.leftShooter.set(shooterSpeed);
    this.rightShooter.set(shooterSpeed);
    this.intake.set(intakeSpeed);
  }
}
