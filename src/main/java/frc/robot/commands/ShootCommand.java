// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmsSubsystem;
import frc.robot.subsystems.IntakeShooterSubsystem;

public class ShootCommand extends Command {

  private final IntakeShooterSubsystem m_intakeShooterSubsystem;
  private final ArmsSubsystem m_arms;

  private final Timer timer;

  /** Creates a new ShootCommand. */
  public ShootCommand(IntakeShooterSubsystem intakeShooterSubsystem, ArmsSubsystem armsSubsystem) {
    addRequirements(intakeShooterSubsystem);
    this.m_intakeShooterSubsystem = intakeShooterSubsystem;
    this.m_arms = armsSubsystem;
    this.timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] positions = m_arms.getPositions();
    double speedTarget = positions[0] >= -4 || positions[1] >= 0 ? -0.3 : -0.9;
    
    if (timer.get() < 0.5)
      m_intakeShooterSubsystem.setSpeed(speedTarget, 0);
    else
      m_intakeShooterSubsystem.setSpeed(speedTarget, 0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.timer.stop();
  }
}
