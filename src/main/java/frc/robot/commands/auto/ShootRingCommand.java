// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeShooterSubsystem;

public class ShootRingCommand extends Command {

  private final IntakeShooterSubsystem m_intakeShooter;
  private final Timer timer;

  private boolean finished = false;

  /** Creates a new ShootRingCommand. */
  public ShootRingCommand(IntakeShooterSubsystem intakeShooterSubsystem) {
    addRequirements(intakeShooterSubsystem);
    this.m_intakeShooter = intakeShooterSubsystem;
    this.timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
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
    if (timer.get() < 0.5)
      m_intakeShooter.setSpeed(-1, 0);
    else if (timer.get() < 1.5)
      m_intakeShooter.setSpeed(-1, 0.5);
    else
      this.finished = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeShooter.setSpeed(0, 0);
    timer.stop();
    System.out.println("SHOT RING");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.finished;
  }
}
