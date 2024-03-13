// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmsSubsystem;
import frc.robot.subsystems.IntakeShooterSubsystem;

public class AmpCommand extends Command {

  // private final ArmsSubsystem arms;
  private final IntakeShooterSubsystem intakeShooter;

  /** Creates a new AmpCommand. */
  public AmpCommand(ArmsSubsystem arms, IntakeShooterSubsystem intakeShooter) {
    addRequirements(arms, intakeShooter);
    // this.arms = arms;
    this.intakeShooter = intakeShooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.intakeShooter.setSpeed(intakeShooter.shooterSpeed, 0);
    // this.arms.setTarget(4, 0.25, 0.25);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (Math.abs(this.arms.getSpeeds()[0]) < 0.05 || Math.abs(this.arms.getSpeeds()[1]) < 0.05) {
    //   this.intakeShooter.setSpeed(-0.4, intakeShooter.intakeSpeed);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // this.arms.setArms(0);
    // this.intakeShooter.setSpeed(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
