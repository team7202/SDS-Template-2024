// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeShooterSubsystem;
import frc.robot.subsystems.RingSensorSubsystem;

public class IntakeCommand extends Command {

  private final IntakeShooterSubsystem m_intakeShooter;
  private final RingSensorSubsystem m_ringSensor;
  private final Timer timer;

  private boolean finished = false;

  private final Debouncer m_debouncer = new Debouncer(0.00001, Debouncer.DebounceType.kBoth);

  /** Creates a new IntakeCommand. */
  public IntakeCommand(IntakeShooterSubsystem intakeShooterSubsystem, RingSensorSubsystem ringSensorSubsystem) {
    addRequirements(intakeShooterSubsystem, ringSensorSubsystem);
    this.m_intakeShooter = intakeShooterSubsystem;
    this.m_ringSensor = ringSensorSubsystem;
    this.timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    SmartDashboard.putBoolean("INTAKE ENDED", false);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_debouncer.calculate(m_ringSensor.ringDetected()))
      this.finished = true;
    else
      m_intakeShooter.setSpeed(0, 0.6);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    m_intakeShooter.setSpeed(0, 0);
    SmartDashboard.putBoolean("INTAKE ENDED", true);
    System.out.println("INTAKE ENDED");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
