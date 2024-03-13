// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmsSubsystem;

public class MoveArmsCommand extends Command {

  private final ArmsSubsystem m_arms;
  private final double m_target;
  private final double m_lowSpeed;
  private final double m_highSpeed;
  
  /** Creates a new MoveArmsCommand. */
  public MoveArmsCommand(ArmsSubsystem arms, double target, double lowSpeed, double highSpeed) {
    addRequirements(arms);
    m_arms = arms;
    m_target = target;
    m_lowSpeed = lowSpeed;
    m_highSpeed = highSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("MOVE ARMS CALLED: " + m_target);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arms.setTarget(m_target, m_lowSpeed, m_highSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
        System.out.println("MOVE ARMS ENDED: " + m_target);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_arms.pid.atSetpoint();
  }
}
