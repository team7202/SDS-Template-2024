// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.Arms.ARMS_PICKUP_HIGH_SPEED;
import static frc.robot.Constants.Arms.ARMS_PICKUP_LOW_SPEED;
import static frc.robot.Constants.Arms.ARMS_PICKUP_POS;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.DriveFeetCommand;
import frc.robot.commands.auto.ShootRingCommand;
import frc.robot.subsystems.ArmsSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SideAutonCommand extends SequentialCommandGroup {
  /** Creates a new AutonomousSequentialCommandGroup. */
  public SideAutonCommand(IntakeShooterSubsystem intakeShooter, DrivetrainSubsystem driveTrain,
      ArmsSubsystem arms) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
        // new MoveArmCommand(arms, -9.75, 0.25, 0.25),
        new WaitCommand(0.5),
        new ShootRingCommand(intakeShooter),
        new MoveArmsCommand(arms, ARMS_PICKUP_POS, ARMS_PICKUP_LOW_SPEED, ARMS_PICKUP_HIGH_SPEED),
        new WaitCommand(0.5),
        new DriveFeetCommand(driveTrain, 3.5, false));
  }
}
