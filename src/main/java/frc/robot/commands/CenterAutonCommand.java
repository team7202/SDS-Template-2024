// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.Arms.ARMS_LOW_HIGH_SPEED;
import static frc.robot.Constants.Arms.ARMS_LOW_LOW_SPEED;
import static frc.robot.Constants.Arms.ARMS_LOW_POS;
import static frc.robot.Constants.Arms.ARMS_PICKUP_HIGH_SPEED;
import static frc.robot.Constants.Arms.ARMS_PICKUP_LOW_SPEED;
import static frc.robot.Constants.Arms.ARMS_PICKUP_POS;
import static frc.robot.Constants.Arms.ARMS_ZERO_HIGH_SPEED;
import static frc.robot.Constants.Arms.ARMS_ZERO_LOW_SPEED;
import static frc.robot.Constants.Arms.ARMS_ZERO_POS;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.commands.auto.DriveFeetCommand;
import frc.robot.commands.auto.IntakeCommand;
import frc.robot.commands.auto.ShootRingCommand;
import frc.robot.subsystems.ArmsSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeShooterSubsystem;
import frc.robot.subsystems.RingSensorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CenterAutonCommand extends SequentialCommandGroup {
  /** Creates a new AutonomousSequentialCommandGroup. */
  public CenterAutonCommand(IntakeShooterSubsystem intakeShooter, DrivetrainSubsystem driveTrain,
      ArmsSubsystem arms, RingSensorSubsystem ringSensor) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
        new MoveArmsCommand(arms, ARMS_PICKUP_POS, ARMS_PICKUP_LOW_SPEED, ARMS_PICKUP_HIGH_SPEED),
        new WaitCommand(3),
        new MoveArmsCommand(arms, ARMS_LOW_POS, ARMS_LOW_LOW_SPEED, ARMS_LOW_HIGH_SPEED),
        new WaitCommand(0.5),
        new ShootRingCommand(intakeShooter),
        new MoveArmsCommand(arms, ARMS_PICKUP_POS, ARMS_PICKUP_LOW_SPEED, ARMS_PICKUP_HIGH_SPEED),
        new WaitCommand(0.5),
        new ParallelCommandGroup(
            new IntakeCommand(intakeShooter, ringSensor),
            new DriveFeetCommand(driveTrain, 1.06, false)),
        new ParallelCommandGroup(
            new DriveFeetCommand(driveTrain, -1.03, false)),
            new SequentialCommandGroup(
                new MoveArmsCommand(arms, ARMS_PICKUP_POS, ARMS_PICKUP_LOW_SPEED, ARMS_PICKUP_HIGH_SPEED),
                new WaitCommand(3),
                new MoveArmsCommand(arms, ARMS_LOW_POS, ARMS_LOW_LOW_SPEED, ARMS_LOW_HIGH_SPEED)),
        new WaitCommand(0.5),
        new ShootRingCommand(intakeShooter),
        new InstantCommand(() -> {
          intakeShooter.setSpeed(0, 0);
          arms.setTarget(ARMS_ZERO_POS, ARMS_ZERO_LOW_SPEED, ARMS_ZERO_HIGH_SPEED);
        }),
        new DriveFeetCommand(driveTrain, 1.06, false));
  }
}
