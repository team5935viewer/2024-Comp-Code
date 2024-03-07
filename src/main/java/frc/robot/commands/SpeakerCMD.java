// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Placer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SpeakerCMD extends SequentialCommandGroup {
  /** Creates a new SpeakerCMD. */
  public SpeakerCMD(Placer placer) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new InstantCommand(() -> new PlacerCMD(placer, 0, 1)).andThen(
    new WaitCommand(.5)).andThen(
    new InstantCommand(() -> new PlacerCMD(placer, .3, 1))).andThen(
    new WaitCommand(1)).andThen(
    new InstantCommand(() -> placer.Stop())));
  }
}