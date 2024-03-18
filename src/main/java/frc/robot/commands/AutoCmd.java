// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.subsystems.Placer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoCmd extends SequentialCommandGroup {
  /** Creates a new AutoCmd. */
  public AutoCmd(Placer placer, CommandSwerveDrivetrain drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    //Command runAuto = drivetrain.getAutoPath("Simple Taxi");
    addCommands(
  
    new PlacerCMD(placer, 0, 1).withTimeout(.25), 
    new PlacerCMD(placer, 0.4, 1).withTimeout(1),
    AutoBuilder.followPath(PathPlannerPath.fromPathFile("Simple Taxi"))
    );
  }
}
