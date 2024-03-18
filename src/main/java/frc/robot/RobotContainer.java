// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AmpPlaceCMD;
import frc.robot.commands.ArmPIDCMD;
import frc.robot.commands.AutoAllignCMD;
import frc.robot.commands.AutoCmd;
import frc.robot.commands.RollerCMD;
import frc.robot.commands.SpeakerCMD;
import frc.robot.commands.PlacerCMD;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Roller;
import frc.robot.subsystems.Placer;

public class RobotContainer {

  private final Arm armSubsystem = new Arm(); // Arm Subsystem
  private final Roller rollerSubsystem = new Roller(); // Intake Subsytem
  private final Placer placerSubsystem = new Placer(); // Placer Subsystem
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final AutoCmd autoCMD = new AutoCmd(placerSubsystem, drivetrain); // Auto Command

  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps*.75; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  private double TurtleModifier = .3; // Cadel

  // Cadel
  private final Pose2d AmpAllignment = new Pose2d(new Translation2d(0, 0), new Rotation2d(0));
  private final Pose2d SpeakerAllignment = new Pose2d(new Translation2d(0, 0), new Rotation2d(0));

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final Joystick equipmentController = new Joystick(1); // My 2nd joystick

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.2).withRotationalDeadband(MaxAngularRate * 0.2) // Add a 10% deadband <--Deadzone? is now 20% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press.
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    // Turtle Mode (slow).  - Cadel
    joystick.rightBumper().whileTrue(drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * TurtleModifier) // Drive forward with.
                                                                                           // negative Y (forward).
            .withVelocityY(-joystick.getLeftX() * MaxSpeed * TurtleModifier) // Drive left with negative X (left).
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate * TurtleModifier) // Drive counterclockwise with negative X (left).
    ));

    // Robot Alligns itself with Amp. - Cadel
    joystick.leftTrigger().whileTrue(new AutoAllignCMD(drivetrain, AmpAllignment));

    // Robot Alligns itself with Speaker. - Cadel
    joystick.rightTrigger().whileTrue(new AutoAllignCMD(drivetrain, SpeakerAllignment));

    /** Normal Operation Buttons */

    /** Intake */
    new JoystickButton(equipmentController, 21).whileTrue(Commands.parallel(new RollerCMD(rollerSubsystem, .7, .6),new PlacerCMD(placerSubsystem, 0.3, 0)));

    /** Moves Arm up */
    new JoystickButton(equipmentController, 22).whileTrue(new ArmPIDCMD(armSubsystem,-20));

    /** Seat Note */
    new JoystickButton(equipmentController, 19).whileTrue(new PlacerCMD(placerSubsystem, -0.24, 0));

    /** Place Speaker */
    new JoystickButton(equipmentController, 18).onTrue(new SpeakerCMD(placerSubsystem));
    
    /** Places Amp and moves arm back to loading*/
    new JoystickButton(equipmentController, 20).onTrue(new AmpPlaceCMD(placerSubsystem, armSubsystem));


    /** Troubleshooting Buttons */

    /** Moves Arm Down */
    new JoystickButton(equipmentController, 24).whileTrue(new ArmPIDCMD(armSubsystem, -1));

    /** Intake Out */
    new JoystickButton(equipmentController, 17).whileTrue(new RollerCMD(rollerSubsystem, -0.7, -0.6));




  }

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return autoCMD;
    //return Commands.print("No autonomous command configured");
  }
}
