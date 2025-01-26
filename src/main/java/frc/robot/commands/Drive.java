// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve_drive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Drive extends Command {
private final Joystick driverL;
  private final Joystick driverR;

  private final Swerve_drive driveSwerve;
  // private final BooleanSupplier robotCentricSupply;

  private final SlewRateLimiter translationLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter strafeLimiter = new SlewRateLimiter(3);

  private double strafeVal;
  private double translationVal;
  private double rotationVal;

  private boolean isAuto;

  /** Creates a new Drive. */
  public Drive(Swerve_drive driveSwerve, Joystick driverL, Joystick driverR, double strafeVal, double translationVal, double rotationVal, boolean isAuto) {
    this.driveSwerve = driveSwerve;
    this.driverL = driverL;
    this.driverR = driverR;
    this.strafeVal = strafeVal;
    this.translationVal = translationVal;
    this.rotationVal = rotationVal;
    this.isAuto = isAuto;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
