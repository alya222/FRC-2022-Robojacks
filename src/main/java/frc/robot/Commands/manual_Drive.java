// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.RevDrivetrain;

public class manual_Drive extends CommandBase {
  private RevDrivetrain m_rDrive;
  private DoubleSupplier m_LeftSpeed;
  private DoubleSupplier m_RightSpeed;
  /** Creates a new manual_Drive. */
  public manual_Drive(DoubleSupplier leftSpeed, DoubleSupplier rightSpeed, RevDrivetrain subsytem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_LeftSpeed = leftSpeed;
    m_RightSpeed = rightSpeed;
    m_rDrive = subsytem;
    addRequirements(m_rDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //Get the joystick values and apply the Deadband
    double leftspeed = m_rDrive.deadband(m_LeftSpeed.getAsDouble(), Constants.percentDeadband);
    double rightspeed = m_rDrive.deadband(m_RightSpeed.getAsDouble(), Constants.percentDeadband);

    //update the drive train
    m_rDrive.set_tankDrive(leftspeed, rightspeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    //shut down the drive train
    m_rDrive.set_tankDrive(0,0);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
