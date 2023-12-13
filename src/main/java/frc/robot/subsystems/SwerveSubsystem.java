package frc.robot.subsystems;


import java.util.Arrays;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);


    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final SwerveDriveOdometry odometer;

    public SwerveSubsystem() {

        // SwerveModulePosition pos = new SwerveModulePosition();
        SwerveModulePosition []initpos = new SwerveModulePosition[] { new SwerveModulePosition(frontLeft.getDrivePosition(), getInitRotation2d(frontLeft)), new SwerveModulePosition(frontRight.getDrivePosition(), getInitRotation2d(frontRight)), 
            new SwerveModulePosition(backLeft.getDrivePosition(), getInitRotation2d(backLeft)), new SwerveModulePosition(backRight.getDrivePosition(), getInitRotation2d(backRight))};

        odometer =  new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
            // get rotation2d maybe instead of manually setting it? Is the Gyro Relative?
           gyro.getRotation2d(), initpos);
        
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
                backRight.resetEncoders();
                backLeft.resetEncoders();
                frontLeft.resetEncoders();
                frontRight.resetEncoders();
                stopModules();
            } catch (Exception e) {
            }
        }).start();
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Rotation2d getInitRotation2d(SwerveModule swrvMod) {
        return Rotation2d.fromRadians(swrvMod.getAbsoluteEncoderRad());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(),null, pose);
        //figure out swerve module position
    }

    @Override
    public void periodic() {

       
        // Used for Odometry purposes only, does not affect Teleop

        SwerveModulePosition lf = new SwerveModulePosition(frontLeft.getDrivePosition(), new Rotation2d(frontLeft.getTurningPosition() *2 * Math.PI));
        SwerveModulePosition rf = new SwerveModulePosition(frontRight.getDrivePosition(), new Rotation2d(frontRight.getTurningPosition() * 2 * Math.PI));
        SwerveModulePosition lb = new SwerveModulePosition(backLeft.getDrivePosition(), new Rotation2d(backLeft.getTurningPosition() * 2 * Math.PI));
        SwerveModulePosition rb = new SwerveModulePosition(backRight.getDrivePosition(), new Rotation2d(backRight.getTurningPosition() * 2 * Math.PI));

        odometer.update(getRotation2d(), 
            new SwerveModulePosition[]{
                lf, rf, lb, rb
            });

            SmartDashboard.putNumber("Gyro:", getHeading());
            SmartDashboard.putNumber("Left Front Swerve: ", frontLeft.getTurningPosition());
            SmartDashboard.putNumber("Right Front Swerve: ", frontRight.getTurningPosition());
            SmartDashboard.putNumber("Left Back Swerve: ", backLeft.getTurningPosition());
            SmartDashboard.putNumber("Right Back Swerve: ", backRight.getTurningPosition());

            SmartDashboard.putNumber("Left Front Swerve Absolute: ", frontLeft.getAbsoluteEncoderRad());
            SmartDashboard.putNumber("Right Front Swerve Absolute: ", frontRight.getAbsoluteEncoderRad());
            SmartDashboard.putNumber("Left Back Swerve Absolute: ", backLeft.getAbsoluteEncoderRad());
            SmartDashboard.putNumber("Right Back Swerve Absolute: ", backRight.getAbsoluteEncoderRad());

            
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }


    public void setWheelState( ){

    }
}