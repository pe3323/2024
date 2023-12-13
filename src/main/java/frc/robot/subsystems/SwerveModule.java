package frc.robot.subsystems;


import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;

    private final AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;
    private final int driveId;
    private final int turnId;
    private final int absoluteId;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.driveId = driveMotorId;
        this.turnId = turningMotorId;
        this.absoluteId = absoluteEncoderId;

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new AnalogInput(absoluteEncoderId);

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);
        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                resetEncoders();
            } catch (Exception e) {
            }
        }).start();
       
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        //SmartDashboard.putNumber("Turning Position " + id, turningEncoder.getPosition());
        //SmartDashboard.putNumber("Turning Encoder radians T: " + turnId + " A: " + absoluteId , turningEncoder.getPosition());
        return turningEncoder.getPosition();
        
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad() {

        // Returns revolutions of the motor
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        //SmartDashboard.putNumber("Voltage from RoboRio Angle of T: " + turnId + " A: " + absoluteId, angle);

        // Converts the number of revolutions by 2PI in order to convert from revolutions to radians
        angle *= 2.0 * Math.PI;
        //SmartDashboard.putNumber("Voltage converted to Radians Angle of T: " + turnId + " A: " + absoluteId, angle);

        // Subtracts the radian offset value for each swerve module from the angle in radians to arrive at our true radian value
        angle -= absoluteEncoderOffsetRad;
        //SmartDashboard.putNumber("Absolute Encoder Radians T: " + turnId + " A: " + absoluteId ,angle * (absoluteEncoderReversed ? -1.0 : 1.0));

        // Returns the angle as a positive or negative value depending on the boolean value of absoluteEncoderReversed
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Target Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
        SmartDashboard.putString("Current Swerve[" + absoluteEncoder.getChannel() + "] state", getState().toString());
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}