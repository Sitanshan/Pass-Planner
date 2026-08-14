// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Telemetry {

    private final double MaxSpeed;

    /* What to publish over networktables for telemetry */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /* Robot swerve drive state */
    private final NetworkTable driveStateTable = inst.getTable("DriveState");
    private final StructPublisher<Pose2d> drivePose = driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();
    private final StructPublisher<ChassisSpeeds> driveSpeeds = driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleStates = driveStateTable.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleTargets = driveStateTable.getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModulePosition> driveModulePositions = driveStateTable.getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();
    private final DoublePublisher driveTimestamp = driveStateTable.getDoubleTopic("Timestamp").publish();
    private final DoublePublisher driveOdometryFrequency = driveStateTable.getDoubleTopic("OdometryFrequency").publish();

    /* Robot pose for field positioning */
    private final NetworkTable table = inst.getTable("Pose");
    private final DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("robotPose").publish();
    private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

    /* Mechanisms to represent the swerve module states */
    private final Mechanism2d[] m_moduleMechanisms = new Mechanism2d[] {
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
    };

    /* A direction and length changing ligament for speed representation */
    private final MechanismLigament2d[] m_moduleSpeeds = new MechanismLigament2d[4];

    /* A direction changing and length constant ligament for module direction */
    private final MechanismLigament2d[] m_moduleDirections = new MechanismLigament2d[4];

    /**
     * Construct a telemetry object with the specified max speed of the robot.
     *
     * @param maxSpeed Maximum speed in meters per second
     */
    public Telemetry(double maxSpeed) {
        this.MaxSpeed = maxSpeed;
        SignalLogger.start();

        /* Set up the module state Mechanism2d telemetry */
        for (int i = 0; i < 4; ++i) {
            MechanismRoot2d rootSpeed = m_moduleMechanisms[i].getRoot("RootSpeed", 0.5, 0.5);
            m_moduleSpeeds[i] = rootSpeed.append(new MechanismLigament2d("Speed", 0.5, 0));

            MechanismRoot2d rootDirection = m_moduleMechanisms[i].getRoot("RootDirection", 0.5, 0.5);
            m_moduleDirections[i] = rootDirection.append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite)));

            SmartDashboard.putData("Module " + i, m_moduleMechanisms[i]);
        }
    }

    /**
     * Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger. 
     */
    public void telemeterize(SwerveDriveState state) {
        /* Telemeterize the swerve drive state */
        drivePose.set(state.Pose);
        // driveSpeeds.set(state.Speeds);
        // driveModuleStates.set(state.ModuleStates);
        // driveModuleTargets.set(state.ModuleTargets);
        // driveModulePositions.set(state.ModulePositions);
        // driveTimestamp.set(state.Timestamp);
        // driveOdometryFrequency.set(1.0 / state.OdometryPeriod);

        /* Also write to log file */
        // Note: CTRE Phoenix 6 Java API usually uses writeDoubleArray for logging Pose components directly
        SignalLogger.writeDoubleArray("DriveState/Pose", new double[] {
            state.Pose.getX(), state.Pose.getY(), state.Pose.getRotation().getDegrees()
        });
        
        // SignalLogger.writeDoubleArray("DriveState/Speeds", new double[] { state.Speeds.vxMetersPerSecond, state.Speeds.vyMetersPerSecond, state.Speeds.omegaRadiansPerSecond });
        // (You can use WPILib DataLogManager for structs if you uncomment other logging fields later)

        /* Telemeterize the pose to a Field2d */
        fieldTypePub.set("Field2d");
        fieldPub.set(new double[] {
            state.Pose.getX(),
            state.Pose.getY(),
            state.Pose.getRotation().getDegrees()
        });

        /* Telemeterize each module state to a Mechanism2d */
        // for (int i = 0; i < m_moduleSpeeds.length; ++i) {
        //     m_moduleDirections[i].setAngle(state.ModuleStates[i].angle.getDegrees());
        //     m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle.getDegrees());
        //     m_moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * MaxSpeed));
        // }
    }
}