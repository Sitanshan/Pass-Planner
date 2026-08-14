// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.generated.TunerConstants;
import frc.robot.utils.PassPlanner; // 确保引入了你的新工具类

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 * [Refactored] 100% Native Tuner X constructors. PassPlanner injected via setter.
 */
public class CommandSwerveDrivetrain extends TunerConstants.TunerSwerveDrivetrain implements Subsystem {

    // 🚀 注入的独立战术引擎 (不再是 final，通过 setter 注入)
    public PassPlanner passPlanner = null;

    private static final double kSimLoopPeriod = 0.004; // 4 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    
    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* SysId Requests */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /* SysId routines ... */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(null, Volts.of(4), null, state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
        new SysIdRoutine.Mechanism(volts -> setControl(m_translationCharacterization.withVolts(volts)), null, this)
    );

    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(null, Volts.of(7), null, state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
        new SysIdRoutine.Mechanism(volts -> setControl(m_steerCharacterization.withVolts(volts)), null, this)
    );

    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.per(Second).of(Math.PI / 6.0), Volts.of(Math.PI), null, state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
        new SysIdRoutine.Mechanism(output -> {
            setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
            SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
        }, null, this)
    );

    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    // ==========================================
    // 💡 AutoMoveComplex 必要的通讯变量与请求
    // ==========================================
    public final SwerveRequest.Idle Idle = new SwerveRequest.Idle();
    public final SwerveRequest.RobotCentric m_safeCoastRequest = new SwerveRequest.RobotCentric();
    
    public double autoSpeedX = 0;
    public double autoSpeedY = 0;
    public double autoRot = 0;
    public boolean autoShooting = false;
    // ==========================================

    /**
     * 100% 还原的原生构造函数 1
     */
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) startSimThread();
    }

    /**
     * 100% 还原的原生构造函数 2
     */
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double odometryUpdateFrequency, SwerveModuleConstants<?, ?, ?>... modules) {
        super(driveTrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) startSimThread();
    }

    /**
     * 100% 还原的原生构造函数 3
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) startSimThread();
    }

    // ==========================================
    // 🔗 依赖注入 (Setter Injection)
    // ==========================================
    /**
     * 将外部实例化的 PassPlanner 引擎注入到底盘中，并完成双向绑定
     */
    public void setPassPlanner(PassPlanner planner) {
        this.passPlanner = planner;
        if (this.passPlanner != null) {
            this.passPlanner.setDrivetrain(this); // 双向绑定
        }
    }
    // ==========================================

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPose, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPose, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    public void addVisionMeasurement(Pose2d visionRobotPose, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(visionRobotPose, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }

    @Override
    public void periodic() {
        // 1. 处理红蓝方视角切换 (Phoenix 6 Native Perspective)
        Optional<Alliance> allianceColor = DriverStation.getAlliance();
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            if (allianceColor.isPresent()) {
                setOperatorPerspectiveForward(
                    allianceColor.get() == DriverStation.Alliance.Red ? kRedAlliancePerspectiveRotation : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            }
        }

        // 2. 将高频位置数据喂给独立的 PassPlanner 引擎更新战术雷达 (只有注入后才会运行)
        if (passPlanner != null) {
            passPlanner.updateReefRegion();
        }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();
        m_simNotifier = new Notifier(() -> {
            double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
}