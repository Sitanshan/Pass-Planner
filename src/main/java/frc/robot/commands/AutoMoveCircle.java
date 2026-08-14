// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoMoveCircle extends Command {

    private final CommandSwerveDrivetrain m_drivetrain;

    // 几何与参数
    private final double angleOff;
    private double m_Cx;
    private double m_Cy;
    private final double m_radius;
    private double m_targetAngleDeg;
    private final double m_targetVel;
    private double m_targetHeadingDeg;
    private boolean m_isCCW;
    private final boolean m_stopAtEnd;
    private final boolean m_faceTravelDir;
    
    // 🚨 架构师修正：去掉 final，以便在 initialize 里根据红蓝方动态覆写！
    private boolean m_invertA;
    private boolean m_invertD;
    
    private final boolean aimAtReef;
    
    private int m_direction; // 1 (CCW) 或 -1 (CW)

    // 角度积分与终点
    private Rotation2d m_lastAngleFromCenter = new Rotation2d();
    private double m_accumulatedDegrees = 0.0;
    private double m_totalDegreesNeeded = 0.0;
    private Translation2d m_targetPoint = new Translation2d(); // 物理终点坐标

    // PID 控制器
    private final PIDController m_radialPidX = new PIDController(2.0, 0.1, 0.1);
    private final PIDController m_radialPidY = new PIDController(2.0, 0.1, 0.1);
    private final PIDController m_distPid = new PIDController(3.0, 0.1, 0.1);    

    private final SwerveRequest.FieldCentricFacingAngle driveClosed = new SwerveRequest.FieldCentricFacingAngle();
    private int targetID=-1;
    // ==========================================
    // 🏗️ Constructor (静态指令模式)
    // ==========================================
    public AutoMoveCircle(CommandSwerveDrivetrain drivetrain, 
                          double centerX, double centerY, double radius, 
                          double targetAngleDeg, boolean isCCW, 
                          double targetVel, boolean stopAtEnd, 
                          double targetHeadingDeg, boolean faceTravelDir, 
                          boolean invertA, boolean invertD, double angleOffset) {
        
        this.m_drivetrain = drivetrain;
        this.m_Cx = centerX;
        this.m_Cy = centerY;
        this.m_radius = radius;
        this.m_targetAngleDeg = targetAngleDeg;
        this.m_isCCW = isCCW;
        this.m_targetVel = targetVel;
        this.m_stopAtEnd = stopAtEnd;
        this.m_targetHeadingDeg = targetHeadingDeg;
        this.m_faceTravelDir = faceTravelDir;
        this.m_invertA = invertA;
        this.m_invertD = invertD;
        this.angleOff = angleOffset;
        
        this.aimAtReef = false; 

        driveClosed.HeadingController.setPID(8.0, 0.0, 0.1);
        driveClosed.withDeadband(0.05)
                   .withRotationalDeadband(0.1)
                   .withDriveRequestType(DriveRequestType.Velocity)
                   .withSteerRequestType(SteerRequestType.Position);

        addRequirements(m_drivetrain);
    }

    // ==========================================
    // 🚀 Constructor (动态 Reef 轨道模式)
    // ==========================================
    public AutoMoveCircle(CommandSwerveDrivetrain drivetrain, double radius, double targetVel, boolean stopAtEnd) {
        this.m_drivetrain = drivetrain;
        this.m_radius = radius;
        this.m_targetVel = targetVel;
        this.m_stopAtEnd = stopAtEnd;
        
        this.aimAtReef = true; 

        // 默认垃圾值
        this.m_Cx = 0; this.m_Cy = 0; this.m_targetAngleDeg = 0;
        this.m_isCCW = false; this.m_targetHeadingDeg = 0;
        this.m_faceTravelDir = false; this.angleOff = 0;
        
        this.m_invertA = false; 
        this.m_invertD = false;

        driveClosed.HeadingController.setPID(8.0, 0.0, 0.1);
        driveClosed.withDeadband(0.05)
                   .withRotationalDeadband(0.1)
                   .withDriveRequestType(DriveRequestType.Velocity)
                   .withSteerRequestType(SteerRequestType.Position);

        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        boolean isRed = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;

        if (aimAtReef) {
            // ==========================================
            // 🎯 动态解算：基于蓝方基准生成
            // ==========================================
            if(m_drivetrain.passPlanner.supplyProxyId==1){
                targetID=1;
                m_drivetrain.passPlanner.supplyProxyId=0;
            }
            else if(m_drivetrain.passPlanner.supplyProxyId==5){
                targetID=5;
                m_drivetrain.passPlanner.supplyProxyId=0;
            }
            else{
                targetID=m_drivetrain.passPlanner.targetReefID;
            }
            m_Cx = 4.489;
            m_Cy = 4.026;
            
            int step = m_drivetrain.passPlanner.calcShortestReefPath(targetID);
            
            if (Math.abs(step) <= 1) {
                m_totalDegreesNeeded = 0; 
                return;
            }

            m_isCCW = (step < 0);
            
            int adjacentID = (step > 0) ? (targetID - 1 + 6) % 6 : (targetID+ 1) % 6;
            m_targetAngleDeg = getFaceCenterAngle(adjacentID)+(m_isCCW ? -0 : 0);

            // 🚨 极其保险：和 AutoMoveClosed 一样，给翻转参数注入灵魂！
            m_invertA = isRed;
            m_invertD = isRed;
        }

        // ==========================================
        // 🏗️ 原生镜像：把坐标和角度真实地平移到红方场地！
        // ==========================================
        if (m_invertD) {
            m_Cy = OperatorConstants.fieldWidth - m_Cy;
            m_targetAngleDeg = -m_targetAngleDeg;
            m_targetHeadingDeg = -m_targetHeadingDeg;
            m_isCCW = !m_isCCW; 
        }
        if (m_invertA) {
            m_Cx = OperatorConstants.fieldHeight - m_Cx;
            m_targetAngleDeg = 180.0 - m_targetAngleDeg;
            m_targetHeadingDeg = 180.0 - m_targetHeadingDeg;
            m_isCCW = !m_isCCW; 
        }
        
        m_direction = m_isCCW ? 1 : -1;

        // 此时的 m_Cx 和 m_Cy 已经是绝对真实的物理坐标了 (无需再偷偷映射到蓝方)
        Translation2d currentTrans = m_drivetrain.getState().Pose.getTranslation();
        m_lastAngleFromCenter = currentTrans.minus(new Translation2d(m_Cx, m_Cy)).getAngle();
        m_accumulatedDegrees = 0.0;

        // 计算行程角度
        double startDeg = m_lastAngleFromCenter.getDegrees();
        double diff = m_targetAngleDeg - startDeg;
        
        if (m_isCCW) {
            while (diff < 0) diff += 360.0;
            while (diff >= 360.0) diff -= 360.0;
        } else {
            diff = -diff;
            while (diff < 0) diff += 360.0;
            while (diff >= 360.0) diff -= 360.0;
        }
        m_totalDegreesNeeded = diff;
        
        // 生成真实绝对终点坐标
        double targetRad = Math.toRadians(m_targetAngleDeg);
        m_targetPoint = new Translation2d(m_Cx, m_Cy).plus(new Translation2d(
            m_radius * Math.cos(targetRad),
            m_radius * Math.sin(targetRad)
        ));
        
        m_distPid.reset();
    }

    @Override
    public void execute() {
        if (m_totalDegreesNeeded == 0) return; 

        // 这里的 currentTrans 和 centerTrans 处于绝对现实坐标系！
        Translation2d currentTrans = m_drivetrain.getState().Pose.getTranslation();
        Translation2d centerTrans = new Translation2d(m_Cx, m_Cy);
        Rotation2d currentAngleFromCenter = currentTrans.minus(centerTrans).getAngle();

        // 1. 角度积分
        double deltaDeg = currentAngleFromCenter.minus(m_lastAngleFromCenter).getDegrees();
        m_accumulatedDegrees += deltaDeg * m_direction; 
        m_lastAngleFromCenter = currentAngleFromCenter;

        double thetaRad = currentAngleFromCenter.getRadians();

        // 2. 原生 PID 纠偏
        Translation2d idealPoint = centerTrans.plus(new Translation2d(
            m_radius * Math.cos(thetaRad), 
            m_radius * Math.sin(thetaRad)
        ));

        double v_radial_x = m_radialPidX.calculate(currentTrans.getX(), idealPoint.getX());
        double v_radial_y = m_radialPidY.calculate(currentTrans.getY(), idealPoint.getY());

        // 3. 理论距离减速
        double totalArcLen = m_totalDegreesNeeded * Math.PI / 180.0 * m_radius;
        double currentArcLen = m_accumulatedDegrees * Math.PI / 180.0 * m_radius;
        double base_v_tangent = m_targetVel;

        if (m_stopAtEnd) {
            base_v_tangent = m_distPid.calculate(currentArcLen, totalArcLen);
        }
        base_v_tangent = MathUtil.clamp(base_v_tangent, -m_targetVel, m_targetVel);

        // 4. 原生切线向量
        double tx = -Math.sin(thetaRad);
        double ty =  Math.cos(thetaRad);
        double v_tangent_x = tx * m_direction * base_v_tangent;
        double v_tangent_y = ty * m_direction * base_v_tangent;

        // 5. 决定车头朝向 (Heading)
        Rotation2d currentHeading;
        if (aimAtReef) {
            // 无论红蓝，原生角度加上180就是车头指向圆心！
            currentHeading = currentAngleFromCenter.plus(Rotation2d.fromDegrees(180.0));
        } else if (m_faceTravelDir) {
            double an = Math.atan2(ty * m_direction, tx * m_direction);
            currentHeading = Rotation2d.fromRadians(an + angleOff * m_direction);
        } else {
            currentHeading = Rotation2d.fromDegrees(m_targetHeadingDeg);
        }

        // 6. 最终速度合成
        double suppX = v_tangent_x + v_radial_x;
        double suppY = v_tangent_y + v_radial_y;
        double current_mag = Math.hypot(suppX, suppY);

        if (current_mag > 1e-6) {
            suppX = (suppX / current_mag) * base_v_tangent;
            suppY = (suppY / current_mag) * base_v_tangent;
        } else {
            suppX = 0.0;
            suppY = 0.0;
        }
        
        // ==========================================
        // 7. 抵消 Phoenix 6 自动反转 (和 AutoMoveClosed 逻辑完全同步！)
        // ==========================================
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        boolean isRed = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
        if (isRed) {
            suppX = -suppX;
            suppY = -suppY;
            // 完美对齐 AutoMoveClosed 中的 "this.d -= 180" 逻辑！
            currentHeading = currentHeading.plus(Rotation2d.fromDegrees(180.0));
        }
        
        if (m_drivetrain.autoShooting) {
            currentHeading = Rotation2d.fromDegrees(m_drivetrain.autoRot);
        }
        
        // 8. 发送给底盘
        m_drivetrain.autoSpeedX = suppX;
        m_drivetrain.autoSpeedY = suppY;
        m_drivetrain.setControl(driveClosed
            .withVelocityX(suppX)
            .withVelocityY(suppY)
            .withTargetDirection(currentHeading)
        );
    }

    @Override
    public boolean isFinished() {
        if (m_totalDegreesNeeded == 0) return true; 
        
        // 这里的 m_targetPoint 已经是真实的红方物理点，直接算直线距离即可！
        Translation2d currentTrans = m_drivetrain.getState().Pose.getTranslation();
        double distToTarget = currentTrans.getDistance(m_targetPoint);

        boolean isPositionReached = (distToTarget < 0.15) && (m_accumulatedDegrees >= m_totalDegreesNeeded * 0.5);
        boolean isAngleReached = m_accumulatedDegrees >= m_totalDegreesNeeded;

        return isPositionReached || isAngleReached;
    }

    @Override
    public void end(boolean interrupted) {
        if (m_stopAtEnd || (interrupted && DriverStation.isAutonomousEnabled())) {
            m_drivetrain.setControl(
                m_drivetrain.m_safeCoastRequest
                            .withVelocityX(0.0)
                            .withVelocityY(0.0)
            );
        } else {
            m_drivetrain.setControl(m_drivetrain.Idle); 
        }
    }

    private double getFaceCenterAngle(int faceID) {
        switch (faceID) {
            case 0: return 180.0;
            case 1: return 120.0;
            case 2: return 60.0;
            case 3: return 0.0;
            case 4: return 300.0;
            case 5: return 240.0;
            default: return 0.0;
        }
    }
}