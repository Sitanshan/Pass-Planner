// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoMoveComplex extends Command {

    private final CommandSwerveDrivetrain m_drivetrain;

    private double[] xArray;
    private double[] yArray;
    private double[] dArray; 
    private double[] speedArray;
    private final boolean stopAtEnd;
    
    private final boolean invertA;
    private final boolean invertD;

    private int currentIndex = 0;
    private Pose2d m_targetWaypoint = new Pose2d();
    private double angleToTarget = 0.0;
    private Pose2d finalWaypoint = new Pose2d();
    private double targetRot=0;
    private final PIDController m_stopPID;
    private final boolean aimAtReef;

    private final SwerveRequest.FieldCentricFacingAngle driveClosed = new SwerveRequest.FieldCentricFacingAngle();

    private static final double kTranslationTolerance = 0.03; 
    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond);
    private boolean aimatsupply=false;
    public AutoMoveComplex(CommandSwerveDrivetrain drivetrain, double[] xArray, double[] yArray, 
                           double[] dArray, double[] speedArray, 
                           boolean stopAtEnd, boolean invert1, boolean invert2,
                           double p, double i, double d) {
        
        this.m_drivetrain = drivetrain;
        this.stopAtEnd = stopAtEnd;
        this.invertA = invert1;
        this.invertD = invert2;
        this.m_stopPID = new PIDController(p, i, d);

        if (xArray == null) {
            this.aimAtReef = true;
        } else if (xArray.length != yArray.length || xArray.length != speedArray.length || xArray.length != dArray.length || xArray.length == 0) {
            throw new IllegalArgumentException("AutoMoveComplex: All arrays must have the exact same non-zero length!");
        } else {
            this.aimAtReef = false;
            this.xArray = xArray.clone();
            this.yArray = yArray.clone();
            this.dArray = dArray.clone();
            this.speedArray = speedArray.clone();
        }

        driveClosed.HeadingController.setPID(8.0, 0.0, 0.1);
        driveClosed.withDeadband(MaxSpeed * 0.01)
                   .withRotationalDeadband(0.1)
                   .withDriveRequestType(DriveRequestType.Velocity)
                   .withSteerRequestType(SteerRequestType.Position);
        
        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        currentIndex = 0;
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        boolean isRed = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
        
        // 🚨 安全提升：加上 clone()，彻底防止修改数组时污染底层引用
        if (aimAtReef) {
            this.xArray = m_drivetrain.passPlanner.dynamicXArray.clone();
            this.yArray = m_drivetrain.passPlanner.dynamicYArray.clone();
            this.speedArray = m_drivetrain.passPlanner.dynamicSpeedArray.clone();
            this.dArray = new double[this.xArray.length];
        }

        if (this.xArray == null || this.xArray.length == 0) {
            System.err.println("⚠️ AutoMoveComplex 警告：接收到空数组，当场结束！");
            this.finalWaypoint = m_drivetrain.getState().Pose;
            return;
        }

        boolean doInvertA = invertA || (aimAtReef && isRed);
        boolean doInvertD = invertD || (aimAtReef && isRed);

        for (int i = 0; i < xArray.length; i++) {
            if (doInvertD) {
                dArray[i] = 0.0 - dArray[i];
                yArray[i] = 8.05 - yArray[i];
            }
            if (doInvertA) {
                dArray[i] = 180.0 - dArray[i];
                while (dArray[i] > 180.0) dArray[i] -= 360.0;
                while (dArray[i] < -180.0) dArray[i] += 360.0;
                xArray[i] = 17.55 - xArray[i];
            }
            if (isRed) {
                dArray[i] -= 180.0;
            }
        }
        
        finalWaypoint = new Pose2d(xArray[xArray.length-1], yArray[yArray.length-1], Rotation2d.fromDegrees(dArray[dArray.length-1]));
    
        m_stopPID.reset();
        updateSegmentState(m_drivetrain.getState().Pose);
        aimatsupply=m_drivetrain.passPlanner.aimAtLeftSupply||m_drivetrain.passPlanner.aimAtRightSupply;
        if(m_drivetrain.passPlanner.aimAtLeftSupply){
            targetRot=-53;
            m_drivetrain.passPlanner.aimAtLeftSupply=false;
        }
        else if(m_drivetrain.passPlanner.aimAtRightSupply){
            targetRot=53;
            m_drivetrain.passPlanner.aimAtRightSupply=false;
        }
        else if(aimAtReef){
            targetRot = getFaceCenterAngle(m_drivetrain.passPlanner.targetReefID) + 180.0;
        }
    }

    @Override
    public void execute() {
        if (xArray == null || xArray.length == 0 || currentIndex >= xArray.length) return;

        Pose2d currentPose = m_drivetrain.getState().Pose; 

        double distToAbsoluteFinal = currentPose.getTranslation().getDistance(finalWaypoint.getTranslation());
        if (distToAbsoluteFinal <= 0.05) {
            currentIndex = xArray.length;
            return;
        }

        double minDistance = Double.MAX_VALUE;
        int closestUpcomingIndex = currentIndex;
        
        for (int i = currentIndex; i < xArray.length; i++) {
            double distToPt = currentPose.getTranslation().getDistance(new Translation2d(xArray[i], yArray[i]));
            if (distToPt < minDistance) {
                minDistance = distToPt;
                closestUpcomingIndex = i;
            }
        }
        
        if (closestUpcomingIndex != currentIndex) {
            currentIndex = closestUpcomingIndex;
            updateSegmentState(currentPose);
        }

        int lookaheadSteps = 2; 
        int targetIndex = Math.min(currentIndex + lookaheadSteps, xArray.length - 1);
        Pose2d lookaheadWaypoint = new Pose2d(xArray[targetIndex], yArray[targetIndex], Rotation2d.fromDegrees(dArray[targetIndex]));
        Rotation2d angleNow = lookaheadWaypoint.getTranslation().minus(currentPose.getTranslation()).getAngle();

        double realTargetVel = speedArray[currentIndex]; 

       if (stopAtEnd) {
            // 1. 先计算当前物理位置到下一个目标点的直线距离
            double distToNextWaypoint = currentPose.getTranslation().getDistance(
                new Translation2d(xArray[currentIndex], yArray[currentIndex])
            );
            
            double remainingArcLength = distToNextWaypoint;

            // 2. 🌟 架构师级修正：精确积分后续所有路径段的真实物理欧氏距离！
            for (int i = currentIndex; i < xArray.length - 1; i++) {
                double dx = xArray[i+1] - xArray[i];
                double dy = yArray[i+1] - yArray[i];
                remainingArcLength += Math.hypot(dx, dy);
            }

            // 3. 将极其精确的剩余弧长扔给 PID 引擎
            double pidOut = Math.abs(m_stopPID.calculate(-remainingArcLength, 0));
            realTargetVel = Math.min(speedArray[currentIndex], pidOut);
            
            // 4. 兜底保护：只要还没到容差范围，绝不让速度降为 0 导致死锁
            if (remainingArcLength > kTranslationTolerance && realTargetVel < 0.1) {
                realTargetVel = 0.1;
            }
        }

        double suppX = realTargetVel * angleNow.getCos();
        double suppY = realTargetVel * angleNow.getSin();
        
       // double targetRot = 0.0;
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        boolean isRed = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
        
        if (!aimAtReef&&!aimatsupply) {
            
            targetRot = m_drivetrain.autoShooting ? m_drivetrain.autoRot : dArray[currentIndex];
        }

        if (isRed) {
            suppX = -suppX;
            suppY = -suppY;
        }
        
        m_drivetrain.autoSpeedX = suppX;
        m_drivetrain.autoSpeedY = suppY;
        
        m_drivetrain.setControl(
            driveClosed.withVelocityX(suppX)
                       .withVelocityY(suppY)
                       .withTargetDirection(Rotation2d.fromDegrees(targetRot)) 
        );
    }

    @Override
    public boolean isFinished() {
        if (xArray == null || xArray.length == 0) return true;

        Pose2d currentPose = m_drivetrain.getState().Pose;
        double distToFinal = currentPose.getTranslation().getDistance(finalWaypoint.getTranslation());
        return currentIndex >= xArray.length || distToFinal <= 0.05;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted && !DriverStation.isAutonomousEnabled()) {
            m_drivetrain.setControl(
                m_drivetrain.m_safeCoastRequest.withVelocityX(0.0).withVelocityY(0.0)
            );
        } else {
            if (stopAtEnd) {
                m_drivetrain.setControl(m_drivetrain.Idle);
            }
        }
    }

    private void updateSegmentState(Pose2d currentPose) {
        if (currentIndex >= xArray.length) return;
        m_targetWaypoint = new Pose2d(xArray[currentIndex], yArray[currentIndex], Rotation2d.fromDegrees(dArray[currentIndex]));
        Translation2d nowTrans = currentPose.getTranslation();
        
        angleToTarget = m_targetWaypoint.getTranslation().minus(nowTrans).getAngle().getDegrees();
        if (angleToTarget < 0) {
            angleToTarget += 360.0;
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