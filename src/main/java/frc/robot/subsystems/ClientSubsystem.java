package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
// 引入 WPILib 网络服务器包，用于 Elastic Dashboard 的布局下载
import edu.wpi.first.net.WebServer; 


public class ClientSubsystem extends SubsystemBase {

    // 重制赛 (REBUILT) 比赛回合和射击权定义
    public enum MatchPhase {
        kAuto,        // 自动阶段 (计时器 0:20 – 0:00，均激活)
        kTransition,  // 过渡阶段 (计时器 2:20 – 2:10，均激活)
        kSwitch1,     // 切换时段1 (计时器 2:10 – 1:45，单方激活)
        kSwitch2,     // 切换时段2 (计时器 1:45 – 1:20，单方激活)
        kSwitch3,     // 切换时段3 (计时器 1:20 – 0:55，单方激活)
        kSwitch4,     // 切换时段4 (计时器 0:55 – 0:30，单方激活)
        kEndgame,     // 最​​终阶段 (计时器 0:30 – 0:00，均激活)
        kUnknown
    }

    private final NetworkTable m_table;
    private final SendableChooser<Boolean> m_firstAttackerChooser = new SendableChooser<>();
    private final Field2d m_field = new Field2d();
    
    // Teleop切换阶段中，我方是否先激活 (由Auto分数决定)
    private boolean is_our_turn_first = true; 

    private final CommandSwerveDrivetrain m_drivesubsystem;

    public ClientSubsystem(CommandSwerveDrivetrain driveSubsystem) {
        this.m_drivesubsystem = driveSubsystem;
        
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        this.m_table = inst.getTable("client");

        // 设置在 Dashboard 上让操作手选择谁最先射击
        m_firstAttackerChooser.setDefaultOption("OUR Alliance First", true);
        m_firstAttackerChooser.addOption("OPPONENT Alliance First", false);
        SmartDashboard.putData("First Shoot Setting", m_firstAttackerChooser);

        SmartDashboard.putData("Field", m_field);

       

        try {
            // Start WebServer for Elastic Dashboard "Download from robot" layout config
            // 在端口 5800 托管 Deploy 目录，供 Elastic Dashboard 获取配置 JSON
            WebServer.start(5800, Filesystem.getDeployDirectory().getAbsolutePath());
        } catch (Exception e) {
            DriverStation.reportWarning("ClientSubsystem: Failed to start WebServer! " + e.getMessage(), false);
        }
    }

    @Override
    public void periodic() {
        try {
            // 实时更新机器人本体在场地上的位置！
            if (m_drivesubsystem != null) {
                // 注意：这里我们读取 Phoenix 6 强类型系统的 Pose
                Pose2d currentPose = m_drivesubsystem.getState().Pose;
                m_field.setRobotPose(currentPose);
            }

            // Publish central match telemetry parameters
            SmartDashboard.putNumber("MatchTime", DriverStation.getMatchTime());
            // 🚨 架构师注意：Java 中电池电压一般从 RobotController 获取
            SmartDashboard.putNumber("BatteryVoltage", RobotController.getBatteryVoltage()); 
            
            updateMatchPhaseCountdown();

        } catch (Exception e) {
            SmartDashboard.putString("ClientSubsystem Periodic Failed:", e.getMessage());
        }
    }

    private void updateMatchPhaseCountdown() {
        // 从 Dashboard 获取用户的选择 (防空指针保护)
        is_our_turn_first = m_firstAttackerChooser.getSelected() != null ? m_firstAttackerChooser.getSelected() : true;

        double match_time = DriverStation.getMatchTime();
        boolean is_auto = DriverStation.isAutonomousEnabled();
        boolean is_teleop = DriverStation.isTeleopEnabled();

        // 比赛未开始或禁用时
        if (match_time < 0 || (!is_auto && !is_teleop)) {
            SmartDashboard.putString("Match_Phase", "Waiting/Disabled");
            SmartDashboard.putNumber("Phase_Countdown", 0.0);
            SmartDashboard.putBoolean("Can_Shoot_Now", false);
            return;
        }

        MatchPhase current_phase = MatchPhase.kUnknown;
        double countdown = 0.0;
        String phase_string = "Unknown";
        boolean can_i_shoot = false;

        // 1. 自动阶段 (AUTO): 倒计时 20 -> 0
        if (is_auto) {
            current_phase = MatchPhase.kAuto;
            countdown = match_time;
            phase_string = "AUTO (Both Active)";
            can_i_shoot = true;
        }
        // 2. 手动阶段 (TELEOP): 倒计时 140 (2:20) -> 0
        else if (is_teleop) {
            if (match_time > 130.0) {  // 2:20 - 2:10
                current_phase = MatchPhase.kTransition;
                countdown = match_time - 130.0;
                phase_string = "Transition (Both)";
                can_i_shoot = true;
            } else if (match_time >= 105.0 && match_time <= 130.0) { // 2:10 - 1:45
                current_phase = MatchPhase.kSwitch1;
                countdown = match_time - 105.0;
                can_i_shoot = is_our_turn_first;
                phase_string = can_i_shoot ? "Switch 1 (OUR Turn)" : "Switch 1 (OPPONENT)";
            } else if (match_time >= 80.0 && match_time < 105.0) {  // 1:45 - 1:20
                current_phase = MatchPhase.kSwitch2;
                countdown = match_time - 80.0;
                can_i_shoot = !is_our_turn_first;  // 交换逻辑
                phase_string = can_i_shoot ? "Switch 2 (OUR Turn)" : "Switch 2 (OPPONENT)";
            } else if (match_time >= 55.0 && match_time < 80.0) {   // 1:20 - 0:55
                current_phase = MatchPhase.kSwitch3;
                countdown = match_time - 55.0;
                can_i_shoot = is_our_turn_first;
                phase_string = can_i_shoot ? "Switch 3 (OUR Turn)" : "Switch 3 (OPPONENT)";
            } else if (match_time >= 30.0 && match_time < 55.0) {   // 0:55 - 0:30
                current_phase = MatchPhase.kSwitch4;
                countdown = match_time - 30.0;
                can_i_shoot = !is_our_turn_first;
                phase_string = can_i_shoot ? "Switch 4 (OUR Turn)" : "Switch 4 (OPPONENT)";
            } else if (match_time < 30.0) { // 0:30 - 0:00 Endgame
                current_phase = MatchPhase.kEndgame;
                countdown = match_time;
                phase_string = "ENDGAME (Both)";
                can_i_shoot = true;
            }
        }

        SmartDashboard.putString("Match_Phase", phase_string);
        SmartDashboard.putNumber("Phase_Countdown", countdown);
        SmartDashboard.putBoolean("Can_Shoot_Now", can_i_shoot);
    }
}