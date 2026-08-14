package frc.robot.utils;

import java.io.File;
import java.io.IOException;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.Filesystem;

public class PassPlannerReader {

    // =======================================================
    // 1. 数据结构层：严格匹配前端 JSON 的键名 (Key Names)
    // =======================================================
    
    // 对应 JSON 最外层的对象
    public static class RobotPath {
        public BrakePID brakePID;
        public PathSegment[] segments;
    }

    // 对应 JSON 里的 "brakePID"
    public static class BrakePID {
        public double p;
        public double i;
        public double d;
    }

    // 对应 JSON 里的 "segments" 数组中的每一项
    public static class PathSegment {
        public double[] xArray;
        public double[] yArray;
        public double[] rotArray;
        public double[] speedArray;
        public boolean stopAtEnd;
    }

    // =======================================================
    // 2. 解析核心层：一键读取
    // =======================================================
    
    /**
     * 读取部署目录下的 JSON 路线文件
     * @param fileName 文件名，不需要带 .json 后缀 (例如传入 "Auto_Red_Amp")
     * @return 包含 PID 和所有路段数组的 RobotPath 对象
     */
    public static RobotPath loadPath(String fileName) {
        try {
            File file = new File(Filesystem.getDeployDirectory(), "paths/" + fileName + ".json");
            ObjectMapper mapper = new ObjectMapper();
            
            // 🚨 架构师防呆指令：忽略 JSON 中专门给网页用的冗余元数据 (waypoints)，防止反序列化崩溃！
            mapper.configure(com.fasterxml.jackson.databind.DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);
            
            return mapper.readValue(file, RobotPath.class);
            
        } catch (IOException e) {
            System.err.println("🚨 架构师警报: 无法读取路径文件: " + fileName + ".json");
            e.printStackTrace();
            RobotPath emptyPath = new RobotPath();
            emptyPath.brakePID = new BrakePID(); 
            emptyPath.segments = new PathSegment[0];
            return emptyPath;
        }
    }
}