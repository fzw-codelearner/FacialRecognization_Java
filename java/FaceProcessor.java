public class FaceProcessor {
    // 声明本地方法
    static {
        System.loadLibrary("FaceLib"); // 加载 JNI 共享库
    }

    public native int[][] detectLandmarks(String imagePath);
    public native void applyExaggeration(String imagePath, String outputPath);

    public static void main(String[] args) {
        FaceProcessor processor = new FaceProcessor();
        String imagePath = "data/image.png";

        // 获取人脸特征点
        int[][] landmarks = processor.detectLandmarks(imagePath);
        System.out.println("Detected Landmarks: " + java.util.Arrays.deepToString(landmarks));

        // 应用夸张变形
        processor.applyExaggeration(imagePath, "output.jpg");
    }
}
