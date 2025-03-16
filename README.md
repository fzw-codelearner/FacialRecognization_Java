# 环境配置
- linux环境
- 安装好了opencv库和dlib库的c++链接库
- linux配置好了java

# 修改
- java文件夹下是Java的主函数
- src文件夹下是JNI文件
- 在Cmakefile文件中修改为你的Opencv库和dilb库的.so文件地址

# 编译
- cd 项目文件夹
- rm -rf build
- mkdir build
- cd build
- cmake ..
- make( 然后就能在build文件夹内部发现build/libFaceLib.so lib/文件)
- cd ..
- cp build/libFaceLib.so lib/

# 运行
- sh run.sh
