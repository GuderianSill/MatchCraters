v1.2.1 删除images目录下png相关文件的依赖，全面使用tiff格式，优化匹配结果的显示效果（增加透明度）

v1.2.2 修复内存不足导致电脑崩溃的BUG，新增内存管理线程

v1.2.3 增加图像处理线程，优化内存管理，优化匹配进度显示效果

v1.3.0 新增图像匹配方法，优化命令行输入形式，优化key文件储存形式

v1.3.1 修法key文件储存与显示不一样的bug，优化命令行显示形式

conda create -n cpp python=3.11

conda activate cpp

conda install conda-forge::cmake

conda install conda-forge::opencv

conda install conda-forge::gdal
