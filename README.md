v1.2.1 删除images目录下png相关文件的依赖，全面使用tiff格式，优化匹配结果的显示效果（增加透明度）

v1.2.2 修复内存不足导致电脑崩溃的BUG，新增内存管理线程

v1.2.3 增加图像处理线程，优化内存管理，优化匹配进度显示效果

conda create -n cpp python=3.11

conda activate cpp

conda install conda-forge::cmake

conda install conda-forge::opencv

conda install conda-forge::gdal
