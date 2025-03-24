#!/bin/bash
#这是一个正则表达式替换命令，^ 表示匹配字符串的开头，dom_ 是要匹配的前缀，// 表示将匹配到的内容替换为空，即删除 dom_ 前缀
#rename 's/^dom_//' dom_*
#rename 's/\.echo\.cal//' *\.echo\.cal\.tif


# 遍历当前目录下所有以 dom_ 开头的文件
for file in dom_*; do
    # 检查文件是否存在
    if [ -e "$file" ]; then
        # 获取去掉 dom_ 前缀后的新文件名
        new_name="${file#dom_int8_}"
        # 执行重命名操作
        mv "$file" "$new_name"
        # 输出重命名信息
        echo "Renamed $file to $new_name"
    fi
done

