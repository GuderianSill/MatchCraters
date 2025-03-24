#!/bin/bash

# 初始化计数器
count=0

# 遍历当前文件夹内的所有子文件夹
while IFS= read -r -d '' subfolder; do
    if [ -d "$subfolder" ]; then
        file_count=$(find "$subfolder" -maxdepth 1 -type f | wc -l)
        if [ "$file_count" -eq 1 ]; then
            ((count++))
            echo "${subfolder%/}"
        fi
    fi
done < <(find . -maxdepth 1 -type d -print0 | sed '1d' | tr '\0' '\n' | while read -r line; do echo -n "$line\0"; done)

echo "当前文件夹里，只有一个文件的子文件夹数量为: $count"
    
