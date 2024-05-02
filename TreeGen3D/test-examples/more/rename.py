import os
import shutil

def copy_and_rename_jpg_files():
    # 获取当前目录
    current_directory = os.getcwd()
    # 遍历当前目录的文件
    for filename in os.listdir(current_directory):
        # 检查文件是否以.jpg结尾
        if filename.endswith(".jpg"):
            # 构造新的文件名
            new_filename = filename[:-4] + "_mini.jpg"
            # 构造原始文件和新文件的完整路径
            original_file = os.path.join(current_directory, filename)
            new_file = os.path.join(current_directory, new_filename)
            # 复制文件
            shutil.copyfile(original_file, new_file)
            print(f"Copied and renamed {filename} to {new_filename}")

# 调用函数
copy_and_rename_jpg_files()