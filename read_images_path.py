import os

# 定义一个函数来查找图片并写入到txt文件
def find_and_write_images(folder_name, output_file):
    # 拼接文件夹的绝对路径
    folder_path = os.path.join(os.getcwd(), folder_name)
    
    # 检查文件夹是否存在
    if not os.path.exists(folder_path):
        print(f"文件夹 {folder_path} 不存在。")
        return

    # 初始化图片列表
    images_list = []

    # 遍历文件夹
    for root, dirs, files in os.walk(folder_path):
        for file in files:
            # 检查文件扩展名是否为图片格式
            if file.lower().endswith(('.jpg', '.jpeg', '.png', '.gif', '.bmp')):
                # 获取文件的绝对路径
                full_path = os.path.abspath(os.path.join(root, file))
                # 添加到图片列表
                images_list.append(full_path)

    # 写入到txt文件
    with open(output_file, 'w') as f:
        for image_path in images_list:
            f.write(image_path + '\n')

# 调用函数
find_and_write_images('images_train', 'images_train.txt')
find_and_write_images('images_val', 'images_val.txt')