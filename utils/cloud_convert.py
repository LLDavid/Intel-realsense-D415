import convertcloud as cvc

file_dir="C:\\Users\\Owner\\Li\\PyCode\\MyRepo\\Intel-realsense-D415\\pcd\\"
file_path = file_dir + str(30)+".pcd"
conv = cvc.Converter()
conv.load_points(file_path)
conv.convert("test.ply")