from hci_anchor_data import HPE_3D
from motion_capture import mc_node

dict_path_50 = 'model_weight/abest_model_50_84.7558-lite.pth'
dict_path_152 = 'model_weight/abest_model_152_80.1034-lite.pth'
smoother_weight = "model_weight/abest_model_7_45.3471.pth"
joint_num = 18
f = [451.477162, 453.875205]  # aloha 电脑相机参数
c = [272.213870, 294.493310]

if __name__ == '__main__':
    hpe = HPE_3D(dict_path_152, smoother_weight, joint_num, 7, f, c, resnet=152)
    mc = mc_node("/vrpn_client_node/hand4/pose")
    hpe.infer()
