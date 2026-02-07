'''
Author: 解博炜
LastEditors: 解博炜
LastEditTime: 2025-03-13
FilePath: hand_open.py
Description: 用于计算手部是否开合
Version: 
Notion: 
Copyright (c) 2025 by BIT807s, All Rights Reserved. 
'''
import numpy as np

def length_rate(v1, v2):
    '''计算两个向量的长度比值,短/长
    '''
    # 计算每个向量的模长
    magnitude_v1 = np.linalg.norm(v1)
    magnitude_v2 = np.linalg.norm(v2)
    if magnitude_v2 >magnitude_v1:
        return magnitude_v1 / magnitude_v2
    else:
        return magnitude_v2 / magnitude_v1

def are_vectors_collinear(v1, v2):
    '''输入两个向量，判断是否共线，返回值为共线的余弦值的绝对值,0为不共线，1为共线
    '''
    # 计算点积
    dot_product = np.dot(v1, v2)
    
    # 计算每个向量的模长
    magnitude_v1 = np.linalg.norm(v1)
    magnitude_v2 = np.linalg.norm(v2)
    
    # 计算夹角的余弦值
    cos_theta = dot_product / (magnitude_v1 * magnitude_v2)
    
    # 如果夹角为 0 或 180 度（即 cos(θ) = ±1），则在同一直线上
    return np.abs(cos_theta)

def calculate_hand_openness(landmarks):
    """
    通过分析手部地标点计算手掌开合度
    适应不同朝向的手部姿态
    
    Args:
        landmarks: 手部21个地标点
        
    Returns:
        bool: 手掌是否打开
    """
    # 定义地标点索引
    WRIST = 0
    THUMB_CMC = 1
    THUMB_MCP = 2
    THUMB_IP = 3
    THUMB_TIP = 4
    INDEX_MCP = 5
    INDEX_PIP = 6
    INDEX_DIP = 7
    INDEX_TIP = 8
    MIDDLE_MCP = 9
    MIDDLE_PIP = 10
    MIDDLE_DIP = 11
    MIDDLE_TIP = 12
    RING_MCP = 13
    RING_PIP = 14
    RING_DIP = 15
    RING_TIP = 16
    PINKY_MCP = 17
    PINKY_PIP = 18
    PINKY_DIP = 19
    PINKY_TIP = 20
    
    # 从地标点提取3D坐标
    points = []
    for landmark in landmarks:
        points.append(np.array([landmark.x, landmark.y, landmark.z]))
    
    # 计算手掌中心点 (使用手腕和MCP关节的平均值)
    palm_center = np.mean([
        points[WRIST],
        points[INDEX_MCP],
        points[MIDDLE_MCP],
        points[RING_MCP],
        points[PINKY_MCP]
    ], axis=0)
    
    # 计算手掌法向量 (使用掌心平面的法向量)
    v1 = points[INDEX_MCP] - points[WRIST]
    v2 = points[PINKY_MCP] - points[WRIST]
    palm_normal = np.cross(v1, v2)
    palm_normal = palm_normal / np.linalg.norm(palm_normal)
    
    # 计算各手指的弯曲度
    finger_bend_angles = []
    
    # 计算食指弯曲度
    index_angle = calculate_finger_bend_angle(
        points[INDEX_MCP], points[INDEX_PIP], points[INDEX_DIP], points[INDEX_TIP]
    )
    finger_bend_angles.append(index_angle)
    
    # 计算中指弯曲度
    middle_angle = calculate_finger_bend_angle(
        points[MIDDLE_MCP], points[MIDDLE_PIP], points[MIDDLE_DIP], points[MIDDLE_TIP]
    )
    finger_bend_angles.append(middle_angle)
    
    # 计算无名指弯曲度
    ring_angle = calculate_finger_bend_angle(
        points[RING_MCP], points[RING_PIP], points[RING_DIP], points[RING_TIP]
    )
    finger_bend_angles.append(ring_angle)
    
    # 计算小指弯曲度
    pinky_angle = calculate_finger_bend_angle(
        points[PINKY_MCP], points[PINKY_PIP], points[PINKY_DIP], points[PINKY_TIP]
    )
    finger_bend_angles.append(pinky_angle)
    
    # # 计算拇指弯曲度 (拇指结构不同，单独计算)
    # thumb_angle =calculate_finger_bend_angle(
    #     points[THUMB_CMC], points[THUMB_MCP], points[THUMB_IP], points[THUMB_TIP]
    # )
    # finger_bend_angles.append(thumb_angle)
    
    # # 计算指尖到掌心的投影距离与手指长度的比值
    # projection_ratios = []
    # fingers = [
    #     (INDEX_MCP, INDEX_TIP),
    #     (MIDDLE_MCP, MIDDLE_TIP),
    #     (RING_MCP, RING_TIP),
    #     (PINKY_MCP, PINKY_TIP),
    #     (THUMB_MCP, THUMB_TIP)
    # ]
    
    # for base_idx, tip_idx in fingers:
    #     # 计算手指向量
    #     finger_vector = points[tip_idx] - points[base_idx]
    #     finger_length = np.linalg.norm(finger_vector)
        
    #     # 计算指尖到掌心的向量
    #     tip_to_palm = points[tip_idx] - palm_center
        
    #     # 计算指尖到掌心的投影在法向量上的分量
    #     projection = np.dot(tip_to_palm, palm_normal)
        
    #     # 计算投影与手指长度的比值
    #     if finger_length > 0:
    #         ratio = projection / finger_length
    #         projection_ratios.append(ratio)
    
    # 计算弯曲角度和投影比例的综合特征
    avg_bend_angle = np.mean(finger_bend_angles)
    # avg_projection_ratio = np.mean(projection_ratios)
    
    # 记录详细信息以便调试
    print(f"平均弯曲角度: {avg_bend_angle:.2f}") #tem
    # print(f"平均投影比例: {avg_projection_ratio:.2f}") #tem
    
    # 根据弯曲角度和投影比例判断手掌开合
    # 较大的角度和较大的投影比例表示手掌打开
    is_open = avg_bend_angle > 120
    return is_open,finger_bend_angles

class HandStateManager:
    """
    手部状态管理器，处理手掌开合状态的平滑切换
    通过连续多帧判断和置信度阈值来稳定状态切换
    """
    def __init__(self, confidence_threshold=0.8, frame_threshold=3):
        """
        初始化状态管理器
        
        Args:
            confidence_threshold: 置信度阈值，高于此值的判断才会被纳入考虑
            frame_threshold: 需要连续多少帧判断相同才切换状态
        """
        self.confidence_threshold = confidence_threshold
        self.frame_threshold = frame_threshold
        
        # 状态变量
        self.is_hand_open = False  # 当前手部状态
        self.gripper_value = 0.0   # 当前夹爪值
        
        # 状态计数器
        self.open_counter = 0      # 连续检测到手掌打开的帧数
        self.closed_counter = 0    # 连续检测到手掌闭合的帧数
        
        # 夹爪参数
        self.open_gripper_value = 0.05  # 夹爪打开值
        self.closed_gripper_value = 0.0 # 夹爪闭合值
        self.switch = False
    
    def update(self, hand_landmarks, hand_confidence=None):
        """
        更新手部状态
        
        Args:
            hand_landmarks: 手部地标点
            hand_confidence: 手部检测置信度
            
        Returns:
            float: 当前的夹爪控制值
        """
        # 如果没有手部地标点，重置计数器
        if hand_landmarks is None:
            self.open_counter = 0
            self.closed_counter = 0
            return self.gripper_value
        
        # 计算手掌开合状态
        is_open, confidence_score, finger_states = calculate_hand_openness2(
            hand_landmarks, hand_confidence
        )
        
        # if is_open:
        #     print(f"手掌张开------  置信度: {confidence_score:.2f}") #temp 打印
        # else:
        #     print(f"------手掌合拢  置信度: {confidence_score:.2f}")
        
        # 只处理置信度足够高的帧
        if confidence_score >= self.confidence_threshold:
            if is_open:
                # 检测到手掌打开
                self.open_counter += 1
                self.closed_counter = 0
                # print("手掌张开------") 
                
            else:
                # 检测到手掌闭合
                self.closed_counter += 1
                self.open_counter = 0
                # print("手掌合拢-")
                
        self.switch = False
        # 根据连续帧计数判断是否切换状态
        if self.open_counter >= self.frame_threshold and not self.is_hand_open:
            # 切换到打开状态
            self.switch = True
            self.is_hand_open = True
            self.gripper_value = self.open_gripper_value
        elif self.closed_counter >= self.frame_threshold and self.is_hand_open:
            # 切换到闭合状态
            self.switch = True
            self.is_hand_open = False
            self.gripper_value = self.closed_gripper_value
            # print(f"------手掌合拢  置信度: {confidence_score:.2f}")
        return self.gripper_value
    
    def get_status_text(self):
        """获取当前状态的文本描述，可用于UI显示"""
        if self.is_hand_open:
            status = "---------打开-----------"
            confidence = self.open_counter / self.frame_threshold
        else:
            status = "---------------------闭合"
            confidence = self.closed_counter / self.frame_threshold
            
        confidence = min(confidence, 1.0)
        return f"手部状态: {status} (稳定度: {confidence:.2f})"


def calculate_hand_openness2(landmarks, hand_confidence=None):
    """
    通过分析手部地标点计算手掌开合度
    适应不同朝向的手部姿态，结合置信度提高准确率
    
    Args:
        landmarks: 手部21个地标点
        hand_confidence: 手部检测的置信度，范围0-1 (可选)
        
    Returns:
        tuple: (is_open, confidence_score, finger_states)
               is_open - 手掌是否打开
               confidence_score - 判断的置信度
               finger_states - 每个手指的状态 (1=打开, 0=弯曲)
    """
    # 定义地标点索引
    WRIST = 0
    THUMB_CMC = 1
    THUMB_MCP = 2
    THUMB_IP = 3
    THUMB_TIP = 4
    INDEX_MCP = 5
    INDEX_PIP = 6
    INDEX_DIP = 7
    INDEX_TIP = 8
    MIDDLE_MCP = 9
    MIDDLE_PIP = 10
    MIDDLE_DIP = 11
    MIDDLE_TIP = 12
    RING_MCP = 13
    RING_PIP = 14
    RING_DIP = 15
    RING_TIP = 16
    PINKY_MCP = 17
    PINKY_PIP = 18
    PINKY_DIP = 19
    PINKY_TIP = 20
    
    # 从地标点提取3D坐标
    points = []
    for landmark in landmarks:
        points.append(np.array([landmark.x, landmark.y, landmark.z]))
    
    # 计算手掌中心点
    palm_center = np.mean([
        points[WRIST],
        points[INDEX_MCP],
        points[MIDDLE_MCP],
        points[RING_MCP],
        points[PINKY_MCP]
    ], axis=0)
    
    # 计算手掌法向量 (使用掌心平面的法向量)
    v1 = points[INDEX_MCP] - points[WRIST]
    v2 = points[PINKY_MCP] - points[WRIST]
    palm_normal = np.cross(v1, v2)
    palm_normal = palm_normal / np.linalg.norm(palm_normal)
    
    # 计算各手指的弯曲度
    finger_bend_angles = []
    
    # 计算食指弯曲度
    index_angle = calculate_finger_bend_angle(
        points[INDEX_MCP], points[INDEX_PIP], points[INDEX_DIP], points[INDEX_TIP]
    )
    finger_bend_angles.append(index_angle)
    
    # 计算中指弯曲度
    middle_angle = calculate_finger_bend_angle(
        points[MIDDLE_MCP], points[MIDDLE_PIP], points[MIDDLE_DIP], points[MIDDLE_TIP]
    )
    finger_bend_angles.append(middle_angle)
    
    # 计算无名指弯曲度
    ring_angle = calculate_finger_bend_angle(
        points[RING_MCP], points[RING_PIP], points[RING_DIP], points[RING_TIP]
    )
    finger_bend_angles.append(ring_angle)
    
    # 计算小指弯曲度
    pinky_angle = calculate_finger_bend_angle(
        points[PINKY_MCP], points[PINKY_PIP], points[PINKY_DIP], points[PINKY_TIP]
    )
    finger_bend_angles.append(pinky_angle)
    
    # 计算拇指弯曲度 (拇指结构不同，使用不同的角度阈值)
    thumb_angle = calculate_finger_bend_angle(
        points[THUMB_CMC], points[THUMB_MCP], points[THUMB_IP], points[THUMB_TIP]
    )
    
    # 计算指尖到掌心的距离比例
    tip_to_palm_distances = []
    finger_lengths = []
    
    # 计算四个手指的长度和指尖到掌心的距离
    fingers = [
        (INDEX_MCP, INDEX_PIP, INDEX_DIP, INDEX_TIP),
        (MIDDLE_MCP, MIDDLE_PIP, MIDDLE_DIP, MIDDLE_TIP),
        (RING_MCP, RING_PIP, RING_DIP, RING_TIP),
        (PINKY_MCP, PINKY_PIP, PINKY_DIP, PINKY_TIP)
    ]
    
    for mcp, pip, dip, tip in fingers:
        # 计算手指长度 (MCP到TIP的直线距离)
        finger_length = np.linalg.norm(points[tip] - points[mcp])
        finger_lengths.append(finger_length)
        
        # 计算指尖到掌心的距离
        tip_to_palm = np.linalg.norm(points[tip] - palm_center)
        tip_to_palm_distances.append(tip_to_palm)
    
    # 计算指尖到掌心的距离与手指长度的比值
    distance_ratios = [tip_dist / finger_len for tip_dist, finger_len in zip(tip_to_palm_distances, finger_lengths)]
    
    # 判断每个手指是否打开 (1=打开, 0=弯曲)
    finger_states = []
    angle_thresholds = [140, 140, 140, 140]  # 四个手指的角度阈值
    
    for i, angle in enumerate(finger_bend_angles):
        # 结合角度和距离比例进行判断
        is_finger_open = (angle > angle_thresholds[i]) and (distance_ratios[i] > 1.2)
        finger_states.append(1 if is_finger_open else 0)
    
    # 拇指使用特殊判断
    thumb_distance = np.linalg.norm(points[THUMB_TIP] - palm_center)
    thumb_length = np.linalg.norm(points[THUMB_TIP] - points[THUMB_CMC])
    thumb_ratio = thumb_distance / thumb_length if thumb_length > 0 else 0
    
    # 添加拇指的状态
    is_thumb_open = (thumb_angle > 120) and (thumb_ratio > 0.7)
    finger_states.insert(0, 1 if is_thumb_open else 0)  # 将拇指状态放在列表开头
    
    # 计算开放手指的数量
    open_finger_count = sum(finger_states)
    
    # 计算置信度得分
    confidence_factors = []
    
    # 1. 基于角度的一致性
    angle_std = np.std(finger_bend_angles)
    angle_consistency = np.exp(-angle_std / 50)  # 标准差越小，一致性越高
    confidence_factors.append(angle_consistency)
    
    # 2. 根据手指状态的清晰度
    angle_clarity = np.mean([min(max(abs(angle - 90) / 90, 0), 1) for angle in finger_bend_angles])
    confidence_factors.append(angle_clarity)
    
    # 3. 如果提供了手部检测置信度，也考虑进去
    if hand_confidence is not None:
        confidence_factors.append(hand_confidence)
    
    # 综合置信度得分 (0-1之间)
    confidence_score = np.mean(confidence_factors)
    
    # 根据开放手指数量、角度和置信度判断手掌状态
    if open_finger_count >= 3:
        is_open = True
    elif open_finger_count <= 1:
        is_open = False
    else:
        # 在边界情况下依赖平均角度
        avg_bend_angle = np.mean(finger_bend_angles)
        is_open = avg_bend_angle > 130
    
    # 记录详细信息以便调试
    # if DEBUG:
    # print(f"平均弯曲角度: {np.mean(finger_bend_angles):.2f}")
    # print(f"手指状态: {finger_states}")
    # print(f"打开手指数: {open_finger_count}")
    # print(f"判断置信度: {confidence_score:.2f}")
    
    return is_open, confidence_score, finger_states

def calculate_finger_bend_angle(mcp, pip, dip, tip):
    """
    计算手指的弯曲角度
    
    参数:
        mcp, pip, dip, tip: 手指的四个关节点的3D坐标
        
    返回:
        float: 手指的弯曲角度 (180度表示完全伸直)
    """
    # 计算两个向量: MCP到PIP和指尖到DIP
    v1 = pip - mcp
    v2 = tip - dip
    
    # 标准化向量
    v1 = v1 / np.linalg.norm(v1)
    v2 = v2 / np.linalg.norm(v2)
    
    # 计算向量夹角 (弧度)
    dot_product = min(max(np.dot(v1, v2), -1.0), 1.0)
    angle_rad = np.arccos(dot_product)
    
    # 转换为度数，并调整为表示弯曲度 (180=完全伸直)
    angle_deg = 180 - np.degrees(angle_rad)
    
    # 考虑手指的弯曲方向
    # 计算中间向量 (PIP到DIP)
    v_mid = dip - pip
    v_mid = v_mid / np.linalg.norm(v_mid)
    
    # 检查弯曲的方向是否正确 (手指通常向掌心弯曲)
    # 如果弯曲方向不对，调整角度
    if np.dot(v1, v_mid) < 0.7:  # 如果PIP与MCP-PIP方向差异较大
        angle_deg = min(angle_deg, 90)  # 限制角度，表示不是伸直的
    
    return angle_deg

def calculate_finger_bend_angle2(mcp, pip, dip, tip):
    """
    计算手指的弯曲角度
    使用手指三个关节形成的两个向量之间的夹角
    
    Args:
        mcp, pip, dip, tip: 手指的四个关节点坐标
        
    Returns:
        float: 弯曲角度(0-180度，180表示完全伸直)
    """
    # 计算两个向量
    v1 = pip - mcp
    v2 = tip - pip
    
    # 归一化向量
    v1 = v1 / np.linalg.norm(v1)
    v2 = v2 / np.linalg.norm(v2)
    
    # 计算点积
    dot_product = np.dot(v1, v2)
    
    # 确保点积在有效范围内
    dot_product = np.clip(dot_product, -1.0, 1.0)
    
    # 计算角度(弧度)并转换为角度
    angle_rad = np.arccos(dot_product)
    angle_deg = np.degrees(angle_rad)
    
    # 返回伸直度(180-角度)，值越大表示手指越伸直
    return 180 - angle_deg