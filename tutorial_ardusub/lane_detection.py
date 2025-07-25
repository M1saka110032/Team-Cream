#/home/eugene/cv-intro/lane_detection.py

import matplotlib.pyplot as plt
import numpy as np
import cv2
def cut_top_half(img):
    height = img.shape[0]
    img = img[height // 2:, :]
    return img
def get_line_length(line):
    x1, y1, x2, y2 = line[0]
    return np.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def get_slopes_intercepts(img,lines):
    height, width, _ = img.shape
    slopes = []
    angles = []
    intercepts = []
    y_intercepts = []
    image_bottom_y = height
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if x2 - x1 != 0:
                slope = (y2 - y1) / (x2 - x1)
                angle = np.degrees(np.arctan2(y1-y2, x1 - x2))
                if 0 < angle <=90:
                    angle = 90 - angle
                elif 90 < angle <= 180:
                    angle =  angle - 90
                elif -90 < angle <= 0:
                    angle = angle + 90
                elif -180 < angle <= -90:
                    angle = 90 -(angle + 180)

                intercept = x1 + (image_bottom_y - y1) / slope if slope != 0 else x1
                y_intercept = y1 - slope * x1
            else:
                slope = np.inf
                angle = 90.0 
                intercept = x1
                y_intercept = np.nan
            slopes.append(slope)
            angles.append(angle)
            intercepts.append(intercept)
            y_intercepts.append(y_intercept)
    return slopes, angles, intercepts, y_intercepts



def filter_StEdSim_lines(img,lines, distance_threshold =100):

    if lines is None or len(lines) == 0:
        return [], [], []
    slopes, angles, intercepts, y_intercepts = get_slopes_intercepts(img,lines)
    line_data = [(line, slope, angle, intercept, get_line_length(line)) for line, slope, angle, intercept in zip(lines, slopes, angles, intercepts)]
    line_data.sort(key=lambda x: x[3])
    
    groups = []
    used = [False] * len(line_data)
    
    for i in range(len(line_data)):
        if used[i]:
            continue
        current_group = [line_data[i]]
        used[i] = True
        
        for j in range(i + 1, len(line_data)):
            if used[j]:
                continue
            line1, slope1, angle1, intercept1 = line_data[i][0], line_data[i][1], line_data[i][2], line_data[i][3]
            line2, slope2, angle2, intercept2 = line_data[j][0], line_data[j][1], line_data[j][2], line_data[j][3]
            
            x11, y11, x12, y12 = line1[0]
            x21, y21, x22, y22 = line2[0]
            start_dist = np.sqrt((x11-x21)**2 + (y11-y21)**2)
            end_dist = np.sqrt((x12-x22)**2 + (y12-y22)**2)
            # check slope and intercept similar
            if start_dist < distance_threshold and end_dist < distance_threshold:
                current_group.append(line_data[j])
                used[j] = True
        
        groups.append(current_group)
    
    # 从每组中选择最长的线段
    filtered_lines = []
    filtered_slopes = []
    filtered_angles = []
    filtered_intercepts = []
    
    for group in groups:
        if group:
            longest_line = max(group, key=lambda x: x[4])
            filtered_lines.append(longest_line[0])
            filtered_slopes.append(longest_line[1])
            filtered_angles.append(longest_line[2])
            filtered_intercepts.append(longest_line[3])

    
    return filtered_lines, filtered_slopes, filtered_angles, filtered_intercepts

def filter_Slo_interc_lines(img, lines, angle_threshold=1, intercept_threshold=200):

    if lines is None or len(lines) == 0:
        return [], [], []
    slopes, angles, intercepts, y_intercepts = get_slopes_intercepts(img,lines)
    line_data = [(line, slope, angle, intercept, get_line_length(line)) for line, slope, angle, intercept in zip(lines, slopes, angles, intercepts)]
    line_data.sort(key=lambda x: x[3])
    
    groups = []
    used = [False] * len(line_data)
    
    for i in range(len(line_data)):
        if used[i]:
            continue
        current_group = [line_data[i]]
        used[i] = True
        
        for j in range(i + 1, len(line_data)):
            if used[j]:
                continue
            slope1, angle1, intercept1 = line_data[i][1], line_data[i][2], line_data[i][3]
            slope2, angle2, intercept2 = line_data[j][1], line_data[j][2], line_data[j][3]
            
            # check slope and intercept similar
            if slope1 == np.inf and slope2 == np.inf:
                if abs(intercept1 - intercept2) < intercept_threshold:
                    current_group.append(line_data[j])
                    used[j] = True
            elif slope1 != np.inf and slope2 != np.inf:
                if abs(angle1 - angle2) < angle_threshold and abs(intercept1 - intercept2) < intercept_threshold:
                    current_group.append(line_data[j])
                    used[j] = True
        
        groups.append(current_group)
    
    # 从每组中选择最长的线段
    filtered_lines = []
    filtered_slopes = []
    filtered_angles = []
    filtered_intercepts = []
    
    for group in groups:
        if group:
            longest_line = max(group, key=lambda x: x[4])
            filtered_lines.append(longest_line[0])
            filtered_slopes.append(longest_line[1])
            filtered_angles.append(longest_line[2])
            filtered_intercepts.append(longest_line[3])

    
    return filtered_lines, filtered_slopes, filtered_angles, filtered_intercepts

def detect_lines(img,threshold1=20,threshold2=60,apertureSize=3,minLineLength=500,maxLineGap=50):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, threshold1=threshold1, threshold2=threshold2, apertureSize=apertureSize) # detect edges
    lines = cv2.HoughLinesP(
                edges,
                1,
                np.pi/180,
                100,
                minLineLength=minLineLength,
                maxLineGap=maxLineGap,
        )
    
    old = len(lines)
    new = 0
    while old != new:
        old = len(lines)
        print(old)
        lines, slopes, angles, intercepts= filter_StEdSim_lines(img,lines)
        lines, slopes, angles, intercepts= filter_Slo_interc_lines(img,lines)
        new = len(lines)
    filtered_lines, filtered_slopes, filtered_angles, filtered_intercepts = lines, slopes, angles, intercepts
    
    #for i in range(len(filtered_lines)):
    #    print(f"line:{filtered_lines[i]} slope:{filtered_slopes[i]:.4f} angle:{filtered_angles[i]:.4f} intercept:{filtered_intercepts[i]:.0f}")
    
    return lines , edges



def draw_lines(img,lines,color= (0, 255, 0)):
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(img, (x1, y1), (x2, y2), color, 2)
    else:
        print("No lines detected")

    return img

def extend_line_to_edges(slope, intercept,y_intercept, width, height):
    points = []
    if slope != np.inf:
        # 与顶部 y=0 的交点
        x_top = int((0 - height) / slope + intercept)
        if 0 <= x_top <= width:
            points.append((x_top, 0))
        # 与底部 y=height 的交点
        x_bottom = int(intercept)
        if 0 <= x_bottom <= width:
            points.append((x_bottom, height))
        # 与右边 x=width 的交点
        y_right = int(slope * (width - intercept) + height)
        if 0 <= y_right <= height:
            points.append((width, y_right))
    # 与左边 x=0 的交点
    y_left = int(slope * (0 - intercept) + height)
    if 0 <= y_left <= height:
        points.append((0, y_left))
    
    if len(points) >= 2:
        return points[:2]
    return None
    
def detect_lanes(img, lines):
    height, width, _ = img.shape
    if lines is None or len(lines) < 2:
        return []

    angle_threshold = 20
    max_position_threshold = 500
    min_position_threshold = 100  # 提高阈值
    min_intercept_diff = 200  # 新增：最小截距差

    slopes, angles, intercepts, y_intercepts = get_slopes_intercepts(img, lines)
    line_data = [(line, slope, angle, intercept, y_intercepts)
                 for line, slope, angle, intercept, y_intercepts in zip(lines, slopes, angles, intercepts, y_intercepts)]
    line_data.sort(key=lambda x: x[3])

    lanes = []
    used = [False] * len(lines)

    for i in range(len(line_data)):
        if used[i]:
            continue

        line1, slope1, angle1, intercept1, y_intercept1 = line_data[i]

        if -0.001<slope1<0.001:
            continue

        print(f"Line {i+1}: {line1[0]}, slope={slope1:.4f}, intercept={intercept1:.4f}, y_intercept={y_intercept1:.4f}")
        if abs(slope1) == np.inf:
            continue
        
        pts1 = extend_line_to_edges(slope1, intercept1,y_intercept1, width, height)

        print(pts1)

        if not pts1:
            continue
        p1_start, p1_end = pts1

        for j in range(i + 1, len(line_data)):
            if used[j]:
                continue

            line2, slope2, angle2, intercept2, y_intercept2 = line_data[j]

            if -0.001<slope2<0.001:
                continue

            print(f"Line {j+1}: {line2[0]}, slope={slope2:.4f}, intercept={intercept2:.4f}, y_intercept={y_intercept2:.4f}")
            if abs(slope2) == np.inf:
                continue

            pts2 = extend_line_to_edges(slope2, intercept2, y_intercept2, width, height)

            print(pts1)

            
            if not pts2:
                continue
            p2_start, p2_end = pts2

            start_dist1 = np.sqrt((p1_start[0] - p2_start[0]) ** 2 + (p1_start[1] - p2_start[1]) ** 2)
            end_dist1 = np.sqrt((p1_end[0] - p2_end[0]) ** 2 + (p1_end[1] - p2_end[1]) ** 2)
            avg_dist1 = (start_dist1 + end_dist1) / 2

            # 组合2：p1_start 到 p2_end，p1_end 到 p2_start
            start_dist2 = np.sqrt((p1_start[0] - p2_end[0]) ** 2 + (p1_start[1] - p2_end[1]) ** 2)
            end_dist2 = np.sqrt((p1_end[0] - p2_start[0]) ** 2 + (p1_end[1] - p2_start[1]) ** 2)
            avg_dist2 = (start_dist2 + end_dist2) / 2

            # 选择平均距离较小的组合
            if avg_dist1 <= avg_dist2:
                selected_start_dist = start_dist1
                selected_end_dist = end_dist1
                selected_avg_dist = avg_dist1
            else:
                selected_start_dist = start_dist2
                selected_end_dist = end_dist2
                selected_avg_dist = avg_dist2

            angle_diff = abs(angle1 - angle2)
            intercept_diff = abs(intercept1 - intercept2)

            print(f"Line pair {i+1},{j+1}: Line1={line1[0]}, Line2={line2[0]}, p1_start={p1_start}, p1_end={p1_end}, p2_start={p2_start}, p2_end={p2_end}, start_dist={selected_start_dist}, end_dist={selected_end_dist}, avg_dist={selected_avg_dist}, angle_diff={angle_diff}, intercept_diff={intercept_diff}")

            if selected_avg_dist > max_position_threshold or selected_avg_dist < min_position_threshold:
                continue
            if intercept_diff < min_intercept_diff:
                continue
            if angle_diff > angle_threshold:
                continue

            x11, y11, x12, y12 = line1[0]
            x21, y21, x22, y22 = line2[0]

            def do_lines_intersect(p1, p2, q1, q2):
                def ccw(a, b, c):
                    return (c[1] - a[1]) * (b[0] - a[0]) > (b[1] - a[1]) * (c[0] - a[0])
                return (ccw(p1, q1, q2) != ccw(p2, q1, q2)) and (ccw(p1, p2, q1) != ccw(p1, p2, q2))

            if do_lines_intersect((x11, y11), (x12, y12), (x21, y21), (x22, y22)):
                continue

            lanes.append((line1, line2))
            used[i] = used[j] = True
            break

    return lanes


def draw_lanes(img, lanes):
    colors = [
        (255, 0, 0),    # Blue
        (0, 255, 0),    # Green
        (0, 0, 255),    # Red
        (255, 255, 0),  # Cyan
        (255, 0, 255),  # Purple
        (0, 255, 255),  # Yellow
        (128, 0, 0),    # Dark Blue
        (0, 128, 0),    # Dark Green
        (0, 0, 128),    # Dark Red
        (128, 128, 0),  # Dark Cyan
        (128, 0, 128),  # Dark Purple
        (0, 128, 128)   # Dark Yellow
    ]
    
    for i, lane in enumerate(lanes):
        color = colors[i % len(colors)]
        
        for line in lane:
            x1, y1, x2, y2 = line[0]
            cv2.line(img, (x1, y1), (x2, y2), color, 2)
    
    return img


def get_lane_center(lanes,img):
    height, width, channels = img.shape
    image_width = width
    if not lanes or len(lanes) == 0:
        return None, None

    min_avg_intercept = float('inf')
    cloest_intercept = None
    closest_slope = None

    for lane in lanes:
        slopes, angles, intercepts, y_intercepts = get_slopes_intercepts(img,lane)
        if len(intercepts) == 2:
            avg_intercept = np.mean(intercepts)
            avg_slope = np.mean([s for s in slopes if s != np.inf]) if any(s != np.inf for s in slopes) else np.inf
            
            if abs(avg_intercept-image_width/2) < abs(min_avg_intercept-image_width/2):
                min_avg_intercept = avg_intercept
                cloest_intercept = int(avg_intercept)
                closest_slope = avg_slope
                pts1 = extend_line_to_edges(closest_slope, cloest_intercept,None, width, height)
                p1_start, p1_end = pts1
    height, width, channels = img.shape
    y_bottom = height
    
    mid_line = [[[p1_start[0], p1_start[1], p1_end[0], p1_end[1]],True,5]]
    mid_slopes, mid_angles, mid_intercepts, mid_y_intercepts = get_slopes_intercepts(img,mid_line)
    mid_slope = mid_slopes[0]
    mid_intercept = mid_intercepts[0]
    mid_angle = mid_angles[0]
    cv2.line(img, (p1_start[0], p1_start[1]), (p1_end[0], p1_end[1]), (0,255,0), 2)



    return mid_intercept, mid_slope, mid_angle

def recommend_direction(center, slope,angle,img):
    height, width, channels = img.shape
    if center is None or slope is None:
        return "unknown"
    slope = 1/slope
    if slope >= 0:
        angle = -angle
    print(center, slope,angle)
    image_width = width
    EDGE_THRESHOLD = 500

    edge_threshold_left = EDGE_THRESHOLD
    edge_threshold_right = image_width - EDGE_THRESHOLD

    if center < edge_threshold_left:
        return "left"
    elif center > edge_threshold_right:
        return "right"
    else:
        return "forward"

def recommend_turn(center, slope, angle,img):

    if center is None or slope is None or angle is None:
        return "unknown"
    slope = 1/slope

    if -1 < slope < 0:
        return "turn left"

    elif 0 <= slope < 1:
         return "turn right"

    else: 
         return "stay Dir"  
    

    

