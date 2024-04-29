import cv2
import numpy as np

# https://cullensun.medium.com/agglomerative-clustering-for-opencv-contours-cd74719b678e
def calculate_contour_distance(contour1, contour2): 
    x1, y1, w1, h1 = cv2.boundingRect(contour1)
    c_x1 = x1 + w1/2
    c_y1 = y1 + h1/2

    x2, y2, w2, h2 = cv2.boundingRect(contour2)
    c_x2 = x2 + w2/2
    c_y2 = y2 + h2/2

    return max(abs(c_x1 - c_x2) - (w1 + w2)/2, abs(c_y1 - c_y2) - (h1 + h2)/2)

def merge_contours(contour1, contour2):
    return np.concatenate((contour1, contour2), axis=0)

def agglomerative_cluster(contours, threshold_distance=40.0):
    current_contours = list(contours)
    while len(current_contours) > 1:
        min_distance = None
        min_coordinate = None

        for x in range(len(current_contours)-1):
            for y in range(x+1, len(current_contours)):
                distance = calculate_contour_distance(current_contours[x], current_contours[y])
                if min_distance is None:
                    min_distance = distance
                    min_coordinate = (x, y)
                elif distance < min_distance:
                    min_distance = distance
                    min_coordinate = (x, y)

        if min_distance < threshold_distance:
            index1, index2 = min_coordinate
            current_contours[index1] = merge_contours(current_contours[index1], current_contours[index2])
            del current_contours[index2]
        else: 
            break

    return current_contours

def contoursEdgeDetection(image):
    bilateral = cv2.bilateralFilter(image, 5, 175, 175)
    edged = cv2.Canny(bilateral, 75, 200)
    _, contours = cv2.findContours(edged, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    circular_contours = []
    for contour in contours:
        # Gets percentage of area that can
        # be approximated with polygon.
        # If low enough (< 30%) then it's roughly circular.
        approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)
        area = cv2.contourArea(contour)
        if len(approx) > 8 and area > 30:
            circular_contours.append(contour)
    return contour

def hsvThreshold(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # bilateral = cv2.bilateralFilter(hsv, 5, 180, 175)

    # orange_low = (150, 150, 100)
    # orange_high = (3, 255, 255)

    orange_low = (170, 150, 100)
    orange_high = (30, 255, 255)

    threshold = cv2.inRange(hsv, orange_low, (180, orange_high[1], orange_high[2])) \
        + cv2.inRange(hsv, (0, orange_low[1], orange_low[2]), orange_high)

    # hsv[:, :, 0] = 0
    # # hsv[:, :, 1] = 0
    # hsv[:, :, 2] = 0
    
    contours, hierarchy = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    return image, contours


# runPipeline() is called every frame by Limelight's backend.
def runPipeline(image, llrobot):
    image = image[int(image.s
    hape[0]/2):]
    img, contours = hsvThreshold(image)

    threshold_area = 30
    contours = [cnt for cnt in contours if cv2.contourArea(cnt) > threshold_area]

    print('# contours', len(contours))

    llpython = [0,0,0,0,0,0,0,0]


    # contours = agglomerative_cluster(contours, threshold_distance=5)
    if len(contours) > 0:
        cv2.drawContours(image, contours, -1, (0, 255, 0), 3)
        largestContour = max(contours, key=cv2.contourArea)
        
        x,y,w,h = cv2.boundingRect(largestContour)
        cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,255),2)
        cv2.circle(image, (int(x+w/2), int(y+h/2)), 3, (0, 0, 255), 5)

        area = cv2.contourArea(largestContour)

        # cv2.circle(image, (int(image.shape[1]/2), int(image.shape[0]/2)), 3, (255, 0, 0), 5)
 
        # TODO: Homography and/or lookup table
        note_mx_when_facing = 112

        centering_error = int(x+w/2) - note_mx_when_facing

        print(centering_error)
        llpython[0:2] = [1, centering_error]

    # LL sees note -> gets centr of bounding rect.
    # Maps contour width to target cx.
    # Return current cx - target cx
   
    return np.array([[]]), img, llpython








"""



    filtered_contours = []
    threshold_area = 100

    contours = [cnt for cnt in contours if cv2.contourArea(cnt) > threshold_area]

    llpython = [0,0,0,0,0,0,0,0]

    if len(contours) > 0:
        cv2.drawContours(image, contours, -1, (0, 255, 0), 3)
        largestContour = max(contours, key=cv2.contourArea)
        
        x,y,w,h = cv2.boundingRect(largestContour)
        cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,255),2)
        cv2.circle(image, (int(x+w/2), int(y+h/2)), 3, (0, 0, 255), 5)

        area = cv2.contourArea(largestContour)

        cv2.circle(image, (int(image.shape[1]/2), int(image.shape[0]/2)), 3, (255, 0, 0), 5)
 
        # TODO: Homography and/or lookup table
        note_mx_when_facing = 112

        centering_error = int(x+w/2) - note_mx_when_facing

        print(centering_error)

        # TODO: Cleaner way of ordering contour data
        llpython[:2] = [1,centering_error]
"""