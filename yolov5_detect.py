import cv2
import numpy as np
import time
import copy

def yolo_onnx(image, net):
    IMG_SIZE = 320
    print(cv2.__version__)
    
    #print(image.shape)
    blob = cv2.dnn.blobFromImage(image, 1/255.0, size = (IMG_SIZE,IMG_SIZE),swapRB = True, crop = False)

    net.setInput(blob)
    #print(blob.shape)
    start = time.time()
    outputs = net.forward()
    end = time.time()
    print("[INFO] yolo took {:.5} seconds".format(end - start))
    boxes = []
    confidences = []
    classIDs = []
    imgh,imgw = image.shape[:2]

    for out in outputs:
        #print(out.shape)
        for detection in out:
            confidence = detection[4]
            if confidence > 0.5:
                #print(detection[:4])
                scores = detection[5:]
                classID = np.argmax(scores)
                if scores[classID] > 0.25:
                    # print(detection)
                    confidences.append(float(confidence))
                    classIDs.append(classID)
                    x,y,w,h = detection[:4]
                    left = int((x-.5*w) * imgw/IMG_SIZE)
                    top = int((y-.5*h) * imgh/IMG_SIZE)
                    width = int(w * imgw/IMG_SIZE)
                    height = int(h * imgh/IMG_SIZE)
                    box = np.array([left, top, width, height])
                    boxes.append(box)

    if (len(boxes) == 0):
        return (image, None)
    indices = []
    #for box in enumerate(boxes):
    indices = cv2.dnn.NMSBoxes(boxes, confidences, 0.25, 0.3)
    # print(indices)
    centers = []
    cv2.circle(image, (400, 320), 10, (0, 255, 0), 4)
    if len(indices) > 0:
        for i in indices.flatten():
            (x, y) = (boxes[i][0], boxes[i][1])
            (w, h) = (boxes[i][2], boxes[i][3])
            center = (x + w/2, y + h/2)
            #color = [int(c) for c in colors[classIDs[i]]]
            # cv2.rectangle(image, (x, y), (x + w, y + h), (0,0,255), 2)
            cv2.circle(image, (int(center[0]), int(center[1])), 10, (255, 0, 0), 1)
            centers.append(center)
    x_avg = (centers[0][0] + centers[1][0])/2
    y_avg = (centers[0][1] + centers[1][1])/2
    cv2.circle(image, (int(x_avg), int(y_avg)), 10, (0, 0, 255), 4)
    avgs = (x_avg, y_avg)
    return (image, avgs)


def get_center_diffs_yolo(image, net):
    yolo = yolo_onnx(image=image.copy(), net=net)
    avgs = yolo[1]
    if avgs is None:
        return None
    diffs = {"y": 320 - avgs[1], "x": 400 - avgs[0]}
    return diffs
