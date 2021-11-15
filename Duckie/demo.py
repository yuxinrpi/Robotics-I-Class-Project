import cv2
import mss
import sys
import numpy as np
import time

# Display barcode and QR code location
def display(im, bbox, data):
    bbox = bbox[0]
    points = []
    for i in bbox:
        points.append((int(i[0]),int(i[1])))
    bbox = points
    print(tuple(bbox[0]))
    cv2.line(im, tuple(bbox[0]), tuple(bbox[1]), (255,0,0), 3)
    cv2.line(im, tuple(bbox[1]), tuple(bbox[2]), (255,0,0), 3)
    cv2.line(im, tuple(bbox[2]), tuple(bbox[3]), (255,0,0), 3)
    cv2.line(im, tuple(bbox[3]), tuple(bbox[0]), (255,0,0), 3)
    cv2.putText(frame, "{}".format(data), tuple(bbox[2]), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0),2)
    # Display results
    cv2.imshow("original", im)


last = time.time()
i = 0
qrDecoder = cv2.QRCodeDetector()
with mss.mss() as sct:
    # Part of the screen to capture
    toppos = 280
    monitor = {"top": toppos, "left": 0, "width": 900, "height": 600}    
    while "Screen capturing":
        frame = np.array(sct.grab(monitor))
        cv2.putText(frame, "fps: {:2.1f}".format(1/(time.time()-last)), (20,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0),2)
        
        
        #cv2.imshow('AAA', frame)
        try:
            data,bbox,rectifiedImage = qrDecoder.detectAndDecode(frame)
        except:
            pass
        if len(data)>0:
            print("Decoded Data : {}".format(data))
            display(frame, bbox, data)
            print(bbox)
            rectifiedImage = np.uint8(rectifiedImage);
            cv2.imshow("Rectified QRCode", rectifiedImage);
        else:
            cv2.imshow('original',frame)
 
 
        #time.sleep(0.1)
        i += 1
        last = time.time()
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break        