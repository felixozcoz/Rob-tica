def trackObject(self, colorRangeMin=[0,0,0], colorRangeMax=[255,255,255]):
    #targetSize=??, target=??, catch=??, ...)
    finished = False
    targetFound = False
    targetPositionReached = False

    while not finished:
        # 1. search the most promising blob ..

        imagen = get_imagen()
        blob   = get_best_blob(imagen, colorRangeMin, colorRangeMax)

        while not targetPositionReached:
            # 2. decide v and w for the robot to get closer to target position
            a, d = get_blob_parameters(blob)
            if d == 0
                self.setSpeed(0,0)
                targetPositionReached = True
                targetFound = True
                finished = True
            else:
                v, w = getNewSpeed(a, d)
                self.setSpeed(v, w)

    return finished

def catch(self):
    # decide the strategy to catch the ball once you have reached the target position
    self.setNestSpeed(0,np.pi / 2)

def updateOdometry(self): #, additional_params?):
    #print('left_offset_length: %.2f, right_offset_length: %.2f' %(left_offset_length, right_offset_length))
    #print("\n==================================")
    #print("== Left Wheel ====================")
    #print("> Grades offset: %dº -> %dº (+%dº)" %(self.sI.value, left_encoder, left_offset))
    #print("> Length offset: %.5f" %(left_offset_length))
    #print("== Right Wheel ===================")
    #print("> Grades offset: %dº -> %dº (+%dº)" %(self.sD.value, right_encoder, right_offset))
    #print("> Length offset: %.5f" %(right_offset_length))
    #print("==================================")
    #print("wi = %.5f, wd = %.5f" %(wi, wd))
    #print('v: %.2f, w = %.2f' %(vw[0], vw[1]))
    #print("==================================")
    #print("> DeltaS: %.5f" %(delta_s))
    #print('delta_x: %.2f, delta_y: %.2f' %(delta_x, delta_y))
    #print("> Total Length (+/-): %.5f" %(self.totalLength.value))
    #print("== Ángulo ========================")
    #print('x: %.2f, y: %.2f, th: %.2f' %(self.x.value, self.y.value, np.rad2deg(self.th.value)))
    #print('delta_s: %.2f' %(delta_s))
    #print("==================================")
    #print("tIni: %.2f, tEnd: %.2f, tEnd-tIni: %.2f, tSleep: %.2f" %(tIni, tEnd, tIni-tEnd, self.P-(tEnd-tIni)))

def _get_best_blob(self, blobs, centered=False):
        # Si no hay blobs, no hay mejor
        if not blobs:
            return None
        # Criterio: más grande y más centrado si son iguales, sino el más grande
        best_blob = None
        for blob in blobs:
            # Filtro
            if blob.pt[1] >= self.cam_center.y:
                print("Current filtered blob:", blob.pt, blob.size)
                if not best_blob:
                    best_blob = blob
                    continue
                else:
                    if best_blob.size < blob.size:
                        best_blob = blob
                    elif (best_blob.size == blob.size) and (abs(best_blob.pt[0] - self.cam_center.x) > abs(blob.pt[0] - self.cam_center.x)):
                        best_blob = blob
        if best_blob:
            print("\nBest blob found: ", best_blob.pt[0], best_blob.pt[1], best_blob.size)
        return best_blob

        if not centered:
            for blob in blobs:
                # Filtro
                if blob.pt[1] >= self.cam_center.y:
                    print("Current filtered blob:", blob.pt, blob.size)
                    if not best_blob:
                        best_blob = blob
                        continue
                    else:
                        if best_blob.size < blob.size:
                            best_blob = blob
                        elif (best_blob.size == blob.size) and (abs(best_blob.pt[0] - self.cam_center.x) > abs(blob.pt[0] - self.cam_center.x)):
                            best_blob = blob
            if best_blob:
                print("\nBest blob found: ", best_blob.pt[0], best_blob.pt[1], best_blob.size)
        else:
            pt = [0,0]; size = 0; count = 0
            for blob in blobs:
                # Filtro
                if blob.pt[1] >= self.cam_center.y:
                    print("Current filtered blob:", blob.pt, blob.size)
                    if (self.cam_center.x - (self.cam_center.x//2)) <= blob.pt[0] and \
                        blob.pt[0] < self.cam_center.x + (self.cam_center.x//2):
                        if not best_blob:
                            best_blob = blob
                        pt[0] += blob.pt[0]
                        pt[1] += blob.pt[1]
                        size  += blob.size
                        count += 1

            if best_blob:
                best_blob.pt = (pt[0]/count, pt[1]/count)
                best_blob.size  = size/count
                print("\nBest blob found: ", best_blob.pt[0], best_blob.pt[1], best_blob.size)

        return best_blob


def getBestBlob(self, keypoints):
        # Si no hay blobs, no hay mejor
        # if not keypoints:
        #     return None
        # Criterio: más grande y más centrado si son iguales, sino el más grande
        best_kp = None
        for kp in keypoints:
            # Filtro
            if kp.pt[1] >= self.cam_center.y:
                print("Current filtered blob:", kp.pt, kp.size)
                if not best_kp:
                    best_kp = kp
                    continue
                else:
                    if best_kp.size < kp.size:
                        best_kp = kp
                    elif (best_kp.size == kp.size) and (abs(best_kp.pt[0]) > abs(kp.pt[0])):
                        best_kp = kp
        if best_kp:
            print("\nBest blob found: ", best_kp.pt[0], best_kp.pt[1], best_kp.size)
        # kp.pt[0] = x coordenate on the image 
        # kp.pt[0] = y coordenate on the image
        # kp.size = diameter of the blob
        return best_kp


        # Criterio: más grande y más centrado si son iguales, sino el más grande
        # 1. kp.pt = coordenadas (x,y) en la imagen.
        # 2. kp.size = diametro del blob
        best_blob    = max(enumerate(keypoints), key=lambda kp: (kp[1].size, -kp[1].pt[0]+self.cam_center.x))[1]
        best_blob.pt = (best_blob.pt[0] - self.cam_center.x, best_blob.pt[1] - self.cam_center.y)
        if best_blob.pt[1] >= self.cam_center.y:
            print("\nBest blob found: ", best_blob.pt[0], best_blob.pt[1], best_blob.size)
            return best_blob
        else:
            return None

def trackObject(self, colorRangeMin, colorRangeMax, showFrame=False):
    #if x >= -epsilon and x < epsilon:
    #    self.setSpeed(0, 0)
    #elif x < -epsilon:
    #    self.setSpeed(0, w)
    #    side = -1
    #elif x >= epsilon:
    #    self.setSpeed(0,-w)
    #    side = 1

    #if best_blob.pt[0] > (center[0]-30) and best_blob.pt[0] < (center[0]+30):
    #    self.setSpeed(3, 0)
    #elif best_blob.pt[0] < center[0]:
    #    self.setSpeed(3, np.pi/4)
    #    #if sense == 0:
    #    #    self.setSpeed(2, np.pi/4)
    #    #    sense = -1
    #    #elif sense == 1:
    #    #    self.setSpeed(2,0)
    #    side = 1
    #elif best_blob.pt[0] > center[0]:
    #    self.setSpeed(3, -np.pi/4)
    #    #if sense == 0:
    #    #    self.setSpeed(2, -np.pi/4)
    #    #    sense = 1
    #    #elif sense == -1:
    #    #    self.setSpeed(2,0)
    #    side = -1

    #if best_blob.pt[0] < center[0]:
    #    side = 1
    #    if sense == 0:
    #        sense = -1
    #else:
    #    side = -1
    #    if sense == 0:
    #        sense = 1
    #    
    #if sense == -1 and best_blob.pt[0] < center[0]:
    #    self.setSpeed(0,  np.pi / 4)
    #elif sense == 1 and best_blob.pt[0] > center[0]:
    #    self.setSpeed(0, -np.pi / 4)
    #else:
    #    self.setSpeed(0,0)