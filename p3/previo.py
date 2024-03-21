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

#######################################################
# AQUI!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#######################################################

Ya ta

Centra la bola, una vez centrada deja de girar
Y sigue recto, si por algun casual la bola se desplaza, cuando va recto, desiste y vuelve a girar
Pero eso es una condición de if de igualdad de tranformaciones evaluando primero que esté alineada en el eje X, no??? que eso es lo que estaba ya

Si si lo entiendo, pero no sería lo mismo que

if no centrada:
    gira
else: # centrada
    recto

sin el if último, sólo con Transform,
Pero POSITION ERROR es cuando sólo utiliza la velocidad Tangencial, qué influye eso en centrarla si se supone que ya estaría centrada. Y en caso de 
que se mueva volvería a girar poniendo v=0

si y no
si ponemos el margen en POSITION ERROR si 
pero ya no lo centrará tan bien
porque dará por valido que la pelota esta centrada a 3/4 de la imagen

pera estoy pensando
uhm
estas diciendo
poner el v dentro del else del primer ==?
porque si ya esta centrado pues simplemente sigues recto y si se mueve, ya no entra en el else


es el if else que he puesto justo encima. Now we are talking, si podria valer eso
Porque siempre evalúa primero que esté centrada, y mientras no lo esté no irá recto

Lo unico que me da cosa es a ver si va fluido entre seguir recto y rotar, aunque en el video no va tan tan

ya ya, pero a estas alturas por lo menos tendríamos algo funcional, luego vendría hacerlo más fluído, cómo lo ves dejarlo como he dicho.

es para quitar los blobs de arriba, si si, eso está bien

que por cierto esta mal calculado
es 3/4 * self.cam_center.y no (1 - 3/4)*self.cam_center.y

está puesto así: 3/4 * self.cam_center.y
ojo no pongais todos los cambios a pelo por si deja de moverse derrepente que ayer pasó, ir pasito a pasito por si hay algo que la version de ese python no traga y nos esta 
jodiendo

Lo que hay en Robot.py ayer funcionaba decente.
Hacemos una copia de Robot.py que se llame Robot3.py y modificamos Robot.py
la hago ahora


if not targetRotationReached:
    # Targer rotation
    blob_transform = Transform(Vector2(x=best_blob.pt[0], y=0))
    if not rotation_transform == blob_transform:
        # Ultima posicion vista. Si sign < 0, esta a la izquierda, si sign > 0, esta a la derecha
        side = np.sign(blob_transform.position.x)
        # Velocidades. Si sign < 0 -> w > 0 (-1*sign*w = -1*-1*w), si sign > 0 -> w < 0 (-1*sign*w = -1*1*w)
        w = self.fw_max(blob_transform.position.x)
    else:
        targetRotationReached = True
else:
    # Target position
    blob_transform = Transform(Vector2(x=0, y=best_blob.pt[1]))
    #if targetRotationReached and blob_transform == cam_center_transform:
    if not position_transform == blob_transform:
        # Calculamos el area del blob
        # blob_area = np.pi * (best_blob.size/2)**2
        # Calculamos la velocidad correspondiente (siempre positiva, el area es inversamente proporcional al area
        # Calcular con la distancia al borde de la imagen
        v = 3 # self.fv(blob_area)
    else:
        print("Fin tracking")
        return True
    if not (best_blob.pt[0] > -self.blob_detector_xmin and best_blob.pt[0] < self.blob_detector_xmin):
        targetRotationReached = False
    
    
