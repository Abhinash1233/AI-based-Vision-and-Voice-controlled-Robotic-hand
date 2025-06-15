# === IMPORTS ===
import multiprocessing
import cv2
import numpy as np
import time
from multiprocessing import Process, Queue  
import serial
import mediapipe as mp
import traceback
import cv2
from ultralytics import YOLO
from roboticstoolbox import DHRobot, RevoluteMDH
from spatialmath import SE3
import numpy as np
import matplotlib.pyplot as plt
import os
import csv

# === VOICE MODULE ===
import pyttsx3
import datetime
import speech_recognition as sr
import wikipedia
import webbrowser
from datetime import date

def voice_assistant(conn1):
    

    engine = pyttsx3.init('sapi5')
    voices = engine.getProperty('voices')
    engine.setProperty('voice', voices[1].id)
    engine.setProperty('rate', 150)

    def speak(audio):
        
            engine.say(audio)
            engine.runAndWait()



    def draw_status_window(text, color):
        img = 255 * np.ones((200, 400, 3), dtype=np.uint8)
        cv2.putText(img, text, (30, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 3)
        cv2.imshow("Voice Assistant", img)
        cv2.waitKey(1)

    def takeCommand():
        r = sr.Recognizer()
        try:
            with sr.Microphone() as source:
                draw_status_window("Mic: Listening...", (0, 255, 0))
                r.pause_threshold = 1
                r.energy_threshold = 400
                audio = r.listen(source, timeout=5)
                draw_status_window("Mic: Recognizing...", (255, 255, 0))
                query = r.recognize_google(audio, language='en-in')
                print(f"User said: {query}")
                return query
        except sr.WaitTimeoutError:
            draw_status_window("Mic: Timeout", (0, 0, 255))
            return "None"
        except sr.UnknownValueError:
            draw_status_window("Mic: Didn't Catch", (0, 255, 255))
            return "None"
        except sr.RequestError:
            draw_status_window("Mic: API Error", (0, 0, 255))
            return "None"
        except Exception as e:
            draw_status_window("Mic: Not Detected", (0, 0, 255))
            time.sleep(2)
            return "None"

    try:
        speak("Initializing voice assistant.")
        hour = int(datetime.datetime.now().hour)
        if hour <12:
            speak("Good morning Sir , It's a nice day to start your work.")
        elif hour >=12 and hour < 18:
            speak("Good afternoon Sir.")
        elif hour >=18 and hour < 21:
            speak("Good Evening Sir.")
        else:
            speak("Good night Sir")
        speak("How may I help you")

        while True:
            query = takeCommand().lower()
            if 'how are you' in query:
                speak("I am fine sir what I can do for you?..")
            elif 'wikipedia' in query:
                speak("Searching...")
                query = query.replace("wikipedia", "")
                results = wikipedia.summary(query, sentences=3)
                speak("According to wikipedia")
                speak(results)
            elif 'open youtube' in query:
                webbrowser.open("https://youtube.com")
            elif 'date' in query:
                current_date = date.today()
                speak(str(current_date))
            elif 'time' in query:
                now = datetime.datetime.now()
                current_time = now.strftime("It's %I:%M %p.")
                speak(current_time)
            elif 'hand detection' in query or 'and detection' in query:
                conn1.put("start")
                speak('starting hand detection')
            elif 'stop detection' in query:
                conn1.put('stop')    
            elif 'object detection' in query:
                conn1.put('object')
                speak("Starting object detection")
            elif 'face detection' in query:
                conn1.put('face')
                speak("Starting Face detection")         
            elif 'ok thanks' in query:
                speak("it's my pleasure Sir")
                conn1.put('Exite')
                break

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except Exception as e:
        traceback.print_exc()
        while True:
            draw_status_window("Mic: Not Connected", (0, 0, 255))
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cv2.destroyWindow("Voice Assistant")

# === CAMERA MODULE ===
def camera_pid_tracking(conn1):

    l = 90
    r = 90
    s = 90

    Kp = 0.01
    Ki = 0
    Kd = 0.0005
    p = 0.01
    d = 0
    HOME_CSV = 'home_position.csv'
    LOG_EVERY = 5
    LOG_EVERY2 = 150
    frame_count = 0
    previous_error = 0
    previous_error1 = 0
    integral = 0
    integral1 = 0
    last_time = time.time()

    arduino_port = "COM9"
    baud_rate = 9600

    mp_hands = mp.solutions.hands
    mp_face_detection = mp.solutions.face_detection

    hands = mp_hands.Hands(min_detection_confidence=0.5, min_tracking_confidence=0.5)
    face_detection = mp_face_detection.FaceDetection(min_detection_confidence=0.85)

    model = YOLO('yolo_box_model/box_detector/weights/best.pt')

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 720)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)

    try:
        arduino = serial.Serial(arduino_port, baud_rate, timeout=1)
        time.sleep(2)
        print("Connected to Arduino")
    except Exception as e:
        print(f"Failed to connect: {e}")
        exit()
    # ---------------------- Define 7-DOF Robot ----------------------
    robot = DHRobot([
        RevoluteMDH(d=11,   a=0,    alpha=0),
        RevoluteMDH(d=0,    a=0,    alpha=np.deg2rad(90)),
        RevoluteMDH(d=19.5, a=0,    alpha=np.deg2rad(-90), offset=np.deg2rad(90)),
        RevoluteMDH(d=0,    a=0,    alpha=np.deg2rad(90),  offset=np.deg2rad(90)),
        RevoluteMDH(d=0,    a=18,   alpha=0,               offset=np.deg2rad(90)),
        RevoluteMDH(d=0,    a=0,    alpha=np.deg2rad(90),  offset=np.deg2rad(180)),
        RevoluteMDH(d=10.5, a=0,    alpha=0)
    ], name="ModifiedRRRRobot")

    offsets = [90, 85, 90, 90, 100, 90, 90]
    # ---------------------- Functions ----------------------
    def save_home_angles(s, l, r):
        with open(HOME_CSV, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([s, l, r])
        print("[INFO] Home angles saved to CSV.")
    def degrees_to_sim_angles(real_angles):
        return [real - offset for real, offset in zip(real_angles, offsets)]

    def sim_to_real_angles(sim_angles):
        return [sim + offset for sim, offset in zip(sim_angles, offsets)]

    '''def plot_robot_axes(robot, q_rad, title):
        T_all = robot.fkine_all(q_rad)
        colors = ['red', 'green', 'blue', 'orange', 'purple', 'cyan', 'magenta']
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_title(title)
        for i, T in enumerate(T_all):
            pos = T.t
            R = T.R
            z_axis = R[:, 2]
            x_axis = R[:, 0]
            color = colors[i % len(colors)]
            ax.quiver(pos[0], pos[1], pos[2], z_axis[0], z_axis[1], z_axis[2], color=color, length=5, normalize=True)
            ax.text(pos[0] + z_axis[0]*5, pos[1] + z_axis[1]*5, pos[2] + z_axis[2]*5, f'Z{i+1}', color=color, fontsize=8)
            ax.quiver(pos[0], pos[1], pos[2], x_axis[0], x_axis[1], x_axis[2], color=color, length=5, normalize=True, linestyle='dotted')
            ax.text(pos[0] + x_axis[0]*5, pos[1] + x_axis[1]*5, pos[2] + x_axis[2]*5, f'X{i+1}', color=color, fontsize=8)
            ax.text(pos[0], pos[1], pos[2], f'J{i+1}', fontsize=9, color='black')
        ax.set_xlabel("X (mm)")
        ax.set_ylabel("Y (mm)")
        ax.set_zlabel("Z (mm)")
        ax.set_xlim(-30, 30)
        ax.set_ylim(-30, 30)
        ax.set_zlim(0, 60)
        ax.view_init(elev=30, azim=45)
        plt.tight_layout()
        plt.show()
        '''
    def forward_kinematics(real_angles):
        sim_angles = degrees_to_sim_angles(real_angles)
        q_rad = np.deg2rad(sim_angles)
        #robot.plot(q_rad, block=False)
        #plot_robot_axes(robot, q_rad, "Input Joint Angles Configuration")
        T_ee = robot.fkine(q_rad)
        return T_ee, q_rad

    def get_object_position(T_ee, distance):
        pos = T_ee.t
        z_axis = T_ee.R[:, 2]
        object_pos = pos + distance * z_axis
        print("\n=== FK Result ===")
        print("Gripper Position (mm):", np.round(pos, 2))
        print("Gripper Z-axis Vector:", np.round(z_axis, 4))
        print("Object Position from LiDAR (mm):", np.round(object_pos, 2))
        return object_pos, z_axis

    def build_orientation(z_axis):
        z = z_axis / np.linalg.norm(z_axis)
        up = np.array([0, 0, 1]) if abs(z[2]) < 0.99 else np.array([0, 1, 0])
        x = np.cross(up, z); x /= np.linalg.norm(x)
        y = np.cross(z, x); y /= np.linalg.norm(y)
        return np.column_stack((x, y, z))

    def inverse_kinematics(target_position, orientation, q_start):
        T_target = SE3.Rt(orientation, target_position)
        ik_result = robot.ikine_LM(T_target, q0=q_start, ilimit=1000, slimit=1000, tol=1e-6)
    
        if not ik_result.success:
            print("❌ IK failed to reach object position.")
            return None
        q_solution = ik_result.q
        q_solution_deg = np.clip(np.rad2deg(q_solution), -90, 90)
        real_solution_angles = sim_to_real_angles(q_solution_deg)

        print("\n✅ IK Solution to Object Position (Simulation Angles):")
        for i, angle in enumerate(q_solution_deg):
            print(f"Joint J{i+1} (Sim): {angle:.2f}°")
    
        print("\n🦾 Real Robot Angles (Send to Robot):")
        for i, angle in enumerate(real_solution_angles):
            print(f"Joint J{i+1}: {angle:.2f}°")

        #robot.plot(np.deg2rad(q_solution_deg), block=False)
        #plot_robot_axes(robot, np.deg2rad(q_solution_deg), "IK Solution Configuration")
    
        return real_solution_angles

    def send_angles(angles):
        if len(angles) != 9:
            print("Error: Please provide exactly 9 angles.")
            return
        angle_string = " ".join(map(str, angles)) + "\n"
        arduino.write(angle_string.encode())
        print(f"Sent: {angle_string.strip()}")
        
        #response = arduino.readline().decode().strip()
    
        #if response:
           # print(f"Arduino: {response}")
    def send_three_angles(angles):
        if len(angles) != 3:
            print("Error: Please provide exactly 3 angles.")
            return
        angle_string = " ".join(map(str, angles)) + "\n"
        arduino.write(angle_string.encode())
        #print(f"Sent: {angle_string.strip()}")
        #response = arduino.readline().decode().strip()
        #if response:
         #   print(f"Arduino: {response}")
    r = 100
    l = 180-r
    send_angles([90,82,90,90,l,r,90,0,3000])
    
    data = ['stop']

    direction = 1 
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.flip(frame,0)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results1 = model.predict(source=frame, conf=0.75, verbose=False)
        results = hands.process(rgb_frame)

        h, w, _ = frame.shape
        xp = w // 2
        yp = h // 2
        frame_center = (xp, yp)
        cv2.circle(frame, frame_center, 20, (255, 0, 0), 2)
        response = arduino.readline().decode().strip()
        print(f"Arduino: {response}")
        msg = None
        if not conn1.empty():
            msg = conn1.get()
            if msg == 'start':
                data[0] ='start'
                
            elif msg == 'stop':
                data[0] = 'stop'
            elif msg == 'object':
                data[0] = 'object'
                l = 180-20
                r = 20
                s=90
                send_angles([s,70,180,30,l,r,90,0,3000])
            elif msg =='Home':
                data[0] = 'Home'        
            elif msg =='Exite':
                data[0] = 'Exite'              
            elif msg == 'face':
                data[0] = 'face'
           
                # Try to load saved home angles
                def load_home_angles():
                    if os.path.exists(HOME_CSV):
                        with open(HOME_CSV, mode='r') as file:
                            reader = csv.reader(file)
                            row = next(reader, [])
                            if len(row) == 3:
                                try:
                                    return [int(v) for v in row]
                                except:
                                    pass  # Fall through to default if conversion fails
                    return None  # No valid data
                home_angles = load_home_angles()
                if home_angles is not None:
                    s, l, r = home_angles
                    print("[INFO] Loaded saved home angles from CSV.")
                else:
                    r = 60
                    l = 180 - r
                    s = 180
                    print("[INFO] No saved angles found, using default.")
                send_angles([s, 50, 180, 40, l, r, 90,140, 3000])
                send_angles([s, 50, 180, 40, l, r, 90,0, 3000])

        print('data =', data[0])
        
        if data[0]=='start':
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    x_min, y_min = w, h
                    x_max, y_max = 0, 0
                    for landmark in hand_landmarks.landmark:
                        x, y = int(landmark.x * w), int(landmark.y * h)
                        x_min, y_min = min(x_min, x), min(y_min, y)
                        x_max, y_max = max(x_max, x), max(y_max, y)
                x2 = (x_min + x_max) // 2
                y2 = (y_min + y_max) // 2
                hand_center = (x2, y2)

                error = y2 - yp
                error1 = x2 - xp
                current_time = time.time()
                dt = current_time - last_time if current_time != last_time else 1e-16

                integral += error * dt
                integral1 += error1 * dt
                derivative = (error - previous_error) / dt
                derivative1 = (error1 - previous_error1)

                output = int(Kp * error + Ki * integral + Kd * derivative)
                output1 = int(p * error1 + Ki * integral1 + d * derivative1)

                l += output
                r -= output
                s += output1

                previous_error = error
                previous_error1 = error1
                last_time = current_time

                cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
                cv2.circle(frame, hand_center, 5, (0, 0, 0), 1)


                Ai_input =[s,l,r]
                angles = Ai_input
                send_three_angles(angles)
            #if results.multi_hand_landmarks is None:
             #   l += 1 * direction
              #  if l >= 160:
               #     direction = -1
                #elif l <= 20:
                 #   direction = 1

               # r = 180 - l  # keep torque balanced
               # s = 90
                #input = [s,l,r,60,180,3,90]
               # send_seven_angles(input)
        elif data[0] == 'object':
            for results1 in results1:
                boxes = results1.boxes
                for box in boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    conf = float(box.conf[0])
                    cls = int(box.cls[0])
                    label = model.names[cls]  # Get class name
                    X_center = (x1+x2)/2
                    Y_center = (y1+y2)/2
            # Draw rectangle and label
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, f"{label} {conf:.2f}", (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    if X_center > (xp+30):
                        s+=1
                    if X_center < (xp-30):
                        s-=1
                    if Y_center > (yp+40):
                        r -= 1
                        l += 1    
                    if Y_center < (yp-40):
                        r+= 1
                        l-=1    
            s = max(0, min(180, s))
            l = max(0, min(180, l))
            r = max(0, min(180, r))       
            Ai_input =[s,l,r]
            angles = Ai_input
            send_three_angles(angles)
            frame_count +=1
            if frame_count % LOG_EVERY2 == 0:
                lidar_values = []
                for i in range(40):  # collect exactly 40 values
                            response = arduino.readline().decode().strip()
                            lidar_values.append(int(response))

                if lidar_values is not None:
                        avg_lidar = sum(lidar_values) / len(lidar_values)
                        
                        # You can now use avg_lidar in further calculations or logging
                angles_input = [s,70,180,30,r-20,90,90]
                real_angles = [float(x) for x in angles_input]
                if len(real_angles) != 7:
                            raise ValueError("You must enter exactly 7 joint angles.")
                distance_input = avg_lidar -1
                T_ee, q_start = forward_kinematics(real_angles)
                object_pos, z_axis = get_object_position(T_ee, distance_input)
                orientation = build_orientation(z_axis)
                Ik_angles = inverse_kinematics(object_pos, orientation, q_start)
                rev = 180-int(Ik_angles[4]) 

                send_angles([s-5,int(Ik_angles[1]),180,int(Ik_angles[3]),rev,int(Ik_angles[4]),90,0,4000])

                frame_count = 0    
                
                data[0]='Home'

        elif data[0] == 'face':
            detections = face_detection.process(rgb_frame)
            if detections.detections:
                for detection in detections.detections:
                    bboxC = detection.location_data.relative_bounding_box
                    ih, iw, _ = frame.shape
                    x = int(bboxC.xmin * iw)
                    y = int(bboxC.ymin * ih)
                    w_box = int(bboxC.width * iw)
                    h_box = int(bboxC.height * ih)

                x2 = x + w_box // 2
                y2 = y + h_box // 2
                face_center = (x2, y2)

                error = y2 - yp
                error1 = x2 - xp
                current_time = time.time()
                dt = current_time - last_time if current_time != last_time else 1e-16

                integral += error * dt
                integral1 += error1 * dt
                derivative = (error - previous_error) / dt
                derivative1 = (error1 - previous_error1) / dt

                output = int(Kp * error + Ki * integral + Kd * derivative)
                output1 = int(p * error1 + Ki * integral1 + d * derivative1)

                l += output
                r -= output
                s += output1

                previous_error = error
                previous_error1 = error1
                last_time = current_time

                cv2.rectangle(frame, (x, y), (x + w_box, y + h_box), (30, 65, 158), 1)
                cv2.circle(frame, face_center, 5, (0, 0, 255), 1)
                cv2.putText(frame, 'Face', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 255), 2)
                s = max(0, min(180, s))
                l = max(0, min(180, l))
                r = max(0, min(180, r))  
                Ai_input = [s, l, r]
                angles = Ai_input
                send_three_angles(angles)
                frame_count += 1
                if frame_count % LOG_EVERY == 0:
                    save_home_angles(s, l, r)

        elif data[0] == 'Home':
            send_angles([90,60,180,20,180-40,40,90,140,3000])
            data[0] = 'stop'

        response = arduino.readline().decode().strip()
        print(f"Arduino: {response}")
        cv2.imshow("MediaPipe Hand Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
    send_angles([90,82,90,90,80,180-80,90,140,3000])
    time.sleep(3)
    arduino.close()
    print("Connection closed.")

# === MAIN MULTIPROCESSING ===
def main():
    q = Queue()  # Single-direction communication
    p1 = Process(target=voice_assistant, args=(q,))
    p2 = Process(target=camera_pid_tracking, args=(q,))

    p1.start()
    p2.start()

    p1.join()
    p2.join()


if __name__ == "__main__":
    multiprocessing.set_start_method('spawn')  # Recommended on Windows
    main()