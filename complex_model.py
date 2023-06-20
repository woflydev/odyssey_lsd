import tensorflow as tf
import cv2

model = tf.keras.models.load_model("data/complex.h5")

def process_data(frame):
    frame = cv2.resize(frame, [360, 180])
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    frame = frame / 255
    tensor = tf.convert_to_tensor([frame])
    output = model.predict(tensor)[0] * 255
    return output

cv2.imwrite("peppa pig honk honk.png",process_data(cv2.imread("data/test1.png")))

    