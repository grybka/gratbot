#a collection of feature to vector encoders

import numpy as np

def image_label_to_vector(image_label):
    valid_labels=["face","person","laptop","cup","book","sports ball","chair"]
    ret=np.zeros(len(valid_labels))
    if image_label not in valid_labels:
        return ret
    ret[valid_labels.index(image_label)]=1.0
    image_to_label_matrix=np.eye(len(valid_labels))
    #faces and people go together
    image_to_label_matrix[valid_labels.index("face"),valid_labels.index("person")]=0.5
    image_to_label_matrix[valid_labels.index("person"),valid_labels.index("face")]=0.5
    return np.dot(image_to_label_matrix,ret)

class FaceEncoder():
    def __init__(self):
        ...

    def encode(self,image):
        ...
