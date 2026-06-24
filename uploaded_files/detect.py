import torch
import torchvision
from torchvision.models.detection import fasterrcnn_resnet50_fpn, FasterRCNN_ResNet50_FPN_Weights
from PIL import Image
import torchvision.transforms as T
import cv2
import numpy as np

# 1. Load the pre-trained model
weights = FasterRCNN_ResNet50_FPN_Weights.DEFAULT
model = fasterrcnn_resnet50_fpn(weights=weights)
model.eval() # Set model to evaluation mode

# 2. Load and preprocess the image
# REPLACE 'test_image.jpg' with the actual filename of your image
img_path = 'test_image.jpg' 
img = Image.open(img_path).convert('RGB')
transform = T.Compose([T.ToTensor()])
img_tensor = transform(img)

# 3. Perform Inference
with torch.no_grad():
    prediction = model([img_tensor])

# 4. Process and Draw Results
# We only look at boxes with a confidence score > 0.8 (80%)
threshold = 0.8
img_cv = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)

for box, score, label in zip(prediction[0]['boxes'], prediction[0]['scores'], prediction[0]['labels']):
    if score > threshold:
        x1, y1, x2, y2 = box.numpy().astype(int)
        cv2.rectangle(img_cv, (x1, y1), (x2, y2), (0, 255, 0), 2)
        print(f"Detected object with confidence {score:.2f} at {box}")

# 5. Save the output
cv2.imwrite('output.jpg', img_cv)
print("Detection complete. Check 'output.jpg' for results.")
